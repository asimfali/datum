//! Pictures without a window: the viewer's renderer in an offscreen pass, read back as RGBA and
//! written as PNG. For the CLI, MCP and tests; without a GPU, wgpu's software adapters do.

use std::sync::Arc;

use egui_wgpu::wgpu;

use datum_graph::Value;

use crate::camera::Camera;
use crate::render::{Frame, Renderer};
use crate::scene::{Geometry, Scene};

const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
const SAMPLES: u32 = 4;
/// The viewport's background.
pub const BACKGROUND: [f64; 3] = [0.93, 0.94, 0.96];

/// A device without a window: any adapter wgpu finds, software ones included.
fn device() -> Result<(wgpu::Device, wgpu::Queue), String> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
        .map_err(|e| format!("no GPU adapter, not even a software one: {e}"))?;
    pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("datum_view_headless"),
        required_limits: wgpu::Limits::downlevel_defaults().using_resolution(adapter.limits()),
        ..Default::default()
    }))
    .map_err(|e| format!("the GPU device could not be opened: {e}"))
}

/// RGBA pixels, `width × height`, of `scene` seen by `camera`.
pub fn picture(scene: &Scene, camera: &Camera, width: u32, height: u32) -> Result<Vec<u8>, String> {
    let (device, queue) = device()?;
    let anchor = scene.anchor();
    let radius = scene.bounds.map_or(1.0, |(lo, hi)| (hi - lo).dot(hi - lo).sqrt() * 0.5);
    let (view_projection, eye) = camera.matrices(anchor, f64::from(width) / f64::from(height), radius);
    let frame = Frame { version: 1, geometry: Arc::new(Geometry::new(scene, anchor)), view_projection, eye };
    let mut renderer = Renderer::new(&device, FORMAT, DEPTH, SAMPLES);
    let size = wgpu::Extent3d { width, height, depth_or_array_layers: 1 };
    let texture = |format, samples, usage| {
        device.create_texture(&wgpu::TextureDescriptor {
            label: Some("datum_view_capture"),
            size,
            mip_level_count: 1,
            sample_count: samples,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats: &[],
        })
    };
    let resolved = texture(FORMAT, 1, wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC);
    let multisampled = texture(FORMAT, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT);
    let depth = texture(DEPTH, SAMPLES, wgpu::TextureUsages::RENDER_ATTACHMENT);
    let (resolved_view, ms_view, depth_view) =
        (resolved.create_view(&Default::default()), multisampled.create_view(&Default::default()), depth.create_view(&Default::default()));
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("datum_view_capture") });
    renderer.prepare(&device, &queue, &frame);
    {
        let [r, g, b] = BACKGROUND;
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("datum_view_capture"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &ms_view,
                depth_slice: None,
                resolve_target: Some(&resolved_view),
                ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color { r, g, b, a: 1.0 }), store: wgpu::StoreOp::Discard },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &depth_view,
                depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Discard }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        renderer.paint(&mut pass);
    }
    let row = width * 4;
    let padded = row.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT) * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let read = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("datum_view_read"),
        size: u64::from(padded) * u64::from(height),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    encoder.copy_texture_to_buffer(
        resolved.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &read,
            layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(padded), rows_per_image: Some(height) },
        },
        size,
    );
    let submission = queue.submit([encoder.finish()]);
    let slice = read.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device.poll(wgpu::PollType::Wait { submission_index: Some(submission), timeout: None }).map_err(|e| format!("waiting for the picture: {e}"))?;
    rx.recv().map_err(|e| e.to_string())?.map_err(|e| format!("reading the picture back: {e}"))?;
    let data = slice.get_mapped_range().map_err(|e| format!("reading the picture back: {e}"))?;
    Ok(data.chunks(padded as usize).take(height as usize).flat_map(|r| &r[..row as usize]).copied().collect())
}

/// A PNG of `values` (bodies and sketches) in standard view `view`, fitted to fill the picture:
/// what `datum snapshot` writes and the MCP tool `render` returns.
pub fn snapshot(values: &[&Value], view: &str, width: u32, height: u32) -> Result<Vec<u8>, String> {
    let mut camera = Camera::view(view)?;
    if width == 0 || height == 0 {
        return Err("the picture needs a width and a height".into());
    }
    let scene = Scene::new(values, &mut Default::default());
    camera.fit(scene.bounds);
    let rgba = picture(&scene, &camera, width, height)?;
    let mut out = Vec::new();
    png(&rgba, width, height, &mut out)?;
    Ok(out)
}

fn png(rgba: &[u8], width: u32, height: u32, out: impl std::io::Write) -> Result<(), String> {
    let mut e = png::Encoder::new(out, width, height);
    e.set_color(png::ColorType::Rgba);
    e.set_depth(png::BitDepth::Eight);
    let mut w = e.write_header().map_err(|e| e.to_string())?;
    w.write_image_data(rgba).map_err(|e| e.to_string())
}
