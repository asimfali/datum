//! Pictures without a window: the viewer's renderer in an offscreen pass, read back as RGBA and
//! written as PNG. For the CLI, MCP and tests; without a GPU, wgpu's software adapters do.

use std::sync::Arc;

use egui_wgpu::wgpu;

use datum_graph::Value;

use crate::camera::Camera;
use crate::render::{Frame, Renderer};
use crate::scene::{Geometry, Part, Scene};

/// Samples per pixel of pictures: the window's default.
pub const SAMPLES: u32 = 4;

/// A device without a window and a renderer, drawing `width × height`.
struct Offscreen {
    device: wgpu::Device,
    queue: wgpu::Queue,
    adapter: String,
    renderer: Renderer,
    width: u32,
    height: u32,
}

impl Offscreen {
    /// Any adapter wgpu finds, software ones included; the fastest first.
    fn new(width: u32, height: u32) -> Result<Self, String> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
        let options = wgpu::RequestAdapterOptions { power_preference: wgpu::PowerPreference::HighPerformance, ..Default::default() };
        let adapter = pollster::block_on(instance.request_adapter(&options)).map_err(|e| format!("no GPU adapter, not even a software one: {e}"))?;
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("datum_view_headless"),
            required_limits: wgpu::Limits::downlevel_defaults().using_resolution(adapter.limits()),
            ..Default::default()
        }))
        .map_err(|e| format!("the GPU device could not be opened: {e}"))?;
        let info = adapter.get_info();
        let renderer = Renderer::new(&device, SAMPLES);
        Ok(Self { device, queue, adapter: format!("{} ({:?})", info.name, info.backend), renderer, width, height })
    }

    /// Draws `frame` and returns the encoder, for a read-back to be appended.
    fn draw(&mut self, frame: &Frame) -> wgpu::CommandEncoder {
        self.renderer.draw(&self.device, &self.queue, frame, self.width, self.height)
    }

    fn wait(&self, submission: wgpu::SubmissionIndex) -> Result<(), String> {
        self.device
            .poll(wgpu::PollType::Wait { submission_index: Some(submission), timeout: None })
            .map(|_| ())
            .map_err(|e| format!("waiting for the GPU: {e}"))
    }

    /// The camera's frame of `geometry` (relative to the scene's anchor).
    fn frame(&self, scene: &Scene, geometry: &Arc<Geometry>, version: u64, camera: &Camera) -> Frame {
        let radius = scene.bounds.map_or(1.0, |(lo, hi)| (hi - lo).dot(hi - lo).sqrt() * 0.5);
        let (view_projection, eye) = camera.matrices(scene.anchor(), f64::from(self.width) / f64::from(self.height), radius);
        Frame { version, geometry: geometry.clone(), view_projection, eye }
    }
}

/// RGBA pixels, `width × height`, of `scene` seen by `camera`.
pub fn picture(scene: &Scene, camera: &Camera, width: u32, height: u32) -> Result<Vec<u8>, String> {
    let mut o = Offscreen::new(width, height)?;
    let frame = o.frame(scene, &Arc::new(Geometry::new(scene, scene.anchor())), 1, camera);
    let mut encoder = o.draw(&frame);
    let row = width * 4;
    let padded = row.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT) * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let read = o.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("datum_view_read"),
        size: u64::from(padded) * u64::from(height),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let picture = o.renderer.picture().ok_or("nothing was drawn")?;
    encoder.copy_texture_to_buffer(
        picture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &read,
            layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(padded), rows_per_image: Some(height) },
        },
        wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
    );
    let submission = o.queue.submit([encoder.finish()]);
    let slice = read.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    o.wait(submission)?;
    rx.recv().map_err(|e| e.to_string())?.map_err(|e| format!("reading the picture back: {e}"))?;
    let data = slice.get_mapped_range().map_err(|e| format!("reading the picture back: {e}"))?;
    Ok(data.chunks(padded as usize).take(height as usize).flat_map(|r| &r[..row as usize]).copied().collect())
}

/// GPU time per frame of `scene` (its geometry `full`, relative to its anchor), `width × height`: the median of `frames` frames on a full turn
/// around it, for the whole model in view and closer and closer up, with faces and edges together and apart,
/// with multisampling and without.
/// Returns the adapter and `(case, milliseconds)`.
pub fn frame_times(scene: &Scene, full: Geometry, width: u32, height: u32, frames: u32) -> Result<(String, Vec<(String, f64)>), String> {
    let mut o = Offscreen::new(width, height)?;
    let contents = [
        ("faces + edges", Arc::new(full.clone())),
        ("faces", Arc::new(Geometry { lines: Part::default(), ..full.clone() })),
        ("edges", Arc::new(Geometry { triangles: Part::default(), ..full })),
    ];
    let mut fitted = Camera::default();
    fitted.fit(scene.bounds, f64::from(width) / f64::from(height));
    let mut cases = Vec::new();
    for samples in [SAMPLES, 1] {
        o.renderer = Renderer::new(&o.device, samples);
        for (distance, closeness) in [("whole", 1.0), ("close", 0.15), ("closer", 0.05)] {
            for (version, (content, geometry)) in contents.iter().enumerate() {
                let mut times = Vec::with_capacity(frames as usize);
                for k in 0..=frames {
                    let mut camera = fitted;
                    camera.distance *= closeness;
                    camera.yaw += std::f64::consts::TAU * f64::from(k) / f64::from(frames.max(1));
                    let start = std::time::Instant::now();
                    let frame = o.frame(scene, geometry, version as u64 + 1, &camera);
                    let encoder = o.draw(&frame);
                    let submission = o.queue.submit([encoder.finish()]);
                    o.wait(submission)?;
                    // The first frame uploads the geometry: not a frame of the turn.
                    if k > 0 {
                        times.push(start.elapsed().as_secs_f64() * 1e3);
                    }
                }
                times.sort_by(f64::total_cmp);
                cases.push((format!("msaa {samples}, {distance}, {content}"), times.get(times.len() / 2).copied().unwrap_or(0.0)));
            }
        }
    }
    Ok((o.adapter, cases))
}

/// A PNG of `values` (bodies and sketches) in standard view `view`, fitted to fill the picture:
/// what `datum snapshot` writes and the MCP tool `render` returns.
pub fn snapshot(values: &[&Value], view: &str, width: u32, height: u32) -> Result<Vec<u8>, String> {
    let mut camera = Camera::view(view)?;
    if width == 0 || height == 0 {
        return Err("the picture needs a width and a height".into());
    }
    let scene = Scene::new(values, &mut Default::default());
    camera.fit(scene.bounds, f64::from(width) / f64::from(height));
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
