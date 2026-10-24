//! Drawing on any wgpu device: shaded faces and B-rep edges with a depth buffer, into a picture
//! of the renderer's own. The window shows that picture as an image, headless pictures read it
//! back ([`crate::capture`]): one renderer, as in SolidCraft's viewport (`crates/gpu`). Having its
//! own picture, the view chooses its multisampling frame by frame, apart from the window's.

use std::ops::Range;
use std::sync::Arc;

use egui_wgpu::wgpu;

use crate::scene::{Geometry, LINE_VERTEX, Part, TRIANGLE_VERTEX};

/// The GPU interfaces the window and headless pictures open, `WGPU_BACKEND` still choosing. On
/// Windows, DirectX 12 with OpenGL for GPUs without it, not Vulkan: a laptop's Vulkan driver
/// killed the window with an access violation no log could catch, and broke headless pictures
/// at the device's teardown (D103).
pub fn backends() -> wgpu::Backends {
    match wgpu::Backends::from_env() {
        Some(chosen) => chosen,
        None if cfg!(windows) => wgpu::Backends::DX12 | wgpu::Backends::GL,
        None => wgpu::Backends::all(),
    }
}

const SHADER: &str = r"
struct U { vp: mat4x4<f32>, eye: vec4<f32> };
@group(0) @binding(0) var<uniform> u: U;

struct TriOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) to_eye: vec3<f32>,
    @location(2) colour: vec4<f32>,
};

@vertex fn vs_tri(@location(0) pos: vec3<f32>, @location(1) normal: vec3<f32>, @location(2) colour: vec4<f32>) -> TriOut {
    var o: TriOut;
    o.clip = u.vp * vec4<f32>(pos, 1.0);
    o.normal = normal;
    o.to_eye = u.eye.xyz - pos;
    o.colour = colour;
    return o;
}

// A headlight: faces turned to the viewer are brightest.
@fragment fn fs_tri(i: TriOut) -> @location(0) vec4<f32> {
    let light = abs(dot(normalize(i.normal), normalize(i.to_eye)));
    return vec4<f32>(i.colour.rgb * (0.35 + 0.65 * light), 1.0);
}

struct LineOut { @builtin(position) clip: vec4<f32>, @location(0) colour: vec4<f32> };

@vertex fn vs_line(@location(0) pos: vec3<f32>, @location(1) colour: vec4<f32>) -> LineOut {
    var o: LineOut;
    o.clip = u.vp * vec4<f32>(pos, 1.0);
    o.colour = colour;
    return o;
}

@fragment fn fs_line(i: LineOut) -> @location(0) vec4<f32> {
    return i.colour;
}
";

/// The picture's colour format: written as is, shown as is by egui (which takes textures as
/// gamma-encoded), so the window and a PNG agree.
pub const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// The viewport's background.
pub const BACKGROUND: [f64; 3] = [0.93, 0.94, 0.96];

/// What one frame draws: the geometry (re-uploaded only when `version` changes) and the camera.
#[derive(Clone)]
pub struct Frame {
    pub version: u64,
    pub geometry: Arc<Geometry>,
    /// View-projection matrix, column-major, for vertices relative to the anchor.
    pub view_projection: [f32; 16],
    /// The eye relative to the anchor.
    pub eye: [f32; 3],
}

/// Samples per pixel of a smooth (multisampled) picture.
pub const SAMPLES: u32 = 4;

pub struct Renderer {
    /// Triangles and lines, without multisampling and with it.
    pipelines: [[wgpu::RenderPipeline; 2]; 2],
    uniform: wgpu::Buffer,
    bind: wgpu::BindGroup,
    /// The uploaded geometry: its version, triangles and lines.
    buffers: Option<(u64, Vertices, Vertices)>,
    /// Index ranges of triangles and lines in view in the frame being drawn.
    visible: [Vec<Range<u32>>; 2],
    target: Option<Target>,
}

/// The blocks of one kind of primitive.
type Vertices = Vec<Block>;

/// Vertex and index buffers of consecutive chunks, no larger than the device allows: a model of
/// ten million triangles needs several (wgpu's default limit is 256 MB a buffer).
struct Block {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    /// The block's indices in the part, and the first of its vertices (its indices count from 0
    /// in the part's numbering).
    first_index: u32,
    end_index: u32,
    first_vertex: u32,
}

/// The picture, `width × height`, and what it is drawn with: its depth; and multisampled colour,
/// resolved into the picture, and depth. One picture either way: the window shows the same
/// texture whether a frame is smooth or not.
struct Target {
    width: u32,
    height: u32,
    picture: wgpu::Texture,
    view: wgpu::TextureView,
    depth: wgpu::TextureView,
    multisampled: [wgpu::TextureView; 2],
}

impl Renderer {
    pub fn new(device: &wgpu::Device) -> Self {
        let module =
            device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("datum_view"), source: wgpu::ShaderSource::Wgsl(SHADER.into()) });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("datum_view_uniform"),
            size: 80,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("datum_view"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("datum_view"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: uniform.as_entire_binding() }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("datum_view"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = |samples: u32,
                        vs: &str,
                        fs: &str,
                        stride: u64,
                        attributes: &[wgpu::VertexAttribute],
                        topology: wgpu::PrimitiveTopology,
                        cull_mode: Option<wgpu::Face>,
                        bias: wgpu::DepthBiasState| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(vs),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &module,
                    entry_point: Some(vs),
                    compilation_options: Default::default(),
                    buffers: &[Some(wgpu::VertexBufferLayout { array_stride: stride, step_mode: wgpu::VertexStepMode::Vertex, attributes })],
                },
                primitive: wgpu::PrimitiveState { topology, cull_mode, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::LessEqual),
                    stencil: Default::default(),
                    bias,
                }),
                multisample: wgpu::MultisampleState { count: samples, ..Default::default() },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: FORMAT, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        // Faces are pushed back a little so that edges on them draw on top. Bodies are closed and
        // their triangles counterclockwise from outside: a triangle turned away is hidden anyway.
        let faces_back = wgpu::DepthBiasState { constant: 2, slope_scale: 1.5, clamp: 0.0 };
        let triangles = |samples| {
            pipeline(
                samples,
                "vs_tri",
                "fs_tri",
                TRIANGLE_VERTEX,
                &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Unorm8x4],
                wgpu::PrimitiveTopology::TriangleList,
                Some(wgpu::Face::Back),
                faces_back,
            )
        };
        let lines = |samples| {
            pipeline(
                samples,
                "vs_line",
                "fs_line",
                LINE_VERTEX,
                &wgpu::vertex_attr_array![0 => Float32x3, 1 => Unorm8x4],
                wgpu::PrimitiveTopology::LineList,
                None,
                wgpu::DepthBiasState::default(),
            )
        };
        let pipelines = [1, SAMPLES].map(|n| [triangles(n), lines(n)]);
        Self { pipelines, uniform, bind, buffers: None, visible: Default::default(), target: None }
    }

    /// Draws `frame` into the picture, `width × height` (made anew when the size changes),
    /// multisampled if `smooth`, and returns the commands to submit, for a read-back to be appended.
    pub fn draw(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frame: &Frame, width: u32, height: u32, smooth: bool) -> wgpu::CommandEncoder {
        self.prepare(device, queue, frame);
        if self.target.as_ref().is_none_or(|t| (t.width, t.height) != (width, height)) {
            self.target = Some(Target::new(device, width, height));
        }
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("datum_view") });
        let Some(t) = &self.target else { return encoder };
        let (view, resolve_target, depth) = if smooth { (&t.multisampled[0], Some(&t.view), &t.multisampled[1]) } else { (&t.view, None, &t.depth) };
        let [r, g, b] = BACKGROUND;
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("datum_view"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
                depth_slice: None,
                resolve_target,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color { r, g, b, a: 1.0 }),
                    store: if resolve_target.is_some() { wgpu::StoreOp::Discard } else { wgpu::StoreOp::Store },
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Discard }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        self.paint(&mut pass, &self.pipelines[usize::from(smooth)]);
        drop(pass);
        encoder
    }

    /// The picture last drawn, if any.
    pub fn picture(&self) -> Option<&wgpu::Texture> {
        self.target.as_ref().map(|t| &t.picture)
    }

    /// Uploads the frame's geometry when it is new, and its camera.
    fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frame: &Frame) {
        if self.buffers.as_ref().is_none_or(|b| b.0 != frame.version) {
            let buffer = |bytes: &[u8], usage| {
                let b = device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("datum_view_geometry"),
                    size: bytes.len() as u64,
                    usage: usage | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                queue.write_buffer(&b, 0, bytes);
                b
            };
            let limit = device.limits().max_buffer_size;
            let part = |p: &Part, stride: u64| {
                split(p, stride, limit)
                    .into_iter()
                    .map(|(idx, vtx)| {
                        let indices: Vec<u8> = p.indices[idx.start as usize..idx.end as usize].iter().flat_map(|i| i.to_le_bytes()).collect();
                        let vertices = &p.vertices[(u64::from(vtx.start) * stride) as usize..(u64::from(vtx.end) * stride) as usize];
                        Block {
                            vertices: buffer(vertices, wgpu::BufferUsages::VERTEX),
                            indices: buffer(&indices, wgpu::BufferUsages::INDEX),
                            first_index: idx.start,
                            end_index: idx.end,
                            first_vertex: vtx.start,
                        }
                    })
                    .collect::<Vec<_>>()
            };
            self.buffers = Some((frame.version, part(&frame.geometry.triangles, TRIANGLE_VERTEX), part(&frame.geometry.lines, LINE_VERTEX)));
        }
        self.visible = [&frame.geometry.triangles, &frame.geometry.lines].map(|p| visible(p, &frame.view_projection));
        let mut u = Vec::with_capacity(80);
        for f in frame.view_projection.iter().chain(&frame.eye).chain(&[0.0]) {
            u.extend_from_slice(&f.to_le_bytes());
        }
        queue.write_buffer(&self.uniform, 0, &u);
    }

    fn paint(&self, pass: &mut wgpu::RenderPass<'_>, pipelines: &[wgpu::RenderPipeline; 2]) {
        let Some((_, tris, lines)) = &self.buffers else { return };
        pass.set_bind_group(0, &self.bind, &[]);
        for ((pipeline, blocks), ranges) in pipelines.iter().zip([tris, lines]).zip(&self.visible) {
            pass.set_pipeline(pipeline);
            for b in blocks {
                pass.set_vertex_buffer(0, b.vertices.slice(..));
                pass.set_index_buffer(b.indices.slice(..), wgpu::IndexFormat::Uint32);
                for r in ranges {
                    let (start, end) = (r.start.max(b.first_index), r.end.min(b.end_index));
                    if start < end {
                        pass.draw_indexed(start - b.first_index..end - b.first_index, -(b.first_vertex as i32), 0..1);
                    }
                }
            }
        }
    }
}

/// Runs of whole chunks of `part`, each within `limit` bytes of vertices (`stride` each) and of
/// indices: their index and vertex ranges.
fn split(part: &Part, stride: u64, limit: u64) -> Vec<(Range<u32>, Range<u32>)> {
    let mut runs = Vec::new();
    let (mut first, mut last) = ((0u32, 0u32), (0u32, 0u32));
    for c in &part.chunks {
        if u64::from(c.vertex_end - first.1) * stride > limit || u64::from(c.end - first.0) * 4 > limit {
            if last.0 > first.0 {
                runs.push((first.0..last.0, first.1..last.1));
            }
            first = last;
        }
        last = (c.end, c.vertex_end);
    }
    if last.0 > first.0 {
        runs.push((first.0..last.0, first.1..last.1));
    }
    runs
}

/// Index ranges of the chunks of `part` that may be in view of `view_projection` (column-major),
/// adjacent ones joined. A chunk is left out when its box lies wholly outside one of the planes
/// of the view's frustum.
fn visible(part: &Part, view_projection: &[f32; 16]) -> Vec<Range<u32>> {
    let row = |i: usize| [0, 1, 2, 3].map(|c| view_projection[c * 4 + i]);
    let (x, y, z, w) = (row(0), row(1), row(2), row(3));
    let add = |a: [f32; 4], b: [f32; 4], s: f32| [0, 1, 2, 3].map(|k| a[k] + s * b[k]);
    // Clip space: -w ≤ x, y ≤ w and 0 ≤ z ≤ w.
    let planes = [add(w, x, 1.0), add(w, x, -1.0), add(w, y, 1.0), add(w, y, -1.0), z, add(w, z, -1.0)];
    let mut ranges: Vec<Range<u32>> = Vec::new();
    let mut start = 0;
    for c in &part.chunks {
        // The box's corner farthest along each plane's normal.
        let inside = planes.iter().all(|p| (0..3).map(|k| p[k] * if p[k] >= 0.0 { c.hi[k] } else { c.lo[k] }).sum::<f32>() + p[3] >= 0.0);
        if inside {
            match ranges.last_mut() {
                Some(r) if r.end == start => r.end = c.end,
                _ => ranges.push(start..c.end),
            }
        }
        start = c.end;
    }
    ranges
}

impl Target {
    fn new(device: &wgpu::Device, width: u32, height: u32) -> Self {
        let texture = |format, samples, usage| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some("datum_view_picture"),
                size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: samples,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        let picture =
            texture(FORMAT, 1, wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_SRC);
        let attachment = |format, samples| texture(format, samples, wgpu::TextureUsages::RENDER_ATTACHMENT).create_view(&Default::default());
        let (depth, multisampled) = (attachment(DEPTH, 1), [attachment(FORMAT, SAMPLES), attachment(DEPTH, SAMPLES)]);
        Self { width, height, view: picture.create_view(&Default::default()), picture, depth, multisampled }
    }
}

#[cfg(test)]
#[allow(clippy::single_range_in_vec_init, clippy::unwrap_used)]
mod tests {
    use datum_math::Vec3;

    use super::*;
    use crate::camera::Camera;
    use crate::scene::Chunk;

    /// Three unit cubes of primitives along X, ten apart; the anchor is the origin.
    fn part() -> Part {
        let chunk = |x: f32, end| Chunk { lo: [x, 0.0, 0.0], hi: [x + 1.0, 1.0, 1.0], end, vertex_end: 0 };
        Part { vertices: Vec::new(), indices: Vec::new(), chunks: vec![chunk(0.0, 30), chunk(10.0, 60), chunk(20.0, 90)] }
    }

    fn ranges(camera: &Camera) -> Vec<Range<u32>> {
        let (view_projection, _) = camera.matrices(Vec3::ZERO, 1.0, 30.0);
        visible(&part(), &view_projection)
    }

    /// Blocks of whole chunks, each within the limit: 30 indices and 10 vertices a chunk.
    #[test]
    fn large_parts_are_split() {
        let chunk = |k: u32| Chunk { lo: [0.0; 3], hi: [1.0; 3], end: 30 * k, vertex_end: 10 * k };
        let p = Part { vertices: Vec::new(), indices: Vec::new(), chunks: (1..=5).map(chunk).collect() };
        assert_eq!(split(&p, 4, 1 << 20), [(0..150, 0..50)]);
        // 30 indices are 120 bytes: two chunks a block, the last alone.
        assert_eq!(split(&p, 4, 240), [(0..60, 0..20), (60..120, 20..40), (120..150, 40..50)]);
    }

    #[test]
    fn chunks_out_of_view_are_left_out() {
        let mut camera = Camera::default();
        camera.fit(Some((Vec3::ZERO, Vec3::new(21.0, 1.0, 1.0))), 1.0);
        assert_eq!(ranges(&camera), [0..90], "all in view, one draw");
        // Close to the middle cube, looking straight down: the others are beside the view.
        let middle = Camera { target: Vec3::new(10.5, 0.5, 0.5), distance: 3.0, ..Camera::view("top").unwrap() };
        assert_eq!(ranges(&middle), [30..60]);
        // From x = 13.5 along −X: the first two ahead, the last behind the eye.
        let along = Camera { target: Vec3::new(10.5, 0.5, 0.5), distance: 3.0, yaw: 0.0, pitch: 0.0, ..Camera::default() };
        assert_eq!(ranges(&along), [0..60]);
    }
}
