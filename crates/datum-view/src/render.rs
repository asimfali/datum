//! Drawing on any wgpu device: shaded faces and B-rep edges with a depth buffer. The window
//! draws through it in an `egui_wgpu` paint callback, headless pictures in an offscreen pass
//! ([`crate::capture`]): one renderer, as in SolidCraft's viewport (`crates/gpu`).

use std::sync::Arc;

use egui_wgpu::wgpu;

use crate::scene::{Geometry, LINE_VERTEX, TRIANGLE_VERTEX};

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

pub struct Renderer {
    triangles: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    uniform: wgpu::Buffer,
    bind: wgpu::BindGroup,
    /// The uploaded geometry: its version, triangles and lines.
    buffers: Option<(u64, Vertices, Vertices)>,
}

/// A vertex buffer and its vertex count; `None` when empty.
type Vertices = Option<(wgpu::Buffer, u32)>;

impl Renderer {
    /// Pipelines for colour target `format`, depth `depth` and `samples` per pixel.
    pub fn new(device: &wgpu::Device, format: wgpu::TextureFormat, depth: wgpu::TextureFormat, samples: u32) -> Self {
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
        let pipeline =
            |vs: &str, fs: &str, stride: u64, attributes: &[wgpu::VertexAttribute], topology: wgpu::PrimitiveTopology, bias: wgpu::DepthBiasState| {
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(vs),
                    layout: Some(&pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &module,
                        entry_point: Some(vs),
                        compilation_options: Default::default(),
                        buffers: &[Some(wgpu::VertexBufferLayout { array_stride: stride, step_mode: wgpu::VertexStepMode::Vertex, attributes })],
                    },
                    primitive: wgpu::PrimitiveState { topology, ..Default::default() },
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: depth,
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
                        targets: &[Some(wgpu::ColorTargetState { format, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            };
        // Faces are pushed back a little so that edges on them draw on top.
        let faces_back = wgpu::DepthBiasState { constant: 2, slope_scale: 1.5, clamp: 0.0 };
        let triangles = pipeline(
            "vs_tri",
            "fs_tri",
            TRIANGLE_VERTEX,
            &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Unorm8x4],
            wgpu::PrimitiveTopology::TriangleList,
            faces_back,
        );
        let lines = pipeline(
            "vs_line",
            "fs_line",
            LINE_VERTEX,
            &wgpu::vertex_attr_array![0 => Float32x3, 1 => Unorm8x4],
            wgpu::PrimitiveTopology::LineList,
            wgpu::DepthBiasState::default(),
        );
        Self { triangles, lines, uniform, bind, buffers: None }
    }

    /// Uploads the frame's geometry when it is new, and its camera.
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frame: &Frame) {
        if self.buffers.as_ref().is_none_or(|b| b.0 != frame.version) {
            let buffer = |bytes: &[u8], stride: u64| {
                (!bytes.is_empty()).then(|| {
                    let b = device.create_buffer(&wgpu::BufferDescriptor {
                        label: Some("datum_view_vertices"),
                        size: bytes.len() as u64,
                        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                        mapped_at_creation: false,
                    });
                    queue.write_buffer(&b, 0, bytes);
                    (b, (bytes.len() as u64 / stride) as u32)
                })
            };
            self.buffers = Some((frame.version, buffer(&frame.geometry.triangles, TRIANGLE_VERTEX), buffer(&frame.geometry.lines, LINE_VERTEX)));
        }
        let mut u = Vec::with_capacity(80);
        for f in frame.view_projection.iter().chain(&frame.eye).chain(&[0.0]) {
            u.extend_from_slice(&f.to_le_bytes());
        }
        queue.write_buffer(&self.uniform, 0, &u);
    }

    pub fn paint(&self, pass: &mut wgpu::RenderPass<'_>) {
        let Some((_, tris, lines)) = &self.buffers else { return };
        pass.set_bind_group(0, &self.bind, &[]);
        if let Some((b, n)) = tris {
            pass.set_pipeline(&self.triangles);
            pass.set_vertex_buffer(0, b.slice(..));
            pass.draw(0..*n, 0..1);
        }
        if let Some((b, n)) = lines {
            pass.set_pipeline(&self.lines);
            pass.set_vertex_buffer(0, b.slice(..));
            pass.draw(0..*n, 0..1);
        }
    }
}
