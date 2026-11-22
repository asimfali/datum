//! Persistent placement of separate native windows. Monitor bounds and positions are physical pixels;
//! the content size is in egui points so that it survives a change of display scale.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

const STORAGE: &str = "datum_native_windows";
const LEGACY: &str = "datum_rules_window";

#[derive(Default)]
pub(super) struct NativeWindows {
    windows: BTreeMap<String, WindowGeometry>,
    opening: BTreeMap<&'static str, Opening>,
}

#[derive(Default)]
struct Opening {
    frames: u8,
    seen: bool,
    placement: Option<Placement>,
}

impl NativeWindows {
    pub(super) fn load(storage: Option<&dyn eframe::Storage>) -> Self {
        let windows = storage.and_then(|s| s.get_string(STORAGE)).and_then(|s| serde_json::from_str(&s).ok());
        let windows = windows.unwrap_or_else(|| {
            let mut windows = BTreeMap::new();
            if let Some(old) = storage.and_then(|s| s.get_string(LEGACY)).and_then(|s| serde_json::from_str(&s).ok()) {
                windows.insert("table.rules".into(), old);
            }
            windows
        });
        Self { windows, opening: BTreeMap::new() }
    }

    pub(super) fn save(&self, storage: &mut dyn eframe::Storage) {
        if let Ok(json) = serde_json::to_string(&self.windows) {
            storage.set_string(STORAGE, json);
            storage.remove_string(LEGACY);
        }
    }

    pub(super) fn show(
        &mut self,
        ctx: &egui::Context,
        frame: Option<&eframe::Frame>,
        key: &'static str,
        title: &str,
        preferred_size: egui::Vec2,
        mut contents: impl FnMut(&mut egui::Ui),
    ) -> bool {
        let opening = self.opening.entry(key).or_default();
        opening.seen = true;
        // Freeze the placement for this opening. Later frames observe user moves and resize
        // events without issuing another move or continually enumerating displays.
        let windows = &mut self.windows;
        let placement = *opening.placement.get_or_insert_with(|| {
            let monitors = monitors(ctx, frame);
            let monitor_size = monitors
                .first()
                .map(|m| m.bounds.size() / m.pixels_per_point)
                .or_else(|| ctx.input(|i| i.viewport().monitor_size))
                .unwrap_or(egui::vec2(1280.0, 800.0));
            let limit = egui::vec2(monitor_size.x * 0.9, monitor_size.y * 0.8);
            let preferred = if valid_size(preferred_size) { preferred_size } else { egui::vec2(800.0, 400.0) };
            let initial_size = preferred.min(limit);
            let entry = windows
                .entry(key.into())
                .or_insert_with(|| WindowGeometry { maximized: preferred.x > limit.x || preferred.y > limit.y, ..Default::default() });
            entry.placement(&monitors, initial_size)
        });
        let mut builder = egui::ViewportBuilder::default().with_title(title.to_owned());
        if opening.frames == 0 {
            builder = builder.with_inner_size(placement.size);
        }
        let mut closed = false;
        ctx.show_viewport_immediate(egui::ViewportId::from_hash_of(key), builder, |ui, class| {
            closed = ui.input(|i| i.viewport().close_requested());
            if class == egui::ViewportClass::Immediate {
                if opening.frames < 3 && !closed {
                    match opening.frames {
                        1 => {
                            if let Some(pos) = placement.position {
                                ui.send_viewport_cmd(egui::ViewportCommand::OuterPosition(pos / ui.ctx().pixels_per_point()));
                            }
                        }
                        2 => {
                            ui.send_viewport_cmd(egui::ViewportCommand::InnerSize(placement.size));
                            ui.send_viewport_cmd(egui::ViewportCommand::Maximized(placement.maximized));
                        }
                        _ => {}
                    }
                    opening.frames += 1;
                    ui.ctx().request_repaint();
                } else if opening.frames == 3
                    && let Some(entry) = windows.get_mut(key)
                {
                    let pixels_per_point = ui.ctx().pixels_per_point();
                    ui.input(|input| entry.remember(input.viewport(), pixels_per_point));
                }
            }
            contents(ui);
        });
        if closed {
            self.opening.remove(key);
        }
        closed
    }

    pub(super) fn finish_frame(&mut self) {
        self.opening.retain(|_, opening| std::mem::take(&mut opening.seen));
    }
}

/// Available displays in physical pixels, with the parent window's monitor first.
fn monitors(ctx: &egui::Context, frame: Option<&eframe::Frame>) -> Vec<Monitor> {
    let Some(window) = frame.and_then(eframe::Frame::winit_window) else { return Vec::new() };
    let current = window.current_monitor();
    let zoom = ctx.zoom_factor();
    let mut monitors = Vec::new();
    for monitor in window.available_monitors() {
        let pos = monitor.position();
        let size = monitor.size();
        let bounds = egui::Rect::from_min_size(egui::pos2(pos.x as f32, pos.y as f32), egui::vec2(size.width as f32, size.height as f32));
        let pixels_per_point = monitor.scale_factor() as f32 * zoom;
        let data = Monitor { bounds, pixels_per_point };
        if current.as_ref() == Some(&monitor) {
            monitors.insert(0, data);
        } else {
            monitors.push(data);
        }
    }
    monitors
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
#[serde(default)]
struct WindowGeometry {
    size: Option<[f32; 2]>,
    position: Option<[f32; 2]>,
    monitor: Option<[f32; 2]>,
    maximized: bool,
}

#[derive(Clone, Copy, Debug)]
struct Monitor {
    bounds: egui::Rect,
    pixels_per_point: f32,
}

#[derive(Clone, Copy, Debug)]
struct Placement {
    position: Option<egui::Pos2>,
    size: egui::Vec2,
    maximized: bool,
}

impl WindowGeometry {
    fn remember(&mut self, viewport: &egui::ViewportInfo, pixels_per_point: f32) {
        if viewport.minimized == Some(true) || !pixels_per_point.is_finite() || pixels_per_point <= 0.0 {
            return;
        }
        if let Some(maximized) = viewport.maximized {
            self.maximized = maximized;
        }
        if let Some(rect) = viewport.outer_rect.filter(|r| valid_rect(*r)) {
            let center = rect.center() * pixels_per_point;
            if finite_pos(center) {
                self.monitor = Some([center.x, center.y]);
            }
            if viewport.maximized == Some(false) && viewport.fullscreen != Some(true) {
                let pos = rect.min * pixels_per_point;
                if finite_pos(pos) {
                    self.position = Some([pos.x, pos.y]);
                }
            }
        }
        if viewport.maximized == Some(false)
            && viewport.fullscreen != Some(true)
            && let Some(rect) = viewport.inner_rect.filter(|r| valid_rect(*r))
        {
            self.size = Some([rect.width(), rect.height()]);
        }
    }

    fn placement(&self, monitors: &[Monitor], default_size: egui::Vec2) -> Placement {
        let fallback = if valid_size(default_size) { default_size } else { egui::vec2(800.0, 400.0) };
        let size = self.size.map(|s| egui::vec2(s[0], s[1])).filter(|s| valid_size(*s)).unwrap_or(fallback);
        let point = self.monitor.map(|p| egui::pos2(p[0], p[1])).filter(|p| finite_pos(*p));
        let valid = |m: &&Monitor| valid_rect(m.bounds) && m.pixels_per_point.is_finite() && m.pixels_per_point > 0.0;
        let mut available = monitors.iter().filter(valid);
        let first = available.next();
        let selected = point.and_then(|p| monitors.iter().filter(valid).find(|m| m.bounds.contains(p))).or(first);
        let Some(monitor) = selected else {
            return Placement { position: None, size, maximized: self.maximized };
        };
        let limit = monitor.bounds.size() / monitor.pixels_per_point;
        let size = size.min(limit);
        let saved = self.position.map(|p| egui::pos2(p[0], p[1])).filter(|p| finite_pos(*p));
        // If the saved display disappeared, clamp to the current display rather than restoring an
        // unreachable origin. A maximized window still starts on the monitor it was last seen on.
        let position = saved.or(point).map(|p| {
            let max = monitor.bounds.max - size * monitor.pixels_per_point;
            egui::pos2(
                p.x.clamp(monitor.bounds.min.x, max.x.max(monitor.bounds.min.x)),
                p.y.clamp(monitor.bounds.min.y, max.y.max(monitor.bounds.min.y)),
            )
        });
        Placement { position, size, maximized: self.maximized }
    }
}

fn finite_pos(p: egui::Pos2) -> bool {
    p.x.is_finite() && p.y.is_finite()
}

fn valid_size(s: egui::Vec2) -> bool {
    s.x.is_finite() && s.y.is_finite() && s.x > 0.0 && s.y > 0.0
}

fn valid_rect(r: egui::Rect) -> bool {
    finite_pos(r.min) && finite_pos(r.max) && valid_size(r.size())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[derive(Default)]
    struct Storage(BTreeMap<String, String>);

    impl eframe::Storage for Storage {
        fn get_string(&self, key: &str) -> Option<String> {
            self.0.get(key).cloned()
        }
        fn set_string(&mut self, key: &str, value: String) {
            self.0.insert(key.into(), value);
        }
        fn remove_string(&mut self, key: &str) {
            self.0.remove(key);
        }
        fn flush(&mut self) {}
    }

    #[test]
    fn migrates_and_keeps_windows_independent() {
        let mut storage = Storage::default();
        let mut old = WindowGeometry::default();
        old.remember(&viewport([20.0, 30.0], [500.0, 300.0], false), 1.0);
        eframe::Storage::set_string(&mut storage, LEGACY, serde_json::to_string(&old).unwrap());
        let mut all = NativeWindows::load(Some(&storage));
        all.windows.insert("table.materials".into(), WindowGeometry { maximized: true, ..Default::default() });
        all.save(&mut storage);
        assert!(!storage.0.contains_key(LEGACY));
        let loaded = NativeWindows::load(Some(&storage));
        assert!(!loaded.windows["table.rules"].maximized);
        assert!(loaded.windows["table.materials"].maximized);
        assert_eq!(loaded.windows["table.rules"].position, Some([20.0, 30.0]));
    }

    #[test]
    fn absent_window_starts_opening_again() {
        let mut all = NativeWindows::default();
        all.opening.insert("a", Opening { frames: 3, seen: true, ..Default::default() });
        all.opening.insert("b", Opening { frames: 2, seen: true, ..Default::default() });
        all.finish_frame();
        all.opening.get_mut("b").unwrap().seen = true;
        all.finish_frame();
        assert!(!all.opening.contains_key("a"));
        assert_eq!(all.opening["b"].frames, 2);
        assert_eq!(all.opening.entry("a").or_default().frames, 0);
    }

    fn monitor(min: [f32; 2], size: [f32; 2], scale: f32) -> Monitor {
        Monitor { bounds: egui::Rect::from_min_size(egui::pos2(min[0], min[1]), egui::vec2(size[0], size[1])), pixels_per_point: scale }
    }

    fn viewport(pos: [f32; 2], size: [f32; 2], maximized: bool) -> egui::ViewportInfo {
        let rect = egui::Rect::from_min_size(egui::pos2(pos[0], pos[1]), egui::vec2(size[0], size[1]));
        egui::ViewportInfo { inner_rect: Some(rect), outer_rect: Some(rect), maximized: Some(maximized), ..Default::default() }
    }

    #[test]
    fn normal_and_maximized_placement_survives_serialization() {
        let screens = [monitor([0.0, 0.0], [1920.0, 1080.0], 1.0), monitor([-2560.0, 0.0], [2560.0, 1440.0], 2.0)];
        let mut state = WindowGeometry::default();
        state.remember(&viewport([-1100.0, 50.0], [600.0, 400.0], false), 2.0);
        state.remember(&viewport([-1280.0, 0.0], [1280.0, 720.0], true), 2.0);
        let json = serde_json::to_string(&state).unwrap();
        let state: WindowGeometry = serde_json::from_str(&json).unwrap();
        let placed = state.placement(&screens, egui::vec2(900.0, 500.0));
        assert!(placed.maximized);
        assert_eq!(placed.size, egui::vec2(600.0, 400.0));
        assert_eq!(placed.position, Some(egui::pos2(-2200.0, 100.0)));
        let fallback = state.placement(&screens[..1], egui::vec2(900.0, 500.0));
        assert_eq!(fallback.position, Some(egui::pos2(0.0, 100.0)));
    }

    #[test]
    fn minimized_and_invalid_values_do_not_replace_normal_geometry() {
        let mut state = WindowGeometry::default();
        state.remember(&viewport([10.0, 20.0], [300.0, 200.0], false), 1.0);
        let mut minimized = viewport([0.0, 0.0], [100.0, 100.0], true);
        minimized.minimized = Some(true);
        state.remember(&minimized, 1.0);
        state.remember(&viewport([0.0, 0.0], [100.0, 100.0], true), f32::NAN);
        let placed = state.placement(&[monitor([0.0, 0.0], [1000.0, 800.0], 1.0)], egui::vec2(800.0, 400.0));
        assert!(!placed.maximized);
        assert_eq!(placed.position, Some(egui::pos2(10.0, 20.0)));
        assert_eq!(placed.size, egui::vec2(300.0, 200.0));
    }
}
