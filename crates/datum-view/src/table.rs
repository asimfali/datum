//! The table editor (D178): a window of its own showing a `datum_table::Table` as a grid,
//! edited as in a spreadsheet, and the first table it is bound to: the model's parameters.
//! The editor only reports what was edited and which rows to delete; what that does to the
//! model is the source's mapping to requests below.

use std::collections::{BTreeMap, BTreeSet};

use datum_table::{Table, layout, parse_tsv};
use egui::{Align2, Event, Key, Modifiers, Pos2, Rect, Sense, Stroke, StrokeKind, TextStyle, Vec2};
use serde_json::{Value as Json, json};

use crate::app::{Request, definition, marks};
use crate::dims::value_of;
use crate::i18n::I18n;
use crate::measure::number;

/// Pixels per millimetre of the table.
const SCALE: f32 = 3.5;

/// A cell edited: row, column, the new text.
pub type Edit = (usize, usize, String);

/// What the editor asks of its source in a frame.
#[derive(Default)]
pub struct Output {
    pub edits: Vec<Edit>,
    /// Rows to delete, ascending.
    pub delete: Vec<usize>,
}

/// A cell being edited, and whether its field still has to take the keyboard.
struct Editing {
    text: String,
    fresh: bool,
}

/// Where an edit ended: the cell the cursor goes to.
#[derive(Clone, Copy)]
enum End {
    Down,
    Right,
    Left,
    Cancel,
}

/// The editor's state: the cursor, the other corner of the selection, the cell being edited and
/// the last error. Row 0 is a header.
#[derive(Default)]
pub struct Editor {
    cursor: (usize, usize),
    anchor: (usize, usize),
    editing: Option<Editing>,
    pub error: Option<String>,
}

impl Editor {
    /// The grid of `table`. `editable(row, column)` says which cells can be changed, also for
    /// rows beyond the table: a paste may reach past its last row.
    pub fn show(&mut self, ui: &mut egui::Ui, table: &Table, editable: &dyn Fn(usize, usize) -> bool, i18n: &I18n) -> Output {
        let lay = layout(table);
        let (rows, columns) = (table.rows().len(), table.columns().len());
        let clamp = |(r, c): (usize, usize)| (r.min(rows.saturating_sub(1)), c.min(columns.saturating_sub(1)));
        (self.cursor, self.anchor) = (clamp(self.cursor), clamp(self.anchor));
        let mut out = Output::default();
        let moved = self.editing.is_none() && ui.input(|i| i.viewport().focused != Some(false)) && self.keys(ui, table, editable, &mut out);
        let error = ui.visuals().error_fg_color;
        let line = ui.spacing().interact_size.y + ui.spacing().item_spacing.y;
        egui::ScrollArea::both().auto_shrink([false, false]).max_height(ui.available_height() - line).show(ui, |ui| {
            let size = lay.size();
            let (rect, resp) = ui.allocate_exact_size(Vec2::new(size[0] as f32, size[1] as f32) * SCALE, Sense::click_and_drag());
            let on_screen = |r: datum_table::Rect| {
                Rect::from_min_size(rect.min + Vec2::new(r.x as f32, r.y as f32) * SCALE, Vec2::new(r.w as f32, r.h as f32) * SCALE)
            };
            let cell_rect = |at: (usize, usize)| lay.cell(at.0, at.1).map(on_screen).unwrap_or(Rect::NOTHING);
            let at = |p: Pos2| lay.at([f64::from((p.x - rect.min.x) / SCALE), f64::from((p.y - rect.min.y) / SCALE)]);
            self.paint(ui, table, editable, &lay, &on_screen, rect);
            if moved {
                ui.scroll_to_rect(cell_rect(self.cursor), None);
            }
            if self.editing.is_some() {
                self.field(ui, cell_rect(self.cursor), rows, columns, &mut out);
            }
            let (pressed, secondary, shift, pos) =
                ui.input(|i| (i.pointer.primary_pressed(), i.pointer.secondary_pressed(), i.modifiers.shift, i.pointer.interact_pos()));
            if resp.contains_pointer()
                && let Some(c) = pos.and_then(at)
            {
                if pressed {
                    self.cursor = c;
                    if !shift {
                        self.anchor = c;
                    }
                } else if secondary && !self.selected(c) {
                    (self.cursor, self.anchor) = (c, c);
                }
            }
            if resp.dragged_by(egui::PointerButton::Primary)
                && let Some(c) = pos.and_then(at)
            {
                self.cursor = c;
            }
            if resp.double_clicked() && editable(self.cursor.0, self.cursor.1) {
                self.edit(table.cell(self.cursor.0, self.cursor.1).to_string());
            }
            resp.context_menu(|ui| {
                if ui.button(i18n.t("table-delete-rows", "Delete rows")).clicked() {
                    out.delete = self.selected_rows();
                    ui.close();
                }
            });
        });
        if let Some(e) = &self.error {
            ui.colored_label(error, e);
        }
        out
    }

    fn selected(&self, (r, c): (usize, usize)) -> bool {
        let (r0, r1) = (self.cursor.0.min(self.anchor.0), self.cursor.0.max(self.anchor.0));
        let (c0, c1) = (self.cursor.1.min(self.anchor.1), self.cursor.1.max(self.anchor.1));
        (r0..=r1).contains(&r) && (c0..=c1).contains(&c)
    }

    fn selected_rows(&self) -> Vec<usize> {
        (self.cursor.0.min(self.anchor.0)..=self.cursor.0.max(self.anchor.0)).collect()
    }

    /// Editing the cursor cell, starting from `text`.
    fn edit(&mut self, text: String) {
        self.editing = Some(Editing { text, fresh: true });
    }

    /// Moves the cursor by `(dr, dc)`; `extend` keeps the selection's other corner.
    fn step(&mut self, (dr, dc): (isize, isize), (rows, columns): (usize, usize), extend: bool) {
        let to = |v: usize, d: isize, n: usize| v.saturating_add_signed(d).min(n.saturating_sub(1));
        self.cursor = (to(self.cursor.0, dr, rows), to(self.cursor.1, dc, columns));
        if !extend {
            self.anchor = self.cursor;
        }
    }

    /// The cursor one cell on in reading order (`forward`) or back, wrapping to the next row.
    fn next(&mut self, (rows, columns): (usize, usize), forward: bool) {
        let (r, c) = self.cursor;
        let at = if forward {
            if c + 1 < columns { (r, c + 1) } else { (r + 1, 0) }
        } else if c > 0 {
            (r, c - 1)
        } else {
            (r.saturating_sub(1), columns - 1)
        };
        self.cursor = (at.0.min(rows - 1), at.1);
        self.anchor = self.cursor;
    }

    /// The keys while no cell is edited. Whether the cursor moved.
    fn keys(&mut self, ui: &egui::Ui, table: &Table, editable: &dyn Fn(usize, usize) -> bool, out: &mut Output) -> bool {
        let before = self.cursor;
        let size = (table.rows().len(), table.columns().len());
        for (key, d) in [(Key::ArrowUp, (-1, 0)), (Key::ArrowDown, (1, 0)), (Key::ArrowLeft, (0, -1)), (Key::ArrowRight, (0, 1))] {
            if ui.input_mut(|i| i.consume_key(Modifiers::SHIFT, key)) {
                self.step(d, size, true);
            } else if ui.input_mut(|i| i.consume_key(Modifiers::NONE, key)) {
                self.step(d, size, false);
            }
        }
        if ui.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::Tab)) {
            self.next(size, false);
        } else if ui.input_mut(|i| i.consume_key(Modifiers::NONE, Key::Tab)) {
            self.next(size, true);
        }
        let (r, c) = self.cursor;
        if (ui.input_mut(|i| i.consume_key(Modifiers::NONE, Key::Enter)) | ui.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F2)))
            && editable(r, c)
        {
            self.edit(table.cell(r, c).to_string());
        }
        if ui.input_mut(|i| i.consume_key(Modifiers::NONE, Key::Delete)) {
            out.edits.extend(self.cells().filter(|&(r, c)| editable(r, c)).map(|(r, c)| (r, c, String::new())));
        }
        if ui.input_mut(|i| i.consume_key(Modifiers::COMMAND, Key::Minus)) {
            out.delete = self.selected_rows();
        }
        let events = ui.input(|i| i.events.clone());
        for e in events {
            match e {
                Event::Copy => ui.ctx().copy_text(table.copy(self.anchor, self.cursor)),
                // Cut, as in Excel: copied, then cleared where it can be.
                Event::Cut => {
                    ui.ctx().copy_text(table.copy(self.anchor, self.cursor));
                    out.edits.extend(self.cells().filter(|&(r, c)| editable(r, c)).map(|(r, c)| (r, c, String::new())));
                }
                Event::Paste(text) => {
                    for (dr, row) in parse_tsv(&text).into_iter().enumerate() {
                        for (dc, cell) in row.into_iter().enumerate() {
                            let (r, c) = (self.cursor.0 + dr, self.cursor.1 + dc);
                            if c < table.columns().len() && editable(r, c) {
                                out.edits.push((r, c, cell));
                            }
                        }
                    }
                }
                // A character typed replaces the cell's text; the field must not get it again.
                Event::Text(s) if editable(r, c) && !s.chars().all(char::is_control) => {
                    ui.input_mut(|i| i.events.retain(|e| !matches!(e, Event::Text(_))));
                    self.edit(s);
                }
                _ => {}
            }
        }
        self.cursor != before
    }

    /// The cells of the selection, row by row.
    fn cells(&self) -> impl Iterator<Item = (usize, usize)> {
        let (r0, r1) = (self.cursor.0.min(self.anchor.0), self.cursor.0.max(self.anchor.0));
        let (c0, c1) = (self.cursor.1.min(self.anchor.1), self.cursor.1.max(self.anchor.1));
        (r0..=r1).flat_map(move |r| (c0..=c1).map(move |c| (r, c)))
    }

    /// The text field over the cell being edited: Enter, Tab and Esc end it (read before the
    /// field, which would take them), a click elsewhere keeps what was typed.
    fn field(&mut self, ui: &mut egui::Ui, rect: Rect, rows: usize, columns: usize, out: &mut Output) {
        let id = egui::Id::new("datum_table_field");
        let end = ui.input_mut(|i| {
            if i.consume_key(Modifiers::NONE, Key::Enter) {
                Some(End::Down)
            } else if i.consume_key(Modifiers::SHIFT, Key::Tab) {
                Some(End::Left)
            } else if i.consume_key(Modifiers::NONE, Key::Tab) {
                Some(End::Right)
            } else if i.consume_key(Modifiers::NONE, Key::Escape) {
                Some(End::Cancel)
            } else {
                None
            }
        });
        let Some(editing) = &mut self.editing else { return };
        if std::mem::take(&mut editing.fresh) {
            ui.memory_mut(|m| m.request_focus(id));
        }
        let r = ui.put(rect, egui::TextEdit::singleline(&mut editing.text).id(id).lock_focus(true).desired_width(rect.width()));
        if end.is_none() && !r.lost_focus() {
            return;
        }
        let text = std::mem::take(&mut editing.text);
        self.editing = None;
        if !matches!(end, Some(End::Cancel)) {
            out.edits.push((self.cursor.0, self.cursor.1, text));
        }
        let by = match end {
            Some(End::Down) => (1, 0),
            _ => (0, 0),
        };
        match end {
            Some(End::Right) => self.next((rows, columns), true),
            Some(End::Left) => self.next((rows, columns), false),
            _ => self.step(by, (rows, columns), false),
        }
    }

    fn paint(
        &self,
        ui: &egui::Ui,
        table: &Table,
        editable: &dyn Fn(usize, usize) -> bool,
        lay: &datum_table::Layout,
        on_screen: &dyn Fn(datum_table::Rect) -> Rect,
        rect: Rect,
    ) {
        let v = ui.visuals();
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, v.extreme_bg_color);
        if let Some(h) = lay.cell(0, 0).zip(lay.cell(0, table.columns().len() - 1)) {
            painter.rect_filled(on_screen(datum_table::Rect { w: h.1.x + h.1.w - h.0.x, ..h.0 }), 0.0, v.widgets.noninteractive.weak_bg_fill);
        }
        let corner = |at: (usize, usize)| lay.cell(at.0, at.1).map(on_screen).unwrap_or(Rect::NOTHING);
        painter.rect_filled(corner(self.anchor).union(corner(self.cursor)), 0.0, v.selection.bg_fill.gamma_multiply(0.5));
        let grid = Stroke::new(1.0, v.widgets.noninteractive.bg_stroke.color);
        for [[x0, y0], [x1, y1]] in &lay.lines {
            let p = |x: f64, y: f64| rect.min + Vec2::new(x as f32, y as f32) * SCALE;
            painter.line_segment([p(*x0, *y0), p(*x1, *y1)], grid);
        }
        let font = TextStyle::Body.resolve(ui.style());
        for r in 0..table.rows().len() {
            for c in 0..table.columns().len() {
                let cell = corner((r, c));
                let color = if r == 0 {
                    v.strong_text_color()
                } else if editable(r, c) {
                    v.text_color()
                } else {
                    v.weak_text_color()
                };
                painter.with_clip_rect(cell.intersect(rect)).text(
                    egui::pos2(cell.min.x + 4.0, cell.center().y),
                    Align2::LEFT_CENTER,
                    table.cell(r, c),
                    font.clone(),
                    color,
                );
            }
        }
        painter.rect_stroke(corner(self.cursor), 0.0, Stroke::new(2.0, v.selection.stroke.color), StrokeKind::Inside);
    }
}

/// The columns of the parameters table, in millimetres: name, value, result.
const WIDTHS: [f64; 3] = [30.0, 50.0, 25.0];

/// The model's parameters (`inspect`) as a table: a header, a row for each parameter and a last,
/// empty row for a new one, which shows what was typed in it so far (`pending`).
pub fn parameters(inspect: &Json, pending: &(String, String), i18n: &I18n) -> Table {
    let params = inspect["parameters"].as_object();
    let mut t = Table::new(WIDTHS.to_vec(), params.map_or(0, |p| p.len()) + 2);
    t.set(0, 0, i18n.t("table-name", "Name"));
    t.set(0, 1, i18n.t("table-value", "Value"));
    t.set(0, 2, i18n.t("table-result", "Result"));
    for (k, (name, p)) in params.into_iter().flatten().enumerate() {
        t.set(k + 1, 0, name.as_str());
        t.set(k + 1, 1, definition(&p["definition"]));
        t.set(k + 1, 2, p["value"].as_f64().map(number).or_else(|| p["error"].as_str().map(String::from)).unwrap_or_default());
    }
    let last = t.rows().len() - 1;
    t.set(last, 0, pending.0.as_str());
    t.set(last, 1, pending.1.as_str());
    t
}

/// The cells of a parameters table of `rows` rows that can be edited: the values, and the new
/// row's name; the new row is the last and any beyond it.
pub fn editable(rows: usize) -> impl Fn(usize, usize) -> bool {
    move |r, c| (c == 1 && r >= 1) || (c == 0 && r + 1 >= rows)
}

/// What the edits do: `SetParameter` for a changed value and for each new parameter. A new
/// row's name and value typed in separate edits wait in `pending`, the first new row's, until
/// there is a name. The second is why nothing was sent, if a name was taken.
pub fn parameter_requests(inspect: &Json, pending: &mut (String, String), edits: &[Edit], i18n: &I18n) -> (Vec<Request>, Option<String>) {
    let params: Vec<(&String, &Json)> = inspect["parameters"].as_object().into_iter().flatten().collect();
    let first_new = params.len() + 1;
    let mut sent = Vec::new();
    let mut new: BTreeMap<usize, (String, String)> = BTreeMap::new();
    for (r, c, text) in edits {
        match (r.checked_sub(1).and_then(|k| params.get(k)), c) {
            // Whether the value is changed is the model's to see; a parameter needs one.
            (Some((name, p)), 1) => {
                if !text.trim().is_empty() && *text != definition(&p["definition"]) {
                    sent.push(Request::SetParameter((*name).clone(), value_of(text)));
                }
            }
            (None, 0 | 1) if *r >= first_new => {
                let row = new.entry(*r).or_insert_with(|| if *r == first_new { pending.clone() } else { Default::default() });
                if *c == 0 { row.0.clone_from(text) } else { row.1.clone_from(text) }
            }
            _ => {}
        }
    }
    let mut error = None;
    let mut taken: BTreeSet<String> = params.iter().map(|(n, _)| (*n).clone()).collect();
    for (r, (name, value)) in new {
        let name = name.trim().to_string();
        if r == first_new {
            *pending = (name.clone(), value.clone());
        }
        if name.is_empty() {
            continue;
        }
        if !taken.insert(name.clone()) {
            error = Some(i18n.f("table-taken", "`{ $name }` is a parameter already", &[("name", &name)]));
        } else {
            if r == first_new {
                *pending = Default::default();
            }
            sent.push(Request::SetParameter(name, value_of(if value.trim().is_empty() { "0" } else { &value })));
        }
    }
    (sent, error)
}

/// The commands removing the parameters in `rows` of the table. In a variant only its own can
/// be removed, as in the side bar.
pub fn remove_requests(inspect: &Json, rows: &[usize]) -> Vec<Request> {
    let own = marks(inspect).3;
    let level = inspect["level"].as_str();
    let params: Vec<&String> = inspect["parameters"].as_object().into_iter().flat_map(|p| p.keys()).collect();
    rows.iter()
        .filter_map(|r| params.get(r.checked_sub(1)?))
        .filter(|name| level.is_none() || own.contains(name))
        .map(|name| Request::Command("remove_parameter", json!({"name": name})))
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn inspect() -> Json {
        json!({"parameters": {
            "a": {"definition": 5.0, "value": 5.0},
            "b": {"definition": "a * 2", "value": 10.0},
            "c": {"definition": "zz", "error": "unknown `zz`"},
        }})
    }

    fn set(name: &str, v: Json) -> Request {
        Request::SetParameter(name.into(), v)
    }

    fn same(got: &[Request], want: &[Request]) {
        assert_eq!(format!("{got:?}"), format!("{want:?}"));
    }

    fn requests(edits: &[(usize, usize, &str)], pending: &mut (String, String)) -> (Vec<Request>, Option<String>) {
        let edits: Vec<Edit> = edits.iter().map(|&(r, c, t)| (r, c, t.to_string())).collect();
        parameter_requests(&inspect(), pending, &edits, &I18n::default())
    }

    #[test]
    fn the_parameters_are_a_table() {
        let t = parameters(&inspect(), &("n".into(), "1".into()), &I18n::default());
        assert_eq!(t.rows().len(), 5, "header, three parameters, a new one");
        assert_eq!(t.copy((0, 0), (0, 2)), "Name\tValue\tResult");
        assert_eq!(t.copy((1, 0), (3, 2)), "a\t5\t5\nb\ta * 2\t10\nc\tzz\tunknown `zz`");
        assert_eq!(t.copy((4, 0), (4, 2)), "n\t1\t");
        let ok = editable(t.rows().len());
        assert!(ok(2, 1) && ok(4, 0) && ok(5, 0) && ok(6, 1));
        assert!(!ok(2, 0) && !ok(2, 2) && !ok(0, 1) && !ok(4, 2));
    }

    #[test]
    fn a_changed_value_is_set() {
        let mut pending = Default::default();
        let (sent, error) = requests(&[(1, 1, "7,5"), (2, 1, "a * 2"), (3, 1, " "), (1, 0, "x"), (1, 2, "9")], &mut pending);
        same(&sent, &[set("a", json!(7.5))]);
        assert_eq!(error, None, "unchanged, empty and read-only cells send nothing");
    }

    #[test]
    fn a_new_parameter_has_a_name_and_a_value() {
        let mut pending = Default::default();
        same(&requests(&[(4, 0, "w"), (4, 1, "a + 1")], &mut pending).0, &[set("w", json!("a + 1"))]);
        assert_eq!(pending, Default::default());
        same(&requests(&[(4, 0, "w")], &mut pending).0, &[set("w", json!(0.0))]);
        // A value without a name stays in the row; the name comes later.
        let (sent, _) = requests(&[(4, 1, "3")], &mut pending);
        assert!(sent.is_empty() && pending == (String::new(), "3".into()));
        same(&requests(&[(4, 0, "z")], &mut pending).0, &[set("z", json!(3.0))]);
    }

    #[test]
    fn a_taken_name_is_an_error() {
        let mut pending = Default::default();
        let (sent, error) = requests(&[(4, 0, "a"), (4, 1, "2")], &mut pending);
        assert!(sent.is_empty());
        assert_eq!(error.as_deref(), Some("`a` is a parameter already"));
        assert_eq!(pending, ("a".into(), "2".into()), "kept to be renamed");
    }

    #[test]
    fn a_pasted_block_adds_parameters() {
        let mut pending = Default::default();
        let (sent, error) = requests(&[(4, 0, "p"), (4, 1, "1"), (5, 0, "q"), (5, 1, "p * 2"), (6, 0, "p"), (6, 1, "8")], &mut pending);
        same(&sent, &[set("p", json!(1.0)), set("q", json!("p * 2"))]);
        assert_eq!(error.as_deref(), Some("`p` is a parameter already"), "the third repeats the first");
    }

    #[test]
    fn clearing_a_value_is_refused() {
        let mut pending = ("k".into(), "1".into());
        let (sent, _) = requests(&[(1, 1, ""), (4, 0, ""), (4, 1, "")], &mut pending);
        assert!(sent.is_empty() && pending == Default::default(), "the new row cleared");
    }

    #[test]
    fn rows_are_removed_but_a_variants_inherited() {
        let removed = |i: &Json, rows: &[usize]| format!("{:?}", remove_requests(i, rows));
        assert_eq!(
            removed(&inspect(), &[0, 1, 3, 4]),
            format!("{:?}", [Request::Command("remove_parameter", json!({"name": "a"})), Request::Command("remove_parameter", json!({"name": "c"}))])
        );
        let mut variant = inspect();
        variant["level"] = json!("v");
        variant["levels"] = json!([{"name": "v", "own_parameters": ["b"]}]);
        assert_eq!(removed(&variant, &[1, 2, 3]), format!("{:?}", [Request::Command("remove_parameter", json!({"name": "b"}))]));
    }
}
