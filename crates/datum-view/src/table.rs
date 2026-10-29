//! The table editor (D178): a window of its own showing a `datum_table::Table` as a grid,
//! edited as in a spreadsheet, and the first table it is bound to: the model's parameters.
//! The editor only reports what was edited and which rows to delete; what that does to the
//! model is the source's mapping to requests below.

use std::collections::{BTreeMap, BTreeSet};

use datum_table::{Line, Piece, Slot, Table, layout, parse_tsv, pieces, place, text_of};
use egui::{Align2, Button, Color32, Event, FontId, Key, Modifiers, Pos2, Rect, Sense, Stroke, StrokeKind, TextStyle, Vec2};
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

/// The narrowest a part's field is, in millimetres: an empty part is a sliver to type into.
const FIELD_MIN: f32 = 10.0;

/// A cell being edited, and whether its field still has to take the keyboard. With a `part` of
/// the cell's `pieces` only that part is edited (its text is `text`, `pieces` holds the rest),
/// otherwise the whole text of the cell. `caret` is the text cursor, in characters.
struct Editing {
    at: (usize, usize),
    pieces: Vec<Piece>,
    part: Option<(usize, Slot)>,
    text: String,
    caret: usize,
    fresh: bool,
}

impl Editing {
    /// The cell's pieces with the edited text in place.
    fn current(&self) -> Vec<Piece> {
        match self.part {
            Some((i, slot)) => with_part(self.pieces.clone(), i, slot, self.text.clone()),
            None => pieces(&self.text),
        }
    }
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
        let can = editable(self.cursor.0, self.cursor.1);
        let button = ui.add_enabled(can, Button::new(i18n.t("table-fraction", "Fraction")));
        // Pressing it ends the field's focus, so while a cell is edited it acts on the press, with the
        // text cursor the field still has.
        let pressed = ui.input(|i| i.pointer.primary_pressed());
        if can && if self.editing.is_some() { button.hovered() && pressed } else { button.clicked() } {
            self.add_fraction(table);
        }
        let moved = self.editing.is_none() && ui.input(|i| i.viewport().focused != Some(false)) && self.keys(ui, table, editable, &mut out);
        let error = ui.visuals().error_fg_color;
        let line = ui.spacing().interact_size.y + ui.spacing().item_spacing.y;
        egui::ScrollArea::both().auto_shrink([false, false]).max_height(ui.available_height() - line).show(ui, |ui| {
            let size = lay.size();
            let (rect, resp) = ui.allocate_exact_size(Vec2::new(size[0] as f32, size[1] as f32) * SCALE, Sense::click_and_drag());
            let on_screen = |r| on_screen(rect.min, r);
            let cell_rect = |at: (usize, usize)| lay.cell(at.0, at.1).map(on_screen).unwrap_or(Rect::NOTHING);
            let mm = |p: Pos2| [f64::from((p.x - rect.min.x) / SCALE), f64::from((p.y - rect.min.y) / SCALE)];
            let at = |p: Pos2| lay.at(mm(p));
            let laid = self.laid_out(ui, table, &lay);
            self.paint(ui, table, editable, &lay, &laid, rect);
            if moved {
                ui.scroll_to_rect(cell_rect(self.cursor), None);
            }
            if let Some(e) = &self.editing {
                let (area, font) = match e.part {
                    Some((i, slot)) => {
                        let b = laid[e.at.0 * columns + e.at.1].1.boxes.iter().find(|b| b.piece == i && b.slot == slot);
                        let b = b.map_or(Rect::NOTHING, |b| on_screen(b.rect));
                        let size = Vec2::new(b.width().max(FIELD_MIN * SCALE), b.height().max(ui.spacing().interact_size.y));
                        (Rect::from_center_size(b.center() + Vec2::new((size.x - b.width()) / 2.0, 0.0), size), Some(b.height()))
                    }
                    None => (cell_rect(e.at), None),
                };
                let font = font.map(|h| FontId::new(h, TextStyle::Body.resolve(ui.style()).family));
                self.field(ui, area, font, (rows, columns), &mut out);
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
                let hit = pos.and_then(|p| laid[self.cursor.0 * columns + self.cursor.1].1.at(mm(p)));
                self.start(table, self.cursor, hit.map(|b| (b.piece, b.slot)));
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

    /// Every cell's pieces and where they go, the edited cell's with the text typed so far.
    fn laid_out(&self, ui: &egui::Ui, table: &Table, lay: &datum_table::Layout) -> Vec<(Vec<Piece>, Line)> {
        let font = TextStyle::Body.resolve(ui.style());
        let width = |t: &str, h: f64| {
            let galley = ui.painter().layout_no_wrap(t.to_string(), FontId::new(h as f32 * SCALE, font.family.clone()), Color32::PLACEHOLDER);
            f64::from(galley.size().x / SCALE)
        };
        let edited = self.editing.as_ref().filter(|e| e.part.is_some());
        let (rows, columns) = (table.rows().len(), table.columns().len());
        (0..rows * columns)
            .map(|k| {
                let at = (k / columns, k % columns);
                let ps = edited.filter(|e| e.at == at).map_or_else(|| pieces(table.cell(at.0, at.1)), Editing::current);
                let line = lay.cell(at.0, at.1).map(|cell| place(&ps, cell, f64::from(font.size / SCALE), &width)).unwrap_or_default();
                (ps, line)
            })
            .collect()
    }

    /// Editing the cell `at`: its first part if it has a fraction, or `hit`, the part clicked;
    /// the whole text otherwise.
    fn start(&mut self, table: &Table, at: (usize, usize), hit: Option<(usize, Slot)>) {
        let mut ps = pieces(table.cell(at.0, at.1));
        let first = match ps.first() {
            Some(Piece::Fraction { .. }) => Slot::Over,
            _ => Slot::Text,
        };
        let part = ps.iter().any(|p| matches!(p, Piece::Fraction { .. })).then_some(hit.unwrap_or((0, first)));
        let text = match part {
            Some((i, slot)) => std::mem::take(part_mut(&mut ps[i], slot)),
            None => table.cell(at.0, at.1).to_string(),
        };
        self.editing = Some(Editing { at, pieces: ps, part, caret: text.chars().count(), text, fresh: true });
    }

    /// Editing the cursor cell, starting from `text`, which replaces its own.
    fn edit(&mut self, text: String) {
        self.editing = Some(Editing { at: self.cursor, pieces: Vec::new(), part: None, caret: text.chars().count(), text, fresh: true });
    }

    /// The Fraction button: a new empty fraction at the text cursor of the part being edited, or at
    /// the end of the cursor cell's line; its upper part is edited next.
    fn add_fraction(&mut self, table: &Table) {
        let (at, ps, new) = match self.editing.take() {
            Some(Editing { at, pieces, part: Some((i, slot)), text, caret, .. }) => {
                let (ps, new) = split_text(with_part(pieces, i, slot, text), i, caret);
                (at, ps, new)
            }
            e => {
                let at = e.as_ref().map_or(self.cursor, |e| e.at);
                let ps = pieces(&e.map_or_else(|| table.cell(at.0, at.1).to_string(), |e| e.text));
                let n = ps.len();
                let (ps, new) = insert_fraction(ps, n);
                (at, ps, new)
            }
        };
        self.editing = Some(Editing { at, pieces: ps, part: Some((new, Slot::Over)), text: String::new(), caret: 0, fresh: true });
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
            self.start(table, (r, c), None);
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

    /// The text field over the cell or the part being edited, in `font` if the part has its own:
    /// Enter, Tab and Esc end it (read before the field, which would take them), a click elsewhere
    /// keeps what was typed.
    fn field(&mut self, ui: &mut egui::Ui, area: Rect, font: Option<FontId>, (rows, columns): (usize, usize), out: &mut Output) {
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
        if editing.fresh {
            ui.memory_mut(|m| m.request_focus(id));
        }
        let mut field = egui::TextEdit::singleline(&mut editing.text).id(id).lock_focus(true).desired_width(area.width());
        if let Some(font) = font {
            field = field.font(font);
        }
        let r = ui.put(area, field);
        editing.fresh &= !r.has_focus();
        if let Some(range) = egui::TextEdit::load_state(ui.ctx(), id).and_then(|s| s.cursor.char_range()) {
            editing.caret = range.primary.index.0;
        }
        if end.is_none() && (editing.fresh || !r.lost_focus()) {
            return;
        }
        let Some(editing) = self.editing.take() else { return };
        if !matches!(end, Some(End::Cancel)) {
            let text = if editing.part.is_some() { text_of(&editing.current()) } else { editing.text };
            out.edits.push((editing.at.0, editing.at.1, text));
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
        laid: &[(Vec<Piece>, Line)],
        rect: Rect,
    ) {
        let on_screen = |r| on_screen(rect.min, r);
        let v = ui.visuals();
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, v.extreme_bg_color);
        if let Some(h) = lay.cell(0, 0).zip(lay.cell(0, table.columns().len() - 1)) {
            painter.rect_filled(on_screen(datum_table::Rect { w: h.1.x + h.1.w - h.0.x, ..h.0 }), 0.0, v.widgets.noninteractive.weak_bg_fill);
        }
        let corner = |at: (usize, usize)| lay.cell(at.0, at.1).map(on_screen).unwrap_or(Rect::NOTHING);
        painter.rect_filled(corner(self.anchor).union(corner(self.cursor)), 0.0, v.selection.bg_fill.gamma_multiply(0.5));
        let grid = Stroke::new(1.0, v.widgets.noninteractive.bg_stroke.color);
        let p = |[x, y]: [f64; 2]| rect.min + Vec2::new(x as f32, y as f32) * SCALE;
        for [from, to] in &lay.lines {
            painter.line_segment([p(*from), p(*to)], grid);
        }
        let font = TextStyle::Body.resolve(ui.style());
        let columns = table.columns().len();
        for r in 0..table.rows().len() {
            for c in 0..columns {
                let cell = corner((r, c));
                let color = if r == 0 {
                    v.strong_text_color()
                } else if editable(r, c) {
                    v.text_color()
                } else {
                    v.weak_text_color()
                };
                let clip = painter.with_clip_rect(cell.intersect(rect));
                let (ps, line) = &laid[r * columns + c];
                for b in &line.boxes {
                    let area = on_screen(b.rect);
                    clip.text(
                        area.center(),
                        Align2::CENTER_CENTER,
                        part(&ps[b.piece], b.slot),
                        FontId::new(area.height(), font.family.clone()),
                        color,
                    );
                }
                for [from, to] in &line.bars {
                    clip.line_segment([p(*from), p(*to)], Stroke::new(1.0, color));
                }
            }
        }
        painter.rect_stroke(corner(self.cursor), 0.0, Stroke::new(2.0, v.selection.stroke.color), StrokeKind::Inside);
    }
}

/// Where table rectangle `r` (millimetres) is on screen, the table's top left being `origin`.
fn on_screen(origin: Pos2, r: datum_table::Rect) -> Rect {
    Rect::from_min_size(origin + Vec2::new(r.x as f32, r.y as f32) * SCALE, Vec2::new(r.w as f32, r.h as f32) * SCALE)
}

/// The text of `piece`'s part `slot`.
fn part(piece: &Piece, slot: Slot) -> &str {
    match (piece, slot) {
        (Piece::Text(t), _) => t,
        (Piece::Fraction { under, .. }, Slot::Under) => under,
        (Piece::Fraction { over, .. }, _) => over,
    }
}

fn part_mut(piece: &mut Piece, slot: Slot) -> &mut String {
    match (piece, slot) {
        (Piece::Text(t), _) => t,
        (Piece::Fraction { under, .. }, Slot::Under) => under,
        (Piece::Fraction { over, .. }, _) => over,
    }
}

/// `pieces` with the text of piece `i`'s part `slot` replaced.
fn with_part(mut pieces: Vec<Piece>, i: usize, slot: Slot, text: String) -> Vec<Piece> {
    *part_mut(&mut pieces[i], slot) = text;
    pieces
}

/// `pieces` with an empty fraction inserted at `index`, and `index`.
fn insert_fraction(mut pieces: Vec<Piece>, index: usize) -> (Vec<Piece>, usize) {
    pieces.insert(index, Piece::Fraction { over: String::new(), under: String::new() });
    (pieces, index)
}

/// `pieces` with an empty fraction where text piece `i` is cut at character `caret`, and the
/// fraction's index; after piece `i` if that is a fraction.
fn split_text(mut pieces: Vec<Piece>, i: usize, caret: usize) -> (Vec<Piece>, usize) {
    let Piece::Text(text) = &pieces[i] else { return insert_fraction(pieces, i + 1) };
    let cut = text.char_indices().nth(caret).map_or(text.len(), |(b, _)| b);
    let (before, after) = (text[..cut].to_string(), text[cut..].to_string());
    let mut new = i;
    let mut parts = Vec::new();
    if !before.is_empty() {
        parts.push(Piece::Text(before));
        new += 1;
    }
    parts.push(Piece::Fraction { over: String::new(), under: String::new() });
    if !after.is_empty() {
        parts.push(Piece::Text(after));
    }
    pieces.splice(i..=i, parts);
    (pieces, new)
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

/// What the table window shows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Source {
    #[default]
    Parameters,
    Materials,
}

/// A new material's cells as typed so far, by column.
pub type Pending = [String; 4];

/// The columns of the materials table, in millimetres: name, designation, density, description.
const MATERIAL_WIDTHS: [f64; 4] = [30.0, 90.0, 35.0, 50.0];

/// The keys of a material's JSON the columns show.
const FIELDS: [&str; 4] = ["name", "designation", "density", "description"];

/// What a material's columns show.
fn material_cells(e: &Json) -> Pending {
    let text = |k: &str| e[k].as_str().unwrap_or_default().to_string();
    [text("name"), text("designation"), e["density"].as_f64().map(number).unwrap_or_default(), text("description")]
}

/// The library's materials (`elements`, in library order) as a table: a header, a row for each
/// and a last, empty row for a new one, which shows what was typed in it so far (`pending`).
pub fn materials(elements: &[Json], pending: &Pending, i18n: &I18n) -> Table {
    let mut t = Table::new(MATERIAL_WIDTHS.to_vec(), elements.len() + 2);
    let header = [
        i18n.t("table-name", "Name"),
        i18n.t("table-designation", "Designation"),
        i18n.t("table-density", "Density (kg/m³)"),
        i18n.t("table-description", "Description"),
    ];
    for (c, h) in header.iter().enumerate() {
        t.set(0, c, &**h);
    }
    for (k, e) in elements.iter().enumerate() {
        for (c, text) in material_cells(e).into_iter().enumerate() {
            t.set(k + 1, c, text);
        }
    }
    let last = t.rows().len() - 1;
    for (c, text) in pending.iter().enumerate() {
        t.set(last, c, text.as_str());
    }
    t
}

/// The cells of a materials table that can be edited: all but the header's.
pub fn editable_materials(r: usize, _: usize) -> bool {
    r >= 1
}

/// Material field `c` as typed: its value, none for an emptied one (the key is dropped), or why
/// not. The name is needed, the density is a number above zero (a comma will do).
fn field_value(c: usize, text: &str, i18n: &I18n) -> Result<Option<Json>, String> {
    let text = text.trim();
    match c {
        0 if text.is_empty() => Err(i18n.t("table-material-nameless", "A material needs a name").to_string()),
        2 if !text.is_empty() => match text.replace(',', ".").parse::<f64>() {
            Ok(d) if d.is_finite() && d > 0.0 => Ok(Some(json!(d))),
            _ => Err(i18n.t("table-material-density", "The density is a number above zero, in kg/m³").to_string()),
        },
        _ => Ok((!text.is_empty()).then(|| json!(text))),
    }
}

/// Sets field `c` of material `e` to `v`, or drops it.
fn set_field(e: &mut Json, c: usize, v: Option<Json>) {
    if let Some(o) = e.as_object_mut() {
        match v {
            Some(v) => o.insert(FIELDS[c].into(), v),
            None => o.remove(FIELDS[c]),
        };
    }
}

fn put_library(element: Json, was: Option<&str>) -> Request {
    let mut params = json!({"level": "material", "element": element});
    if let Some(was) = was {
        params["was"] = json!(was);
    }
    Request::Command("put_library", params)
}

/// What the edits do: `put_library` of the material with the changed fields (one request for a
/// row), and of a new material once its row has a name. The new row's cells wait in `pending`,
/// the first new row's, until then. The second is the last error: an emptied name, a bad density,
/// a name taken; what is refused is not sent.
pub fn material_requests(elements: &[Json], pending: &mut Pending, edits: &[Edit], i18n: &I18n) -> (Vec<Request>, Option<String>) {
    let first_new = elements.len() + 1;
    let taken = |name: &str| i18n.f("table-material-taken", "`{ $name }` is a material already", &[("name", name)]);
    let mut names: Vec<String> = elements.iter().map(|e| material_cells(e)[0].clone()).collect();
    let mut changed: BTreeMap<usize, Json> = BTreeMap::new();
    let mut new: BTreeMap<usize, Pending> = BTreeMap::new();
    let mut error = None;
    for (r, c, text) in edits {
        if *c >= FIELDS.len() {
            continue;
        }
        if let Some(k) = r.checked_sub(1).filter(|&k| k < elements.len()) {
            let e = changed.get(&k).unwrap_or(&elements[k]);
            if material_cells(e)[*c] == text.trim() {
                continue;
            }
            match field_value(*c, text, i18n) {
                Err(why) => error = Some(why),
                Ok(_) if *c == 0 && names.iter().enumerate().any(|(j, n)| j != k && n == text.trim()) => error = Some(taken(text.trim())),
                Ok(v) => {
                    if *c == 0 {
                        names[k] = text.trim().to_string();
                    }
                    set_field(changed.entry(k).or_insert_with(|| elements[k].clone()), *c, v);
                }
            }
        } else if *r >= first_new {
            let row = new.entry(*r).or_insert_with(|| if *r == first_new { pending.clone() } else { Default::default() });
            row[*c].clone_from(text);
        }
    }
    let mut sent: Vec<Request> = changed.into_iter().map(|(k, e)| put_library(e, Some(&material_cells(&elements[k])[0]))).collect();
    for (r, mut row) in new {
        row[0] = row[0].trim().to_string();
        if r == first_new {
            pending.clone_from(&row);
        }
        if row[0].is_empty() {
            continue;
        }
        if names.contains(&row[0]) {
            error = Some(taken(&row[0]));
            continue;
        }
        let mut element = json!({"name": row[0]});
        let fields: Result<Vec<_>, _> = (1..FIELDS.len()).map(|c| field_value(c, &row[c], i18n)).collect();
        match fields {
            Err(why) => error = Some(why),
            Ok(values) => {
                for (c, v) in (1..).zip(values) {
                    set_field(&mut element, c, v);
                }
                if r == first_new {
                    *pending = Default::default();
                }
                names.push(row[0].clone());
                sent.push(put_library(element, None));
            }
        }
    }
    (sent, error)
}

/// The commands removing the materials in `rows` of the table.
pub fn remove_material_requests(elements: &[Json], rows: &[usize]) -> Vec<Request> {
    rows.iter()
        .filter_map(|r| elements.get(r.checked_sub(1)?))
        .map(|e| Request::Command("remove_library", json!({"level": "material", "name": material_cells(e)[0]})))
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use egui_kittest::kittest::Queryable;

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

    fn text(t: &str) -> Piece {
        Piece::Text(t.into())
    }

    fn fraction(over: &str, under: &str) -> Piece {
        Piece::Fraction { over: over.into(), under: under.into() }
    }

    #[test]
    fn a_part_is_replaced() {
        let ps = vec![text("Лист "), fraction("a", "б")];
        assert_eq!(with_part(ps.clone(), 1, Slot::Under, "в".into()), vec![text("Лист "), fraction("a", "в")]);
        assert_eq!(with_part(ps, 0, Slot::Text, "Круг".into()), vec![text("Круг"), fraction("a", "б")]);
    }

    #[test]
    fn a_fraction_splits_the_text_at_the_cursor() {
        let (ps, new) = split_text(vec![text("Лист ГОСТ")], 0, 5);
        assert_eq!((ps, new), (vec![text("Лист "), fraction("", ""), text("ГОСТ")], 1));
        let (ps, new) = split_text(vec![fraction("a", "б"), text("Лист")], 1, 0);
        assert_eq!((ps, new), (vec![fraction("a", "б"), fraction("", ""), text("Лист")], 1));
        let (ps, new) = split_text(vec![text("Лист")], 0, 4);
        assert_eq!((ps, new), (vec![text("Лист"), fraction("", "")], 1));
    }

    #[test]
    fn a_fraction_goes_after_a_fraction_or_at_the_end() {
        let ps = vec![fraction("a", "б"), text("Лист")];
        assert_eq!(split_text(ps.clone(), 0, 3), (vec![fraction("a", "б"), fraction("", ""), text("Лист")], 1));
        assert_eq!(insert_fraction(ps, 2), (vec![fraction("a", "б"), text("Лист"), fraction("", "")], 2));
        assert_eq!(insert_fraction(Vec::new(), 0), (vec![fraction("", "")], 0));
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

    fn library() -> Vec<Json> {
        vec![
            json!({"name": "Сталь 3", "designation": "Лист [ГОСТ 19903 | Ст3]", "density": 7850.0, "description": "общая"}),
            json!({"name": "АМг2", "density": 2670.0}),
        ]
    }

    fn material_edits(edits: &[(usize, usize, &str)], pending: &mut Pending) -> (Vec<Request>, Option<String>) {
        let edits: Vec<Edit> = edits.iter().map(|&(r, c, t)| (r, c, t.to_string())).collect();
        material_requests(&library(), pending, &edits, &I18n::default())
    }

    #[test]
    fn the_materials_are_a_table() {
        let pending = ["Н".into(), String::new(), "1,5".into(), String::new()];
        let t = materials(&library(), &pending, &I18n::default());
        assert_eq!(t.rows().len(), 4, "header, two materials, a new one");
        assert_eq!(t.copy((0, 0), (0, 3)), "Name\tDesignation\tDensity (kg/m³)\tDescription");
        assert_eq!(t.copy((1, 0), (2, 3)), "Сталь 3\tЛист [ГОСТ 19903 | Ст3]\t7850\tобщая\nАМг2\t\t2670\t");
        assert_eq!(t.copy((3, 0), (3, 3)), "Н\t\t1,5\t");
        assert!(editable_materials(1, 0) && editable_materials(3, 3) && !editable_materials(0, 1));
    }

    #[test]
    fn a_changed_cell_puts_the_material() {
        let mut pending = Default::default();
        let (sent, error) = material_edits(&[(2, 2, "2,7"), (2, 1, "Лист"), (2, 3, "")], &mut pending);
        let want = put_library(json!({"name": "АМг2", "designation": "Лист", "density": 2.7}), Some("АМг2"));
        same(&sent, &[want]);
        assert_eq!(error, None);
        let (sent, _) = material_edits(&[(1, 1, ""), (1, 3, ""), (1, 0, "Сталь 20")], &mut pending);
        same(&sent, &[put_library(json!({"name": "Сталь 20", "density": 7850.0}), Some("Сталь 3"))]);
    }

    #[test]
    fn an_unchanged_or_refused_cell_sends_nothing() {
        let mut pending = Default::default();
        let (sent, error) = material_edits(&[(1, 0, "Сталь 3"), (1, 2, "7850"), (3, 5, "x")], &mut pending);
        assert!(sent.is_empty() && error.is_none());
        for (edit, why) in [
            ((1, 0, " "), "A material needs a name"),
            ((1, 2, "много"), "The density is a number above zero, in kg/m³"),
            ((1, 2, "0"), "The density is a number above zero, in kg/m³"),
            ((1, 0, "АМг2"), "`АМг2` is a material already"),
        ] {
            let (sent, error) = material_edits(&[edit], &mut pending);
            assert!(sent.is_empty(), "{edit:?}");
            assert_eq!(error.as_deref(), Some(why));
        }
    }

    #[test]
    fn a_new_material_waits_for_its_name() {
        let mut pending = Default::default();
        let (sent, _) = material_edits(&[(3, 2, "8,0"), (3, 1, "Круг")], &mut pending);
        assert!(sent.is_empty() && pending == ["", "Круг", "8,0", ""].map(String::from));
        let (sent, _) = material_edits(&[(3, 0, " Сталь 45 ")], &mut pending);
        same(&sent, &[put_library(json!({"name": "Сталь 45", "designation": "Круг", "density": 8.0}), None)]);
        assert_eq!(pending, Pending::default());
    }

    #[test]
    fn a_new_material_with_a_bad_field_or_a_taken_name_stays() {
        let mut pending = Default::default();
        let (sent, error) = material_edits(&[(3, 0, "М1"), (3, 2, "-1")], &mut pending);
        assert!(sent.is_empty() && error.is_some() && pending[0] == "М1", "kept to be corrected");
        let (sent, error) = material_edits(&[(3, 0, "АМг2")], &mut pending);
        assert!(sent.is_empty());
        assert_eq!(error.as_deref(), Some("`АМг2` is a material already"));
    }

    #[test]
    fn materials_are_removed_by_name() {
        let got = remove_material_requests(&library(), &[0, 2, 7]);
        same(&got, &[Request::Command("remove_library", json!({"level": "material", "name": "АМг2"}))]);
    }

    /// The editor as the window shows it: a material's fraction drawn, its under line edited by
    /// a double-click on it, and the Fraction button pressed while a text part is edited.
    #[test]
    fn a_fraction_is_edited_where_it_is_clicked() {
        let mut t = materials(&library(), &Pending::default(), &I18n::default());
        t.set(1, 1, "Лист [Б-ПН | Ст3]");
        let lay = layout(&t);
        let state = (Editor::default(), t.clone(), Vec::<Edit>::new());
        let mut h = egui_kittest::Harness::builder().with_size([800.0, 200.0]).with_step_dt(1.0 / 60.0).build_ui_state(
            |ui, (editor, table, edits): &mut (Editor, Table, Vec<Edit>)| {
                edits.extend(editor.show(ui, table, &editable_materials, &I18n::default()).edits);
            },
            state,
        );
        h.run();
        // The cell's line laid out as the editor lays it out, under the button's row.
        let button = h.get_by_label("Fraction").rect();
        let origin = egui::pos2(button.min.x, button.max.y + h.ctx.style_of(h.ctx.theme()).spacing.item_spacing.y);
        let font = TextStyle::Body.resolve(&h.ctx.style_of(h.ctx.theme()));
        let width = |t: &str, mm: f64| {
            let g = h.ctx.fonts_mut(|f| f.layout_no_wrap(t.to_string(), FontId::new(mm as f32 * SCALE, font.family.clone()), Color32::PLACEHOLDER));
            f64::from(g.size().x / SCALE)
        };
        let line = place(&pieces(t.cell(1, 1)), lay.cell(1, 1).unwrap(), f64::from(font.size / SCALE), &width);
        let centre = |slot: Slot| on_screen(origin, line.boxes.iter().find(|b| b.slot == slot).unwrap().rect).center();
        let (under, word) = (centre(Slot::Under), centre(Slot::Text));
        let press = |h: &mut egui_kittest::Harness<'_, (Editor, Table, Vec<Edit>)>, at: egui::Pos2| {
            h.hover_at(at);
            for pressed in [true, false] {
                h.event(Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed, modifiers: Modifiers::NONE });
                h.step();
            }
        };
        let enter = |h: &mut egui_kittest::Harness<'_, (Editor, Table, Vec<Edit>)>, text: &str| {
            h.event(Event::Text(text.into()));
            h.step();
            h.event(Event::Key { key: Key::Enter, physical_key: None, pressed: true, repeat: false, modifiers: Modifiers::NONE });
            h.run();
        };
        press(&mut h, under);
        press(&mut h, under);
        h.run();
        enter(&mut h, "08пс");
        let edits = std::mem::take(&mut h.state_mut().2);
        assert_eq!(edits.len(), 1, "{edits:?}");
        let (r, c, text) = &edits[0];
        assert_eq!((*r, *c), (1, 1));
        assert!(text.starts_with("Лист [Б-ПН | ") && text.contains("08пс") && text.ends_with(']'), "the under line edited: {text}");
        press(&mut h, word);
        press(&mut h, word);
        h.run();
        press(&mut h, button.center());
        h.run();
        enter(&mut h, "x");
        let edits = std::mem::take(&mut h.state_mut().2);
        assert!(edits.iter().any(|(_, _, t)| t.contains("[x | ]")), "a new fraction, its over line typed: {edits:?}");
    }
}
