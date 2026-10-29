//! Tables (D178): one model and one layout for the table editor, drawings (the title block,
//! specifications) and exports. A table is a grid: column widths, row heights and the text of
//! each cell, in millimetres as on a sheet. The layout is a pure function of it, so the editor's
//! window, a sheet and a PDF draw the same cells. What a table shows (a model's parameters, the
//! library's materials) and what an edit does is its source's business, not this crate's.

/// The height of a row on a drawing by ESKD, and of a new row.
pub const ROW: f64 = 8.0;

/// A grid of cells: widths of its columns, heights of its rows, and each cell's text, row by row.
#[derive(Clone, Debug, PartialEq)]
pub struct Table {
    columns: Vec<f64>,
    rows: Vec<f64>,
    cells: Vec<String>,
}

impl Table {
    /// Columns of widths `columns` and `rows` rows of [`ROW`] height, every cell empty.
    pub fn new(columns: Vec<f64>, rows: usize) -> Self {
        let cells = vec![String::new(); columns.len() * rows];
        Self { columns, rows: vec![ROW; rows], cells }
    }

    pub fn columns(&self) -> &[f64] {
        &self.columns
    }

    pub fn rows(&self) -> &[f64] {
        &self.rows
    }

    /// The text of cell `(row, column)`; empty outside the table.
    pub fn cell(&self, row: usize, column: usize) -> &str {
        self.index(row, column).map_or("", |k| &self.cells[k])
    }

    /// Cell `(row, column)` given `text`; outside the table, nothing.
    pub fn set(&mut self, row: usize, column: usize, text: impl Into<String>) {
        if let Some(k) = self.index(row, column) {
            self.cells[k] = text.into();
        }
    }

    /// Column `column` made `width` wide.
    pub fn set_width(&mut self, column: usize, width: f64) {
        if let Some(w) = self.columns.get_mut(column) {
            *w = width;
        }
    }

    fn index(&self, row: usize, column: usize) -> Option<usize> {
        (row < self.rows.len() && column < self.columns.len()).then(|| row * self.columns.len() + column)
    }

    /// The text of the cells from `from` to `to` (corners, either way round), as Excel copies
    /// a range: cells by tabs, rows by line ends.
    pub fn copy(&self, from: (usize, usize), to: (usize, usize)) -> String {
        let (rows, columns) = (from.0.min(to.0)..=from.0.max(to.0), from.1.min(to.1)..=from.1.max(to.1));
        let block: Vec<Vec<&str>> = rows.map(|r| columns.clone().map(|c| self.cell(r, c)).collect()).collect();
        to_tsv(&block)
    }
}

/// A rectangle on the table, in millimetres from its top left corner, `y` down the rows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rect {
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
}

/// Where a table's cells and lines are: what every drawing of it draws.
#[derive(Clone, Debug, PartialEq)]
pub struct Layout {
    /// The left edge of each column and the right edge of the last: `columns + 1` values.
    pub xs: Vec<f64>,
    /// The top edge of each row and the bottom of the last: `rows + 1` values.
    pub ys: Vec<f64>,
    /// The grid's lines: each row's and column's edge across the whole table.
    pub lines: Vec<[[f64; 2]; 2]>,
}

impl Layout {
    /// Cell `(row, column)`'s rectangle.
    pub fn cell(&self, row: usize, column: usize) -> Option<Rect> {
        let (x, y) = (*self.xs.get(column)?, *self.ys.get(row)?);
        Some(Rect { x, y, w: self.xs.get(column + 1)? - x, h: self.ys.get(row + 1)? - y })
    }

    /// The whole table's width and height.
    pub fn size(&self) -> [f64; 2] {
        [self.xs.last().copied().unwrap_or(0.0), self.ys.last().copied().unwrap_or(0.0)]
    }

    /// The cell under point `p`, if it is on the table.
    pub fn at(&self, p: [f64; 2]) -> Option<(usize, usize)> {
        let find = |edges: &[f64], v: f64| {
            let k = edges.partition_point(|&e| e <= v);
            (k > 0 && k < edges.len()).then(|| k - 1)
        };
        Some((find(&self.ys, p[1])?, find(&self.xs, p[0])?))
    }
}

/// Where `t`'s cells and lines are. Edges are summed from the top left in order: the same
/// numbers on every machine.
pub fn layout(t: &Table) -> Layout {
    let edges = |sizes: &[f64]| {
        let mut v = Vec::with_capacity(sizes.len() + 1);
        v.push(0.0);
        for s in sizes {
            v.push(v[v.len() - 1] + s);
        }
        v
    };
    let (xs, ys) = (edges(&t.columns), edges(&t.rows));
    let [w, h] = [xs[xs.len() - 1], ys[ys.len() - 1]];
    let mut lines = Vec::with_capacity(xs.len() + ys.len());
    lines.extend(ys.iter().map(|&y| [[0.0, y], [w, y]]));
    lines.extend(xs.iter().map(|&x| [[x, 0.0], [x, h]]));
    Layout { xs, ys, lines }
}

/// A range as Excel puts it on the clipboard: cells by tabs, rows by line ends; a cell holding a
/// tab, a line end or a quote quoted, its quotes doubled.
pub fn to_tsv(block: &[Vec<&str>]) -> String {
    let cell = |s: &str| if s.contains(['\t', '\n', '\r', '"']) { format!("\"{}\"", s.replace('"', "\"\"")) } else { s.to_string() };
    block.iter().map(|row| row.iter().map(|s| cell(s)).collect::<Vec<_>>().join("\t")).collect::<Vec<_>>().join("\n")
}

/// A range from the clipboard, as Excel or LibreOffice copy it ([`to_tsv`]): rows of cells.
/// `\r\n` or `\n` ends a row; the line end after the last row is no row of its own.
pub fn parse_tsv(text: &str) -> Vec<Vec<String>> {
    let mut rows: Vec<Vec<String>> = vec![Vec::new()];
    let mut cell = String::new();
    let mut chars = text.chars().peekable();
    let mut start = true;
    while let Some(ch) = chars.next() {
        match ch {
            '"' if start => {
                // Quoted: up to the lone quote; a doubled one is a quote.
                while let Some(q) = chars.next() {
                    match q {
                        '"' if chars.peek() == Some(&'"') => {
                            chars.next();
                            cell.push('"');
                        }
                        '"' => break,
                        _ => cell.push(q),
                    }
                }
                start = false;
            }
            '\t' => {
                end_cell(&mut rows, &mut cell);
                start = true;
            }
            '\r' if chars.peek() == Some(&'\n') => {}
            '\n' => {
                end_cell(&mut rows, &mut cell);
                rows.push(Vec::new());
                start = true;
            }
            _ => {
                cell.push(ch);
                start = false;
            }
        }
    }
    if start && cell.is_empty() && rows.last().is_some_and(Vec::is_empty) {
        rows.pop();
    } else {
        end_cell(&mut rows, &mut cell);
    }
    rows
}

fn end_cell(rows: &mut [Vec<String>], cell: &mut String) {
    if let Some(r) = rows.last_mut() {
        r.push(std::mem::take(cell));
    }
}

#[cfg(test)]
mod tests;
