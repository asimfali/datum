//! What a cell holds (D178, D180): pieces in a line, text and fractions (a text over a bar and
//! one under it, as a material is written by GOST: `Лист` and the gauge over the grade). A cell
//! keeps its text form, `Лист [Б-ПН-{t} ГОСТ 19903-2015 | 08пс ГОСТ 16523-97]`, which goes to
//! Excel and comes from an agent as it is; the pieces are read from it.

use crate::Rect;

/// A piece of a cell's line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Piece {
    Text(String),
    Fraction { over: String, under: String },
}

/// The pieces of a cell's text: `[over | under]` is a fraction, the rest is text. A bracket
/// with no bar before its closing one, or with a bracket inside, is text.
pub fn pieces(text: &str) -> Vec<Piece> {
    let mut out = Vec::new();
    let mut rest = text;
    let mut plain = String::new();
    while let Some(open) = rest.find('[') {
        let inner = &rest[open + 1..];
        let fraction = inner
            .find(']')
            .map(|close| &inner[..close])
            .filter(|body| !body.contains('['))
            .and_then(|body| body.split_once('|').filter(|(_, under)| !under.contains('|')).map(|f| (f, body.len())));
        match fraction {
            Some(((over, under), len)) => {
                plain.push_str(&rest[..open]);
                if !plain.is_empty() {
                    out.push(Piece::Text(std::mem::take(&mut plain)));
                }
                out.push(Piece::Fraction { over: over.trim().to_string(), under: under.trim().to_string() });
                rest = &inner[len + 1..];
            }
            None => {
                plain.push_str(&rest[..=open]);
                rest = inner;
            }
        }
    }
    plain.push_str(rest);
    if !plain.is_empty() {
        out.push(Piece::Text(plain));
    }
    out
}

/// The text form of `pieces`, read back by [`pieces`]. A fraction's line loses its `[`, `]` and
/// `|`: they would end the fraction.
pub fn text_of(pieces: &[Piece]) -> String {
    let line = |t: &str| t.replace(['[', ']', '|'], "");
    pieces
        .iter()
        .map(|p| match p {
            Piece::Text(t) => t.clone(),
            Piece::Fraction { over, under } => format!("[{} | {}]", line(over), line(under)),
        })
        .collect()
}

/// The height of a fraction's lines to the text's, as a stacked fraction in AutoCAD: both fit an
/// ESKD row with the text's height.
pub const STACK: f64 = 0.7;

/// The gap from the left edge of a cell to its text, in millimetres.
pub const PAD: f64 = 1.0;

/// Which part of a piece a box holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    Text,
    Over,
    Under,
}

/// A box of a cell's line: piece `piece`'s part `slot`, its text drawn centred in `rect`. An
/// empty part of a fraction has a box too, to click into.
#[derive(Clone, Debug, PartialEq)]
pub struct Boxed {
    pub piece: usize,
    pub slot: Slot,
    pub rect: Rect,
}

/// Where a cell's line goes: the boxes of its text, and the bars of its fractions.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Line {
    pub boxes: Vec<Boxed>,
    pub bars: Vec<[[f64; 2]; 2]>,
}

impl Line {
    /// The box under point `p`.
    pub fn at(&self, p: [f64; 2]) -> Option<&Boxed> {
        self.boxes.iter().find(|b| (b.rect.x..=b.rect.x + b.rect.w).contains(&p[0]) && (b.rect.y..=b.rect.y + b.rect.h).contains(&p[1]))
    }
}

/// Where `pieces` go in `cell`: from its left edge after [`PAD`], centred on its middle line,
/// text `height` high; a fraction's lines are [`STACK`] of it, over and under a bar as wide as
/// the wider of them and no narrower than the text is high. `width(text, height)` is how wide
/// `text` is at `height`: the font's business, not the table's.
pub fn place(pieces: &[Piece], cell: Rect, height: f64, width: &dyn Fn(&str, f64) -> f64) -> Line {
    let mid = cell.y + cell.h / 2.0;
    let small = height * STACK;
    let gap = small / 4.0;
    let mut x = cell.x + PAD;
    let mut line = Line::default();
    for (piece, p) in pieces.iter().enumerate() {
        match p {
            Piece::Text(t) => {
                let w = width(t, height);
                line.boxes.push(Boxed { piece, slot: Slot::Text, rect: Rect { x, y: mid - height / 2.0, w, h: height } });
                x += w;
            }
            Piece::Fraction { over, under } => {
                let w = width(over, small).max(width(under, small)).max(height);
                line.boxes.push(Boxed { piece, slot: Slot::Over, rect: Rect { x, y: mid - gap - small, w, h: small } });
                line.boxes.push(Boxed { piece, slot: Slot::Under, rect: Rect { x, y: mid + gap, w, h: small } });
                line.bars.push([[x, mid], [x + w, mid]]);
                x += w;
            }
        }
    }
    line
}
