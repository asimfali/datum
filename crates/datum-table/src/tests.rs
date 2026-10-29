use super::*;

fn grid() -> Table {
    let mut t = Table::new(vec![20.0, 30.0, 15.0], 3);
    t.set(0, 0, "name");
    t.set(1, 1, "w / 2");
    t.set(2, 2, "a\tb");
    t
}

#[test]
fn cells_are_set_and_read_inside_only() {
    let mut t = grid();
    assert_eq!((t.cell(0, 0), t.cell(1, 1), t.cell(0, 1)), ("name", "w / 2", ""));
    t.set(3, 0, "out");
    t.set(0, 3, "out");
    assert_eq!((t.cell(3, 0), t.cell(0, 3)), ("", ""));
    assert_eq!(t.rows(), [ROW; 3]);
}

#[test]
fn the_layout_places_cells_and_lines() {
    let mut t = grid();
    t.set_width(1, 40.0);
    let l = layout(&t);
    assert_eq!(l.xs, [0.0, 20.0, 60.0, 75.0]);
    assert_eq!(l.ys, [0.0, 8.0, 16.0, 24.0]);
    assert_eq!(l.size(), [75.0, 24.0]);
    assert_eq!(l.cell(1, 1), Some(Rect { x: 20.0, y: 8.0, w: 40.0, h: 8.0 }));
    assert_eq!(l.cell(3, 0), None);
    assert_eq!(l.lines.len(), 4 + 4);
    assert!(l.lines.contains(&[[0.0, 24.0], [75.0, 24.0]]) && l.lines.contains(&[[60.0, 0.0], [60.0, 24.0]]));
}

#[test]
fn a_point_finds_its_cell() {
    let l = layout(&grid());
    assert_eq!(l.at([25.0, 9.0]), Some((1, 1)));
    assert_eq!(l.at([0.0, 0.0]), Some((0, 0)));
    assert_eq!(l.at([20.0, 8.0]), Some((1, 1)), "an edge belongs to the cell after it");
    assert_eq!(l.at([75.0, 1.0]), None);
    assert_eq!(l.at([-1.0, 1.0]), None);
    assert_eq!(l.at([1.0, 30.0]), None);
}

#[test]
fn a_range_goes_to_excel_and_back() {
    let t = grid();
    let text = t.copy((2, 2), (1, 1));
    assert_eq!(text, "w / 2\t\n\t\"a\tb\"");
    assert_eq!(parse_tsv(&text), [vec!["w / 2", ""], vec!["", "a\tb"]]);
}

#[test]
fn excel_s_clipboard_is_read() {
    // Excel ends every row, the last too, with \r\n; a cell with a line end is quoted.
    assert_eq!(parse_tsv("a\tb\r\nc\td\r\n"), [vec!["a", "b"], vec!["c", "d"]]);
    assert_eq!(parse_tsv("\"two\nlines\"\t\"say \"\"hi\"\"\"\n"), [vec!["two\nlines", "say \"hi\""]]);
    assert_eq!(parse_tsv("a\t\t"), [vec!["a", "", ""]]);
    assert_eq!(parse_tsv("12,5"), [vec!["12,5"]]);
    assert_eq!(parse_tsv("a\n\nb"), [vec!["a"], vec![""], vec!["b"]]);
    assert!(parse_tsv("").is_empty());
    assert_eq!(to_tsv(&[vec!["x\"y"]]), "\"x\"\"y\"");
}
