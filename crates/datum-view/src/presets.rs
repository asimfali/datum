//! The fillet presets' editor (D284): the radiuses and the chamfer sizes of the sketch's fillet
//! tool (D283) typed in a window of their own; OK applies them at once and writes them into
//! `settings.json`, the rest of the file kept. A chamfer's entry is a size, or a pair of two
//! (the second waits for the geometry, the first one works).

/// The editor's rows as typed, and what parsing said last.
/// What the rows parse into: the radiuses, and the chamfers' sizes (the second waits).
pub type Presets = (Vec<f64>, Vec<(f64, Option<f64>)>);

#[derive(Default, Clone, Debug)]
pub struct Editor {
    pub fillet: Vec<String>,
    /// Each row: the first size, and the second (empty: none).
    pub chamfer: Vec<(String, String)>,
    pub error: Option<String>,
}

impl Editor {
    /// The rows of the presets now in force.
    pub fn of(fillet: &[f64], chamfer: &[(f64, Option<f64>)]) -> Self {
        Self {
            fillet: fillet.iter().map(|r| crate::draw::short(*r).to_string()).collect(),
            chamfer: chamfer
                .iter()
                .map(|(a, b)| (crate::draw::short(*a).to_string(), b.map_or_else(String::new, |b| crate::draw::short(b).to_string())))
                .collect(),
            error: None,
        }
    }

    /// What the rows say: the presets, or why not (a row empty goes, one that is no positive
    /// number is said). Sorted, without repeats.
    pub fn parse(&self) -> Result<Presets, String> {
        let one = |t: &str, what: &str| -> Result<f64, String> {
            let v = t.trim().parse::<f64>().map_err(|_| format!("{what}: `{t}` is no number"))?;
            if v.is_finite() && v > 0.0 { Ok((v * 1e6).round() / 1e6) } else { Err(format!("{what}: `{t}` is no positive number")) }
        };
        let mut fillet = Vec::new();
        for (i, row) in self.fillet.iter().enumerate() {
            if row.trim().is_empty() {
                continue;
            }
            fillet.push(one(row, &format!("the {i}th radius"))?);
        }
        fillet.sort_by(f64::total_cmp);
        fillet.dedup();
        let mut chamfer = Vec::new();
        for (i, (a, b)) in self.chamfer.iter().enumerate() {
            if a.trim().is_empty() && b.trim().is_empty() {
                continue;
            }
            let a = one(a, &format!("the {i}th chamfer's first size"))?;
            let b = if b.trim().is_empty() { None } else { Some(one(b, &format!("the {i}th chamfer's second size"))?) };
            chamfer.push((a, b));
        }
        chamfer.sort_by(|x, y| {
            x.0.total_cmp(&y.0).then_with(|| match (x.1, y.1) {
                (None, None) => std::cmp::Ordering::Equal,
                (None, Some(_)) => std::cmp::Ordering::Less,
                (Some(_), None) => std::cmp::Ordering::Greater,
                (Some(x), Some(y)) => x.total_cmp(&y),
            })
        });
        chamfer.dedup_by(|x, y| x.0.total_cmp(&y.0) == std::cmp::Ordering::Equal && x.1 == y.1);
        Ok((fillet, chamfer))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The rows parse: empty rows go, what is no positive number is said, sorted, without repeats.
    #[test]
    fn the_rows_parse_into_presets() {
        let e = Editor {
            fillet: vec!["5".into(), "2".into(), String::new(), "5".into(), "x".into()],
            chamfer: vec![("2".into(), String::new()), ("2".into(), "4".into()), (String::new(), String::new()), ("0".into(), String::new())],
            error: None,
        };
        let said = e.parse().unwrap_err();
        assert!(said.contains("the 4th radius"), "{said}");
        let e = Editor {
            fillet: vec!["5".into(), "2".into(), String::new(), "5".into()],
            chamfer: vec![("2".into(), String::new()), ("2".into(), "4".into()), (String::new(), String::new())],
            error: None,
        };
        let (fillet, chamfer) = e.parse().unwrap();
        assert_eq!(fillet, [2.0, 5.0], "sorted, without repeats, empty gone");
        assert_eq!(chamfer, [(2.0, None), (2.0, Some(4.0))], "the empty row gone");
    }

    /// Of the presets now in force the rows come, and the JSON keeps the pair as a pair.
    #[test]
    fn the_rows_of_the_presets_and_their_json() {
        let e = Editor::of(&[2.0, 5.0], &[(2.0, None), (2.0, Some(4.0))]);
        assert_eq!(e.fillet, ["2", "5"]);
        assert_eq!(e.chamfer, [("2".into(), String::new()), ("2".into(), "4".into())]);
    }
}
