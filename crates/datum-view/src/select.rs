//! How a click or a frame changes a selection, as in Inventor; one rule for everything the window
//! selects: a sketch's elements, a pattern's copies.

/// Whether the keys held add to the selection: Shift or Ctrl (Cmd on a Mac).
pub fn adding(m: &egui::Modifiers) -> bool {
    m.shift || m.ctrl || m.command
}

/// A click on `picks` (what one click takes: an element, or all a group holds): alone, they are
/// the selection; adding, they join it, or leave it if it holds them all already.
pub fn click<T: PartialEq>(selected: &mut Vec<T>, picks: Vec<T>, adding: bool) {
    if !adding {
        *selected = picks;
    } else if picks.iter().all(|p| selected.contains(p)) {
        selected.retain(|q| !picks.contains(q));
    } else {
        join(selected, picks);
    }
}

/// A frame around `picks`: alone, they are the selection; adding, they join it.
pub fn frame<T: PartialEq>(selected: &mut Vec<T>, picks: Vec<T>, adding: bool) {
    if !adding {
        selected.clear();
    }
    join(selected, picks);
}

fn join<T: PartialEq>(selected: &mut Vec<T>, picks: Vec<T>) {
    for p in picks {
        if !selected.contains(&p) {
            selected.push(p);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clicks_replace_add_and_take_away() {
        let mut s = vec![1];
        click(&mut s, vec![2], false);
        assert_eq!(s, [2]);
        click(&mut s, vec![3, 4], true);
        assert_eq!(s, [2, 3, 4]);
        click(&mut s, vec![3], true);
        assert_eq!(s, [2, 4], "a click adding takes away what is held");
        click(&mut s, vec![4, 5], true);
        assert_eq!(s, [2, 4, 5], "a group held in part joins whole");
        click(&mut s, vec![], false);
        assert!(s.is_empty(), "a click on nothing clears");
    }

    #[test]
    fn frames_replace_or_add() {
        let mut s = vec![1, 2];
        frame(&mut s, vec![2, 3], true);
        assert_eq!(s, [1, 2, 3]);
        frame(&mut s, vec![4], false);
        assert_eq!(s, [4]);
    }

    #[test]
    fn shift_or_ctrl_adds() {
        assert!(adding(&egui::Modifiers::SHIFT));
        assert!(adding(&egui::Modifiers::CTRL));
        assert!(adding(&egui::Modifiers::COMMAND));
        assert!(!adding(&egui::Modifiers::ALT));
        assert!(!adding(&egui::Modifiers::NONE));
    }
}
