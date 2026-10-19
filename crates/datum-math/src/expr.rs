//! Expressions for parameters and dimensions (M3c, D57): numbers, parameter names, `+ - * /`,
//! unary minus, parentheses, `pi`, and the functions `sqrt abs min max sin cos tan` (angles in
//! degrees, as everywhere in a model).
//!
//! After `cadcraft-constraints` (`expr.rs`; MIT OR Apache-2.0, © ArtCraft Team and the CADCraft
//! contributors), extended with functions. Trigonometry goes through `libm`: values are the same
//! bit for bit on every platform (D34).

use std::collections::BTreeMap;
use std::fmt;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Longest expression accepted (characters).
const MAX_LEN: usize = 1024;
/// Deepest nesting (parentheses, unary minus, calls) and parameter reference chain.
const MAX_DEPTH: usize = 64;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExprError {
    Syntax(String),
    Unknown(String),
    Cycle(String),
    DivZero,
    TooDeep,
    NotFinite,
    /// A function outside its domain, such as the root of a negative number.
    Domain(String),
}

impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Syntax(m) => write!(f, "syntax error: {m}"),
            Self::Unknown(n) => write!(f, "unknown parameter `{n}`"),
            Self::Cycle(n) => write!(f, "parameter `{n}` depends on itself"),
            Self::DivZero => write!(f, "division by zero"),
            Self::TooDeep => write!(f, "expression too long or too deeply nested"),
            Self::NotFinite => write!(f, "the result is not a finite number"),
            Self::Domain(m) => write!(f, "{m}"),
        }
    }
}

impl std::error::Error for ExprError {}

/// A number in a model: given, or an expression of the model's parameters. In JSON a number or a
/// string: `12.5`, `"w / 2 - gap"`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum Num {
    Value(f64),
    Expr(String),
}

impl Num {
    /// The number, with parameter values from `lookup`.
    pub fn resolve(&self, lookup: &impl Fn(&str) -> Result<f64, ExprError>) -> Result<f64, ExprError> {
        match self {
            Self::Value(v) => Ok(*v),
            Self::Expr(e) => eval(e, lookup),
        }
    }

    /// The number once resolved; `None` for an expression.
    pub fn value(&self) -> Option<f64> {
        match self {
            Self::Value(v) => Some(*v),
            Self::Expr(_) => None,
        }
    }
}

impl fmt::Display for Num {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Value(v) => write!(f, "{v}"),
            Self::Expr(e) => write!(f, "{e}"),
        }
    }
}

impl From<f64> for Num {
    fn from(v: f64) -> Self {
        Self::Value(v)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Func {
    Sqrt,
    Abs,
    Min,
    Max,
    Sin,
    Cos,
    Tan,
}

const FUNCS: [(&str, Func); 7] =
    [("sqrt", Func::Sqrt), ("abs", Func::Abs), ("min", Func::Min), ("max", Func::Max), ("sin", Func::Sin), ("cos", Func::Cos), ("tan", Func::Tan)];

#[derive(Clone, Debug, PartialEq)]
enum Expr {
    Num(f64),
    Var(String),
    Neg(Box<Expr>),
    Bin(u8, Box<Expr>, Box<Expr>),
    Call(Func, Vec<Expr>),
}

/// A valid parameter name: a letter or `_`, then letters, digits or `_`; not `pi` or a function.
pub fn valid_name(s: &str) -> bool {
    let mut c = s.chars();
    matches!(c.next(), Some(ch) if ch.is_ascii_alphabetic() || ch == '_')
        && c.all(|ch| ch.is_ascii_alphanumeric() || ch == '_')
        && s.len() <= 64
        && !s.eq_ignore_ascii_case("pi")
        && !FUNCS.iter().any(|(n, _)| s.eq_ignore_ascii_case(n))
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
    depth: usize,
}

impl Parser<'_> {
    fn peek(&mut self) -> Option<u8> {
        while self.s.get(self.i).is_some_and(u8::is_ascii_whitespace) {
            self.i += 1;
        }
        self.s.get(self.i).copied()
    }

    fn nested<T>(&mut self, f: impl FnOnce(&mut Self) -> Result<T, ExprError>) -> Result<T, ExprError> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return Err(ExprError::TooDeep);
        }
        let r = f(self);
        self.depth -= 1;
        r
    }

    fn sum(&mut self) -> Result<Expr, ExprError> {
        self.nested(|p| {
            let mut e = p.product()?;
            while let Some(c @ (b'+' | b'-')) = p.peek() {
                p.i += 1;
                e = Expr::Bin(c, Box::new(e), Box::new(p.product()?));
            }
            Ok(e)
        })
    }

    fn product(&mut self) -> Result<Expr, ExprError> {
        let mut e = self.unary()?;
        while let Some(c @ (b'*' | b'/')) = self.peek() {
            self.i += 1;
            e = Expr::Bin(c, Box::new(e), Box::new(self.unary()?));
        }
        Ok(e)
    }

    fn unary(&mut self) -> Result<Expr, ExprError> {
        match self.peek() {
            Some(b'-') => {
                self.i += 1;
                self.nested(|p| Ok(Expr::Neg(Box::new(p.unary()?))))
            }
            Some(b'+') => {
                self.i += 1;
                self.nested(Self::unary)
            }
            _ => self.atom(),
        }
    }

    fn word(&mut self, ok: impl Fn(u8) -> bool) -> &str {
        let start = self.i;
        while self.s.get(self.i).is_some_and(|c| ok(*c)) {
            self.i += 1;
        }
        std::str::from_utf8(&self.s[start..self.i]).unwrap_or_default()
    }

    fn atom(&mut self) -> Result<Expr, ExprError> {
        match self.peek() {
            Some(b'(') => {
                self.i += 1;
                let e = self.sum()?;
                self.expect(b')')?;
                Ok(e)
            }
            Some(c) if c.is_ascii_digit() || c == b'.' => {
                let start = self.i;
                self.word(|c| c.is_ascii_digit() || c == b'.');
                // Exponent: 1e3, 2.5E-2.
                if self.s.get(self.i).is_some_and(|c| *c == b'e' || *c == b'E') {
                    let save = self.i;
                    self.i += 1;
                    if self.s.get(self.i).is_some_and(|c| *c == b'+' || *c == b'-') {
                        self.i += 1;
                    }
                    if self.word(|c| c.is_ascii_digit()).is_empty() {
                        self.i = save;
                    }
                }
                let t = std::str::from_utf8(&self.s[start..self.i]).unwrap_or_default();
                t.parse().map(Expr::Num).map_err(|_| ExprError::Syntax(format!("bad number `{t}`")))
            }
            Some(c) if c.is_ascii_alphabetic() || c == b'_' => {
                let name = self.word(|c| c.is_ascii_alphanumeric() || c == b'_').to_string();
                if name.eq_ignore_ascii_case("pi") {
                    return Ok(Expr::Num(std::f64::consts::PI));
                }
                if self.peek() != Some(b'(') {
                    return Ok(Expr::Var(name));
                }
                let f = FUNCS
                    .iter()
                    .find(|(n, _)| n.eq_ignore_ascii_case(&name))
                    .map(|f| f.1)
                    .ok_or_else(|| ExprError::Syntax(format!("unknown function `{name}`")))?;
                self.i += 1;
                let args = self.nested(|p| {
                    let mut args = vec![p.sum()?];
                    while p.peek() == Some(b',') {
                        p.i += 1;
                        args.push(p.sum()?);
                    }
                    p.expect(b')')?;
                    Ok(args)
                })?;
                let ok = match f {
                    Func::Min | Func::Max => !args.is_empty(),
                    _ => args.len() == 1,
                };
                if !ok {
                    return Err(ExprError::Syntax(format!(
                        "`{name}` takes {}",
                        if matches!(f, Func::Min | Func::Max) { "one or more arguments" } else { "one argument" }
                    )));
                }
                Ok(Expr::Call(f, args))
            }
            Some(c) => Err(ExprError::Syntax(format!("unexpected `{}`", c as char))),
            None => Err(ExprError::Syntax("unexpected end".into())),
        }
    }

    fn expect(&mut self, c: u8) -> Result<(), ExprError> {
        if self.peek() != Some(c) {
            return Err(ExprError::Syntax(format!("missing `{}`", c as char)));
        }
        self.i += 1;
        Ok(())
    }
}

fn parse(text: &str) -> Result<Expr, ExprError> {
    if text.len() > MAX_LEN {
        return Err(ExprError::TooDeep);
    }
    if text.trim().is_empty() {
        return Err(ExprError::Syntax("empty expression".into()));
    }
    let mut p = Parser { s: text.as_bytes(), i: 0, depth: 0 };
    let e = p.sum()?;
    if p.peek().is_some() {
        return Err(ExprError::Syntax(format!("unexpected `{}`", text[p.i..].trim())));
    }
    Ok(e)
}

fn value(e: &Expr, lookup: &dyn Fn(&str) -> Result<f64, ExprError>) -> Result<f64, ExprError> {
    let v = match e {
        Expr::Num(v) => *v,
        Expr::Var(n) => lookup(n)?,
        Expr::Neg(a) => -value(a, lookup)?,
        Expr::Bin(op, a, b) => {
            let (x, y) = (value(a, lookup)?, value(b, lookup)?);
            match op {
                b'+' => x + y,
                b'-' => x - y,
                b'*' => x * y,
                _ if y == 0.0 => return Err(ExprError::DivZero),
                _ => x / y,
            }
        }
        Expr::Call(f, args) => {
            let args = args.iter().map(|a| value(a, lookup)).collect::<Result<Vec<_>, _>>()?;
            let x = args[0];
            match f {
                Func::Sqrt if x < 0.0 => return Err(ExprError::Domain(format!("square root of {x}"))),
                Func::Sqrt => x.sqrt(),
                Func::Abs => x.abs(),
                Func::Min => args.iter().copied().fold(f64::INFINITY, f64::min),
                Func::Max => args.iter().copied().fold(f64::NEG_INFINITY, f64::max),
                Func::Sin => libm::sin(x.to_radians()),
                Func::Cos => libm::cos(x.to_radians()),
                Func::Tan => libm::tan(x.to_radians()),
            }
        }
    };
    if v.is_finite() { Ok(v) } else { Err(ExprError::NotFinite) }
}

/// Evaluates `text`, taking parameter values from `lookup`.
pub fn eval(text: &str, lookup: &impl Fn(&str) -> Result<f64, ExprError>) -> Result<f64, ExprError> {
    value(&parse(text)?, lookup)
}

/// Values of all parameters, each defined by an expression of the others: one result per name.
/// Cycles and unknown names are errors of the parameters that meet them.
pub fn parameter_values(defs: &BTreeMap<String, Num>) -> BTreeMap<String, Result<f64, ExprError>> {
    fn of(
        name: &str,
        defs: &BTreeMap<String, Num>,
        done: &std::cell::RefCell<BTreeMap<String, Result<f64, ExprError>>>,
        stack: &std::cell::RefCell<Vec<String>>,
    ) -> Result<f64, ExprError> {
        if let Some(r) = done.borrow().get(name) {
            return r.clone();
        }
        if stack.borrow().iter().any(|s| s == name) {
            return Err(ExprError::Cycle(name.to_string()));
        }
        if stack.borrow().len() >= MAX_DEPTH {
            return Err(ExprError::TooDeep);
        }
        let def = defs.get(name).ok_or_else(|| ExprError::Unknown(name.to_string()))?;
        stack.borrow_mut().push(name.to_string());
        let r = def.resolve(&|n| of(n, defs, done, stack));
        stack.borrow_mut().pop();
        done.borrow_mut().insert(name.to_string(), r.clone());
        r
    }
    let done = std::cell::RefCell::new(BTreeMap::new());
    let stack = std::cell::RefCell::new(Vec::new());
    for name in defs.keys() {
        let _ = of(name, defs, &done, &stack);
    }
    done.into_inner()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn constant(t: &str) -> Result<f64, ExprError> {
        eval(t, &|n| Err(ExprError::Unknown(n.to_string())))
    }

    #[test]
    fn arithmetic_and_functions() {
        assert_eq!(constant("1+2*3"), Ok(7.0));
        assert_eq!(constant("(1+2)*3"), Ok(9.0));
        assert_eq!(constant("-2*-3"), Ok(6.0));
        assert_eq!(constant("8-2-1"), Ok(5.0));
        assert_eq!(constant("1e2+.5"), Ok(100.5));
        assert_eq!(constant("sqrt(16) + abs(-2) + max(1, 7, 3) - min(4, 2)"), Ok(11.0));
        assert_eq!(constant("sin(30)"), Ok(libm::sin(30f64.to_radians())));
        assert!((constant("cos(60)").unwrap() - 0.5).abs() < 1e-15);
        assert!((constant("tan(45)").unwrap() - 1.0).abs() < 1e-15);
        assert!((constant("PI").unwrap() - std::f64::consts::PI).abs() == 0.0);
    }

    #[test]
    fn errors() {
        assert_eq!(constant("1/0"), Err(ExprError::DivZero));
        assert!(matches!(constant("sqrt(-1)"), Err(ExprError::Domain(_))));
        for bad in ["1+", "(1", "1 2", "", "1.2.3", "é", "foo(1)", "sqrt()", "sqrt(1, 2)", "min()"] {
            assert!(matches!(constant(bad), Err(ExprError::Syntax(_))), "{bad}");
        }
        assert!(matches!(constant("x"), Err(ExprError::Unknown(_))));
        assert!(matches!(constant("1e308*1e308"), Err(ExprError::NotFinite)));
        assert!(matches!(constant(&"(".repeat(500)), Err(ExprError::TooDeep)));
        assert!(matches!(constant(&"-".repeat(500)), Err(ExprError::TooDeep)));
    }

    #[test]
    fn parameters_and_cycles() {
        let defs: BTreeMap<String, Num> =
            [("w", "10"), ("h", "w/2"), ("a", "b+1"), ("b", "a+1"), ("c", "zz")].map(|(k, v)| (k.to_string(), Num::Expr(v.to_string()))).into();
        let v = parameter_values(&defs);
        assert_eq!(v["h"], Ok(5.0));
        assert_eq!(v["w"], Ok(10.0));
        assert!(matches!(v["a"], Err(ExprError::Cycle(_))));
        assert!(matches!(v["c"], Err(ExprError::Unknown(_))));
        assert!(valid_name("d1") && valid_name("_w") && !valid_name("1d") && !valid_name("a-b") && !valid_name("pi") && !valid_name("sqrt"));
    }

    #[test]
    fn numbers_in_json() {
        let n: Vec<Num> = serde_json::from_str(r#"[2.5, "w / 2"]"#).unwrap_or_default();
        assert_eq!(n, vec![Num::Value(2.5), Num::Expr("w / 2".into())]);
    }
}
