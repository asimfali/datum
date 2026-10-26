//! Datum feature graph: memoised, incremental, parallel evaluation (ARCHITECTURE §5).
//!
//! A node's cache key is the hash of its feature ID, its parameters and the content hashes of its
//! inputs. Every evaluation walks the whole graph and reuses cached bodies by key, so:
//! - an edit recomputes only the nodes whose key changed (incremental);
//! - a recomputed node that yields the same body leaves the keys below it unchanged (early cutoff);
//! - going back to old parameters hits the cache (undo for free).
//!
//! Nodes run on rayon as soon as their inputs are done. Results do not depend on thread count.

use std::{
    collections::{BTreeMap, HashMap},
    fmt,
    ops::RangeInclusive,
    sync::{
        Arc, Mutex, OnceLock,
        atomic::{AtomicUsize, Ordering},
    },
};

use datum_brep::{BrepError, TopoId};
use datum_math::expr::{ExprError, Num, parameter_values};
pub use datum_ops::{Op, Value};

pub type NodeIx = u32;
/// A feature's value, shared between evaluations through the cache.
pub type ValueRef = Arc<Value>;

#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    pub name: String,
    /// Derived from the name, so it survives edits and reordering.
    pub id: TopoId,
    pub op: Op,
    pub inputs: Vec<NodeIx>,
    /// Off (0, or a formula coming to 0): the feature passes its first input on unchanged, as if it
    /// were not there (D159). A variant turns a parent's feature off instead of removing it.
    pub enabled: Num,
}

/// The model as a DAG. Inputs always precede the node, so the graph is acyclic by construction.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Graph {
    nodes: Vec<Node>,
    by_name: BTreeMap<String, NodeIx>,
    /// Model parameters by name: numbers or expressions of each other (D57).
    parameters: BTreeMap<String, Num>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GraphError {
    DuplicateName(String),
    /// Not a valid parameter name, or one some feature still uses.
    Parameter(String),
    UnknownInput {
        node: String,
        input: NodeIx,
    },
    Arity {
        node: String,
        expected: RangeInclusive<usize>,
        got: usize,
    },
}

impl fmt::Display for GraphError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateName(n) => write!(f, "a feature named `{n}` already exists"),
            Self::Parameter(m) => write!(f, "{m}"),
            Self::UnknownInput { node, input } => write!(f, "`{node}`: input #{input} is not an earlier feature"),
            Self::Arity { node, expected, got } => write!(f, "`{node}` takes {} input(s), got {got}", datum_ops::describe_arity(expected.clone())),
        }
    }
}

impl std::error::Error for GraphError {}

impl Graph {
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub fn find(&self, name: &str) -> Option<NodeIx> {
        self.by_name.get(name).copied()
    }

    pub fn add(&mut self, name: &str, op: Op, inputs: &[NodeIx]) -> Result<NodeIx, GraphError> {
        if self.by_name.contains_key(name) {
            return Err(GraphError::DuplicateName(name.into()));
        }
        if let Some(&input) = inputs.iter().find(|&&i| i as usize >= self.nodes.len()) {
            return Err(GraphError::UnknownInput { node: name.into(), input });
        }
        check_arity(name, &op, inputs.len())?;
        let ix = self.nodes.len() as NodeIx;
        self.nodes.push(Node {
            name: name.into(),
            id: TopoId::derive("feature").tag(name).finish(),
            op,
            inputs: inputs.to_vec(),
            enabled: Num::Value(1.0),
        });
        self.by_name.insert(name.into(), ix);
        Ok(ix)
    }

    pub fn parameters(&self) -> &BTreeMap<String, Num> {
        &self.parameters
    }

    /// Sets parameter `name` to a number or an expression of other parameters.
    pub fn set_parameter(&mut self, name: &str, value: Num) -> Result<(), GraphError> {
        if !datum_math::expr::valid_name(name) {
            return Err(GraphError::Parameter(format!("`{name}` is not a valid parameter name: a letter or `_`, then letters, digits or `_`")));
        }
        self.parameters.insert(name.into(), value);
        Ok(())
    }

    pub fn remove_parameter(&mut self, name: &str) -> Result<(), GraphError> {
        self.parameters.remove(name).map(|_| ()).ok_or_else(|| GraphError::Parameter(format!("no parameter named `{name}`")))
    }

    /// Turns a node on or off: a number (0: off) or a formula of the parameters.
    pub fn set_enabled(&mut self, ix: NodeIx, enabled: Num) {
        self.nodes[ix as usize].enabled = enabled;
    }

    /// Replaces a node's operation (a parameter edit) keeping its inputs.
    pub fn set_op(&mut self, ix: NodeIx, op: Op) -> Result<(), GraphError> {
        let node = &mut self.nodes[ix as usize];
        check_arity(&node.name, &op, node.inputs.len())?;
        node.op = op;
        Ok(())
    }
}

fn check_arity(name: &str, op: &Op, got: usize) -> Result<(), GraphError> {
    if op.arity().contains(&got) { Ok(()) } else { Err(GraphError::Arity { node: name.into(), expected: op.arity(), got }) }
}

#[derive(Clone, Debug, PartialEq)]
pub enum EvalError {
    Op(BrepError),
    /// A number of the operation could not be evaluated.
    Parameter(String),
    /// The named input failed, so this node could not run.
    Input(String),
    /// Turned off, with no input to pass on.
    Disabled,
}

impl fmt::Display for EvalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Op(e) => e.fmt(f),
            Self::Parameter(m) => write!(f, "parameter: {m}"),
            Self::Input(n) => write!(f, "input `{n}` failed"),
            Self::Disabled => write!(f, "turned off, and it has no input to pass on"),
        }
    }
}

impl std::error::Error for EvalError {}

/// The outcome of one evaluation: a result per node, in node order.
#[derive(Debug)]
pub struct Evaluation {
    pub results: Vec<Result<ValueRef, EvalError>>,
    /// Nodes computed in this evaluation.
    pub computed: usize,
    /// Nodes taken from the cache.
    pub reused: usize,
}

impl Evaluation {
    /// Values of the features nothing else uses: what the model ends in. Failed ones are left out.
    /// An unfold uses nothing up: its blank is made for export, the sheet stays what the model
    /// ends in. Nor does a sketch or a work plane on a body's face (D150): they only refer to it,
    /// the body stays.
    pub fn final_values(&self, graph: &Graph) -> Vec<&Value> {
        self.finals(graph).into_iter().map(|i| self.results[i as usize].as_ref().map(|v| v.as_ref())).filter_map(Result::ok).collect()
    }

    /// The features [`Self::final_values`] are of, failed ones too.
    pub fn finals(&self, graph: &Graph) -> Vec<NodeIx> {
        let mut used = vec![false; graph.nodes.len()];
        for n in graph.nodes.iter().filter(|n| !matches!(n.op, Op::Unfold { .. } | Op::Sketch { .. } | Op::Plane { .. })) {
            for &i in &n.inputs {
                used[i as usize] = true;
            }
        }
        (0..graph.nodes.len() as NodeIx).filter(|&i| !used[i as usize]).collect()
    }
}

/// Evaluates graphs, keeping computed bodies by key across evaluations. The cache is unbounded
/// for now; eviction comes when memory is measured (ARCHITECTURE §5.1).
#[derive(Default)]
pub struct Evaluator {
    cache: Mutex<HashMap<[u8; 32], ValueRef>>,
}

impl Evaluator {
    /// Evaluates every node in the current rayon pool.
    pub fn evaluate(&self, graph: &Graph) -> Evaluation {
        let n = graph.nodes.len();
        let mut dependents = vec![Vec::new(); n];
        for (i, node) in graph.nodes.iter().enumerate() {
            for &j in &node.inputs {
                dependents[j as usize].push(i as NodeIx);
            }
        }
        let values = parameter_values(&graph.parameters);
        let run = Run {
            graph,
            values,
            evaluator: self,
            results: (0..n).map(|_| OnceLock::new()).collect(),
            pending: graph.nodes.iter().map(|node| AtomicUsize::new(node.inputs.len())).collect(),
            dependents,
            computed: AtomicUsize::new(0),
            reused: AtomicUsize::new(0),
        };
        rayon::scope(|s| {
            for (i, node) in graph.nodes.iter().enumerate() {
                if node.inputs.is_empty() {
                    let run = &run;
                    s.spawn(move |s| run.step(s, i));
                }
            }
        });
        let Run { results, computed, reused, .. } = run;
        // Every node runs exactly once: its pending count reaches zero when its last input is done.
        let results = results.into_iter().map(|r| r.into_inner().unwrap_or_else(|| Err(EvalError::Input("<unreached>".into())))).collect();
        Evaluation { results, computed: computed.into_inner(), reused: reused.into_inner() }
    }

    fn cache(&self) -> std::sync::MutexGuard<'_, HashMap<[u8; 32], ValueRef>> {
        // A panic while holding the lock cannot leave the map half-updated: insert is atomic.
        self.cache.lock().unwrap_or_else(|e| e.into_inner())
    }
}

struct Run<'a> {
    graph: &'a Graph,
    /// Parameter values, computed once per evaluation.
    values: BTreeMap<String, Result<f64, ExprError>>,
    evaluator: &'a Evaluator,
    results: Vec<OnceLock<Result<ValueRef, EvalError>>>,
    pending: Vec<AtomicUsize>,
    dependents: Vec<Vec<NodeIx>>,
    computed: AtomicUsize,
    reused: AtomicUsize,
}

impl<'a> Run<'a> {
    /// Runs node `i`, then the nodes it made ready: all but one are spawned, the last one runs on
    /// this thread, so a chain stays on one core instead of hopping between threads.
    fn step<'s>(&'s self, s: &rayon::Scope<'s>, mut i: usize) {
        loop {
            let _ = self.results[i].set(self.eval_node(i));
            let mut next = None;
            for &d in &self.dependents[i] {
                if self.pending[d as usize].fetch_sub(1, Ordering::AcqRel) == 1
                    && let Some(prev) = next.replace(d as usize)
                {
                    s.spawn(move |s| self.step(s, prev));
                }
            }
            match next {
                Some(d) => i = d,
                None => return,
            }
        }
    }

    fn eval_node(&self, i: usize) -> Result<ValueRef, EvalError> {
        let node = &self.graph.nodes[i];
        let mut inputs = Vec::with_capacity(node.inputs.len());
        for &j in &node.inputs {
            match self.results[j as usize].get() {
                Some(Ok(b)) => inputs.push(b.clone()),
                _ => return Err(EvalError::Input(self.graph.nodes[j as usize].name.clone())),
            }
        }
        // Numbers are evaluated before the key: a parameter edit recomputes exactly the features
        // whose numbers change.
        let lookup = |n: &str| self.values.get(n).cloned().unwrap_or_else(|| Err(ExprError::Unknown(n.into())));
        // Off: the first input as it came, its own key and all (D159).
        if node.enabled.resolve(&lookup).map_err(|e| EvalError::Parameter(format!("enabled: {e}")))? == 0.0 {
            return inputs.into_iter().next().ok_or(EvalError::Disabled);
        }
        let op = node.op.resolve(&lookup).map_err(EvalError::Parameter)?;
        let mut h = blake3::Hasher::new();
        h.update(&node.id.0.to_le_bytes());
        op.hash_params(&mut h);
        for b in &inputs {
            h.update(&b.hash());
        }
        let key = *h.finalize().as_bytes();
        if let Some(b) = self.evaluator.cache().get(&key).cloned() {
            self.reused.fetch_add(1, Ordering::Relaxed);
            return Ok(b);
        }
        let refs: Vec<&Value> = inputs.iter().map(|b| b.as_ref()).collect();
        let body = Arc::new(op.eval(node.id, &refs).map_err(EvalError::Op)?);
        self.computed.fetch_add(1, Ordering::Relaxed);
        self.evaluator.cache().insert(key, body.clone());
        Ok(body)
    }
}
