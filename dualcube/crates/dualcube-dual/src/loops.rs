use dualcube_types::prelude::*;
use serde::{Deserialize, Serialize};
use slotmap::SlotMap;
use std::time::Instant;

slotmap::new_key_type! {
    pub struct LoopID;
}

#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct NodeCopy {
    pub id: [EdgeID; 2],
    pub t: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Loop {
    pub edges: Vec<EdgeID>,
    pub direction: Direction,
}

#[must_use]
pub fn wrap_pairs<T: Copy>(sequence: &[T]) -> Vec<(T, T)> {
    sequence
        .iter()
        .cycle()
        .copied()
        .take(sequence.len() + 1)
        .tuple_windows()
        .collect()
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

#[derive(Clone, Debug)]
pub struct LoopState {
    pub loops: SlotMap<LoopID, Loop>,
    pub occupied: ids::SecMap<EDGE, INPUT, Vec<LoopID>>,
    pub last_loop: Option<LoopID>,
}

impl Default for LoopState {
    fn default() -> Self {
        Self::new()
    }
}

impl LoopState {
    #[must_use]
    pub fn new() -> Self {
        Self {
            loops: SlotMap::with_key(),
            occupied: ids::SecMap::new(),
            last_loop: None,
        }
    }

    #[must_use]
    pub fn from_loops(loops: SlotMap<LoopID, Loop>) -> Self {
        Self {
            occupied: Loop::occupied(&loops),
            loops,
            last_loop: None,
        }
    }

    pub fn as_mut(&mut self) -> LoopStateMut<'_> {
        LoopStateMut {
            loops: &mut self.loops,
            occupied: &mut self.occupied,
            last_loop: &mut self.last_loop,
        }
    }
}

pub struct LoopStateMut<'a> {
    pub loops: &'a mut SlotMap<LoopID, Loop>,
    pub occupied: &'a mut ids::SecMap<EDGE, INPUT, Vec<LoopID>>,
    pub last_loop: &'a mut Option<LoopID>,
}

impl LoopStateMut<'_> {
    pub fn recompute_occupied(&mut self) {
        let timer = Instant::now();
        let loop_count = self.loops.len();
        let mut edge_refs = 0usize;

        self.occupied.clear();
        for (loop_id, loop_) in self.loops.iter() {
            edge_refs += loop_.edges.len();
            for &e in &loop_.edges {
                if !self.occupied.contains_key(&e) {
                    self.occupied.insert(&e, vec![]);
                }
                self.occupied.get_mut(&e).unwrap().push(loop_id);
            }
        }

        info!(
            "b_loops::recompute_occupied: loops={} edge_refs={} occupied_edges={} elapsed_ms={:.3}",
            loop_count,
            edge_refs,
            self.occupied.iter().count(),
            elapsed_ms(timer)
        );
    }

    pub fn del_loop(&mut self, loop_id: LoopID) {
        let timer = Instant::now();
        let edge_count = self.loops[loop_id].edges.len();

        for &e in &self.loops[loop_id].edges.clone() {
            if let Some(v) = self.occupied.get_mut(&e) {
                v.retain(|&l| l != loop_id);
                if v.is_empty() {
                    self.occupied.remove(&e);
                }
            }
        }

        self.loops.remove(loop_id);

        info!(
            "b_loops::del_loop: loop={loop_id:?} edges={} remaining_loops={} elapsed_ms={:.3}",
            edge_count,
            self.loops.len(),
            elapsed_ms(timer)
        );
    }

    pub fn add_loop(&mut self, l: Loop) -> LoopID {
        let timer = Instant::now();
        let edge_count = l.edges.len();
        let direction = l.direction;
        let loop_id = self.loops.insert(l);

        for e in self.loops[loop_id].edges.clone() {
            if !self.occupied.contains_key(&e) {
                self.occupied.insert(&e, vec![]);
            }
            self.occupied.get_mut(&e).unwrap().push(loop_id);
        }

        *self.last_loop = Some(loop_id);

        info!(
            "b_loops::add_loop: loop={loop_id:?} direction={direction:?} edges={} total_loops={} elapsed_ms={:.3}",
            edge_count,
            self.loops.len(),
            elapsed_ms(timer)
        );

        loop_id
    }
}

impl Loop {
    #[must_use]
    pub fn contains_pair(&self, needle: (EdgeID, EdgeID)) -> bool {
        wrap_pairs(&self.edges)
            .into_iter()
            .any(|(a, b)| a == needle.0 && b == needle.1)
    }

    #[must_use]
    fn find_edge(&self, needle: EdgeID) -> usize {
        self.edges.iter().position(|&e| e == needle).unwrap()
    }

    #[must_use]
    pub fn between(&self, start: EdgeID, end: EdgeID) -> Vec<EdgeID> {
        let start_pos = self.find_edge(start);
        let end_pos = self.find_edge(end);

        let mut seq = vec![];
        if start_pos < end_pos {
            seq.extend(self.edges[start_pos..=end_pos].iter());
        } else {
            seq.extend(self.edges[start_pos..].iter());
            seq.extend(self.edges[..=end_pos].iter());
        }
        seq.into_iter().copied().collect()
    }

    #[must_use]
    pub fn occupied(loops: &SlotMap<LoopID, Self>) -> ids::SecMap<EDGE, INPUT, Vec<LoopID>> {
        let mut occupied = ids::SecMap::new();
        for loop_id in loops.keys() {
            for &edge in &loops[loop_id].edges {
                if !occupied.contains_key(&edge) {
                    occupied.insert(&edge, vec![]);
                }
                occupied.get_mut(&edge).unwrap().push(loop_id);
            }
        }
        occupied
    }
}
