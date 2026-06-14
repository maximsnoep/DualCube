//! Dual loops and their computation.
//!
//! A [`Loop`] is a cyclic sequence of mesh half-edges, labeled with a principal
//! direction. A collection of loops forms the basis of the dual structure (see
//! [`crate::dual`]). This module owns the [`Loop`] type as well as the
//! [`Solution`] methods that bookkeep loops (which edges are occupied by which
//! loops) and that trace new loops through the flow graphs.

use crate::dual::PropertyViolationError;
use crate::prelude::*;
use crate::solutions::Solution;
use grapff::Grapff;
use itertools::Itertools;
use mehsh::prelude::*;
use ordered_float::OrderedFloat;
use orx_parallel::*;
use rand::{rng, seq::IteratorRandom};
use serde::{Deserialize, Serialize};
use slotmap::SlotMap;
use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::Arc;

slotmap::new_key_type! {
    pub struct LoopID;
}

#[derive(Default, Debug, Clone, Copy, Eq, PartialEq, Hash)]
pub struct NodeCopy {
    pub id: [EdgeID; 2],
    pub t: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DijkstraEntry {
    cost: Reverse<OrderedFloat<f64>>,
    insertion_order: Reverse<usize>,
    node: EdgeID,
}

impl Ord for DijkstraEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.cost
            .cmp(&other.cost)
            .then_with(|| self.insertion_order.cmp(&other.insertion_order))
    }
}

impl PartialOrd for DijkstraEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// A loop forms the basis of the dual structure.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Loop {
    /// A loop is defined by a sequence of half-edges.
    pub edges: Vec<EdgeID>,
    /// The direction (labeling) associated with the loop.
    pub direction: PrincipalDirection,
}

/// Returns the consecutive (wrapping) pairs of a sequence: `[a, b, c]` becomes
/// `[(a, b), (b, c), (c, a)]`.
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
            // if start_pos < end_pos, we return [start...end]
            seq.extend(self.edges[start_pos..=end_pos].iter());
        } else {
            // if start_pos > end_pos, we return [start...MAX] + [0...end]
            seq.extend(self.edges[start_pos..].iter());
            seq.extend(self.edges[..=end_pos].iter());
        }
        seq
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

impl Solution {
    pub fn recompute_occupied(&mut self) {
        *self
            .available_flow_graphs
            .write()
            .expect("available flow graph cache lock poisoned") = None;
        self.occupied.clear();
        for (loop_id, loop_) in &self.loops {
            for &e in &loop_.edges {
                if !self.occupied.contains_key(&e) {
                    self.occupied.insert(&e, vec![]);
                }
                self.occupied.get_mut(&e).unwrap().push(loop_id);
            }
        }
    }

    pub fn del_loop(&mut self, loop_id: LoopID) {
        *self
            .available_flow_graphs
            .write()
            .expect("available flow graph cache lock poisoned") = None;
        for &e in &self.loops[loop_id].edges.clone() {
            if let Some(v) = self.occupied.get_mut(&e) {
                v.retain(|&l| l != loop_id);
                if v.is_empty() {
                    self.occupied.remove(&e);
                }
            }
        }

        self.loops.remove(loop_id);
    }

    pub fn add_loop(&mut self, l: Loop) -> LoopID {
        *self
            .available_flow_graphs
            .write()
            .expect("available flow graph cache lock poisoned") = None;
        let loop_id = self.loops.insert(l);

        for e in self.loops[loop_id].edges.clone() {
            if !self.occupied.contains_key(&e) {
                self.occupied.insert(&e, vec![]);
            }
            self.occupied.get_mut(&e).unwrap().push(loop_id);
        }

        self.last_loop = Some(loop_id);

        loop_id
    }

    pub fn get_coordinates_of_loop_in_edge(&self, l: LoopID, e: EdgeID) -> Vector3D {
        let loops_in_edge = self.loops_on_edge(e);
        // sort based on the global order of the loops
        // ... todo
        // get the index of the edge in the loop
        let edge_index = self.loops[l].edges.iter().position(|&e2| e2 == e).unwrap();
        let incoming_or_outgoing = edge_index % 2 == 0;
        // find the index of the loop in the sorted list
        let i = {
            if incoming_or_outgoing {
                loops_in_edge.iter().position(|&l2| l2 == l).unwrap() as f64
            } else {
                loops_in_edge.iter().rev().position(|&l2| l2 == l).unwrap() as f64
            }
        };
        // compute the coordinates, based on the index
        let n = loops_in_edge.len() as f64;
        let offset = (i + 1.) / (n + 1.);
        // define the loop segment, starting point is p, ending point is q

        let Some([v1, v2]) = self.mesh_ref.vertices(e).collect_array::<2>() else {
            panic!()
        };
        let p = self.mesh_ref.position(v1);
        let q = self.mesh_ref.position(v2);
        // compute the coordinates
        p + offset * (q - p)
    }

    pub fn get_loops_in_direction(&self, direction: PrincipalDirection) -> Vec<LoopID> {
        self.loops
            .iter()
            .filter_map(|(id, l)| {
                if l.direction == direction {
                    Some(id)
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn count_loops_in_direction(&self, direction: PrincipalDirection) -> usize {
        self.get_loops_in_direction(direction).len()
    }

    pub fn loop_to_direction(&self, loop_id: LoopID) -> PrincipalDirection {
        self.loops[loop_id].direction
    }

    pub fn get_pairs_of_loop(&self, loop_id: LoopID) -> Vec<[EdgeID; 2]> {
        self.get_pairs_of_sequence(&self.loops[loop_id].edges)
    }

    pub fn get_pairs_of_sequence(&self, sequence: &[EdgeID]) -> Vec<[EdgeID; 2]> {
        sequence
            .windows(2)
            .filter_map(|w| {
                if self.mesh_ref.twin(w[0]) == w[1] {
                    None
                } else {
                    Some([w[0], w[1]])
                }
            })
            .collect()
    }

    pub fn cycled_windows(sequence: &[EdgeID]) -> Vec<[EdgeID; 2]> {
        (0..sequence.len())
            .map(|i| {
                let a = sequence[i];
                let b = sequence[(i + 1) % sequence.len()];
                [a, b]
            })
            .collect_vec()
    }

    pub fn is_occupied(&self, [e1, e2]: [EdgeID; 2]) -> Option<LoopID> {
        if self.mesh_ref.twin(e1) == e2 {
            return None;
        }

        let loops_e1 = self.occupied.get(&e1)?;
        let loops_e2 = self.occupied.get(&e2)?;
        let smaller = if loops_e1.len() <= loops_e2.len() {
            loops_e1
        } else {
            loops_e2
        };
        let larger: HashSet<_> = if loops_e1.len() <= loops_e2.len() {
            loops_e2.iter().copied().collect()
        } else {
            loops_e1.iter().copied().collect()
        };

        smaller.iter().copied().find(|loop_id| {
            larger.contains(loop_id)
                && self.loops[*loop_id]
                    .edges
                    .iter()
                    .enumerate()
                    .any(|(i, &a)| {
                        let b =
                            self.loops[*loop_id].edges[(i + 1) % self.loops[*loop_id].edges.len()];
                        (a == e1 && b == e2) || (a == e2 && b == e1)
                    })
        })
    }

    pub fn loops_on_edge(&self, edge: EdgeID) -> Vec<LoopID> {
        self.occupied.get(&edge).cloned().unwrap_or_default()
    }

    pub fn occupied_edgepairs(&self) -> HashSet<(EdgeID, EdgeID)> {
        self.loops
            .values()
            .flat_map(|loop_| {
                Self::cycled_windows(&loop_.edges)
                    .into_iter()
                    .filter(|[a, b]| self.mesh_ref.twin(*a) != *b)
                    .flat_map(|[a, b]| [(a, b), (b, a)])
            })
            .collect()
    }

    pub fn check_loop(&self, lewp: &[EdgeID]) -> Result<(), PropertyViolationError> {
        let edges = lewp;

        if edges.is_empty() {
            return Err(PropertyViolationError::UnknownError);
        }

        // Check if none of the edges are already occupied
        for edge_pair in Self::cycled_windows(edges) {
            if self.is_occupied(edge_pair).is_some() {
                log::debug!("check_loop: rejected, edge pair {edge_pair:?} is already occupied");
                return Err(PropertyViolationError::UnknownError);
            }
        }

        // Check if the loop contains any duplicates
        let mut seen = HashSet::new();
        for &edge in edges {
            if !seen.insert(edge) {
                log::debug!("check_loop: rejected, edge {edge:?} occurs more than once");
                return Err(PropertyViolationError::UnknownError);
            }
        }

        // Check if the loop is valid.
        // A loop should alternate between edges that are twins, and edges that share a face.
        // If `alternate` is true, then the next edge should be a twin of the current edge.
        let mut alternate = self.mesh_ref.twin(edges[0]) == edges[1];
        for edge_pair in Self::cycled_windows(edges) {
            if alternate {
                if self.mesh_ref.twin(edge_pair[0]) != edge_pair[1] {
                    log::debug!(
                        "check_loop: rejected, expected {:?} and {:?} to be twin edges",
                        edge_pair[0],
                        edge_pair[1]
                    );
                    return Err(PropertyViolationError::UnknownError);
                }
                assert!(self.mesh_ref.twin(edge_pair[0]) == edge_pair[1]);
                alternate = false;
            } else {
                if self.mesh_ref.face(edge_pair[0]) != self.mesh_ref.face(edge_pair[1]) {
                    log::debug!(
                        "check_loop: rejected, expected {:?} and {:?} to share a face",
                        edge_pair[0],
                        edge_pair[1]
                    );
                    return Err(PropertyViolationError::UnknownError);
                }
                assert!(self.mesh_ref.face(edge_pair[0]) == self.mesh_ref.face(edge_pair[1]));
                alternate = true;
            }
        }

        Ok(())
    }

    pub fn construct_part_of_loop(
        &self,
        [e1, e2]: [EdgeID; 2],
        domain: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let heuristic_scale = self.flow_graph_heuristic_scale(domain, measure);
        self.construct_part_of_loop_cached(
            [e1, e2],
            domain,
            measure,
            heuristic_scale,
            &mut HashMap::new(),
        )
    }

    fn construct_part_of_loop_cached(
        &self,
        [e1, e2]: [EdgeID; 2],
        domain: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
        heuristic_scale: f64,
        path_cache: &mut HashMap<(EdgeID, EdgeID), Option<(Vec<EdgeID>, f64)>>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        if let Some(cached) = path_cache.get(&(e1, e2)) {
            return cached.clone();
        }

        let result = domain
            .shortest_path_heuristic(e1, e2, measure, |(node, goal)| {
                self.twin_aware_flow_graph_heuristic(node, goal, heuristic_scale)
            })
            .map(|(solution, cost)| (solution, *cost));
        path_cache.insert((e1, e2), result.clone());
        result
    }

    fn flow_graph_heuristic_scale(
        &self,
        graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> f64 {
        graph
            .edges()
            .into_iter()
            .filter(|(from, to, _)| self.mesh_ref.twin(*from) != *to)
            .filter_map(|(from, to, weight)| {
                let geometric_length =
                    (self.mesh_ref.position(to) - self.mesh_ref.position(from)).norm();
                if geometric_length > 1e-12 {
                    Some((*measure(weight) / geometric_length).max(0.0))
                } else {
                    None
                }
            })
            .fold(f64::INFINITY, f64::min)
            .min(1.0)
    }

    /// Admissible lower bound for flow-graph A* searches.
    ///
    /// Twin hops are zero-cost, so plain Euclidean distance between half-edges is
    /// not valid: the search may cross to a twin for free before paying for a
    /// same-face move. This heuristic minimizes distance over the free one-twin
    /// neighborhoods of both endpoints, then scales by the smallest observed
    /// measured-cost/geometric-length ratio of any paid same-face graph edge.
    /// Every paid path has measured cost at least that scale times its geometric
    /// length, and zero-cost twin hops are represented by the endpoint sets, so
    /// this never overestimates the remaining shortest-path cost.
    fn twin_aware_flow_graph_heuristic(
        &self,
        node: EdgeID,
        goal: EdgeID,
        scale: f64,
    ) -> OrderedFloat<f64> {
        if !scale.is_finite() || scale <= 0.0 {
            return OrderedFloat(0.0);
        }

        let node_options = [node, self.mesh_ref.twin(node)];
        let goal_options = [goal, self.mesh_ref.twin(goal)];
        let distance = node_options
            .into_iter()
            .cartesian_product(goal_options)
            .map(|(a, b)| (self.mesh_ref.position(b) - self.mesh_ref.position(a)).norm())
            .fold(f64::INFINITY, f64::min);

        OrderedFloat(scale * distance)
    }

    fn best_loop_from_start_dijkstra(
        &self,
        start: EdgeID,
        graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let mut incoming: HashMap<EdgeID, Vec<(EdgeID, f64)>> = HashMap::new();
        for (from, to, weight) in graph.edges() {
            incoming
                .entry(to)
                .or_default()
                .push((from, *measure(weight)));
        }

        let mut dist = HashMap::from([(start, 0.0)]);
        let mut next_to_target = HashMap::new();
        let mut heap = BinaryHeap::from([DijkstraEntry {
            cost: Reverse(OrderedFloat(0.0)),
            insertion_order: Reverse(0),
            node: start,
        }]);
        let mut counter = 1usize;

        while let Some(DijkstraEntry {
            cost: Reverse(cost),
            node,
            ..
        }) = heap.pop()
        {
            if *cost > dist[&node] {
                continue;
            }
            for &(prev, weight) in incoming.get(&node).map(Vec::as_slice).unwrap_or_default() {
                let next_cost = *cost + weight;
                if next_cost < *dist.get(&prev).unwrap_or(&f64::INFINITY) {
                    dist.insert(prev, next_cost);
                    next_to_target.insert(prev, node);
                    heap.push(DijkstraEntry {
                        cost: Reverse(OrderedFloat(next_cost)),
                        insertion_order: Reverse(counter),
                        node: prev,
                    });
                    counter += 1;
                }
            }
        }

        graph
            .neighbors(start)
            .into_iter()
            .filter_map(|neighbor| {
                let edge_cost = *measure(graph.get_directed_weight(start, neighbor)?);
                let path_cost = *dist.get(&neighbor)?;
                let mut path = vec![neighbor];
                let mut current = neighbor;
                while current != start {
                    current = *next_to_target.get(&current)?;
                    path.push(current);
                }

                let short = self.remove_redundant_same_face_edges(&path);
                self.check_loop(&short).ok()?;
                Some((short, edge_cost + path_cost))
            })
            .min_by_key(|(_, cost)| OrderedFloat(*cost))
    }

    pub fn construct_loop(
        &self,
        [e1, e2]: [EdgeID; 2],
        domain: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        if !domain.node_exists(e1) || !domain.node_exists(e2) {
            return None;
        }

        if domain.node_to_index(&e1).is_some() && domain.node_to_index(&e2).is_some() {
            // Get the better direction.
            let forward = domain.get_directed_weight(e1, e2).map(measure);
            let backward = domain.get_directed_weight(e2, e1).map(measure);
            let (n1, n2, edge_cost) = match (forward, backward) {
                (Some(forward), Some(backward)) => {
                    if forward <= backward {
                        (e1, e2, *forward)
                    } else {
                        (e2, e1, *backward)
                    }
                }
                (Some(forward), None) => (e1, e2, *forward),
                (None, Some(backward)) => (e2, e1, *backward),
                (None, None) => return None,
            };

            let heuristic_scale = self.flow_graph_heuristic_scale(domain, measure);
            let (solution, path_cost) =
                domain.shortest_path_heuristic(n2, n1, measure, |(node, goal)| {
                    self.twin_aware_flow_graph_heuristic(node, goal, heuristic_scale)
                })?;
            let cost = edge_cost + *path_cost;

            let flatten = solution;

            let short = self.remove_redundant_same_face_edges(&flatten);

            self.check_loop(&short).ok()?;

            Some((short, cost))
        } else {
            None
        }
    }

    fn remove_redundant_same_face_edges(&self, edges: &[EdgeID]) -> Vec<EdgeID> {
        if edges.len() < 3 {
            return edges.to_vec();
        }

        let mut short = Vec::with_capacity(edges.len());
        for i in 0..edges.len() {
            let prev = edges[(i + edges.len() - 1) % edges.len()];
            let current = edges[i];
            let next = edges[(i + 1) % edges.len()];
            if self.mesh_ref.face(current) == self.mesh_ref.face(prev)
                && self.mesh_ref.face(current) == self.mesh_ref.face(next)
            {
                continue;
            }
            short.push(current);
        }
        short
    }

    fn orient_anchor_pair(
        &self,
        [e1, e2]: [EdgeID; 2],
        graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<[EdgeID; 2]> {
        let directed_weight = |from, to| -> Option<f64> { graph.get_directed_weight(from, to) };
        let forward = directed_weight(e1, e2).map(measure);
        let backward = directed_weight(e2, e1).map(measure);

        match (forward, backward) {
            (Some(forward), Some(backward)) => {
                if forward <= backward {
                    Some([e1, e2])
                } else {
                    Some([e2, e1])
                }
            }
            (Some(_), None) => Some([e1, e2]),
            (None, Some(_)) => Some([e2, e1]),
            (None, None) => None,
        }
    }

    fn available_flow_graphs(&self) -> Option<Arc<[grapff::fixed::FixedGraph<EdgeID, f64>; 3]>> {
        if let Some(cached) = self
            .available_flow_graphs
            .read()
            .expect("available flow graph cache lock poisoned")
            .as_ref()
        {
            return Some(cached.clone());
        }

        let flow_graphs = self.flow_graphs.as_ref()?;
        let occupied = self.occupied_edgepairs();
        let available = [
            PrincipalDirection::X,
            PrincipalDirection::Y,
            PrincipalDirection::Z,
        ]
        .map(|direction| {
            let filter_edges = |edge: (&EdgeID, &EdgeID)| !occupied.contains(&(*edge.0, *edge.1));
            let filter_nodes = |&node: &EdgeID| {
                !self.occupied.get(&node).is_some_and(|loops| {
                    loops
                        .iter()
                        .any(|&loop_id| self.loops[loop_id].direction == direction)
                })
            };
            flow_graphs[direction as usize]
                .filter_edges(filter_edges)
                .filter_nodes(filter_nodes)
        });

        let available = Arc::new(available);
        *self
            .available_flow_graphs
            .write()
            .expect("available flow graph cache lock poisoned") = Some(available.clone());

        Some(available)
    }

    /// Compute a simple shortest closed loop through one graph node.
    pub fn construct_loop_from_start(
        &self,
        start: EdgeID,
        direction: PrincipalDirection,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let available = self.available_flow_graphs()?;
        let g = &available[direction as usize];
        if !g.node_exists(start) {
            return None;
        }

        self.best_loop_from_start_dijkstra(start, g, &measure)
    }

    /// Compute a simple closed loop through all graph-node anchors in the given order.
    ///
    /// This is intentionally fast/greedy: it joins consecutive anchors with shortest
    /// paths, tries both orientations, then rejects the result if it self-intersects.
    pub fn construct_loop_through_anchor_nodes(
        &self,
        anchors: &[EdgeID],
        direction: PrincipalDirection,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let available = self.available_flow_graphs()?;
        let g = &available[direction as usize];
        self.construct_loop_through_anchor_nodes_in_graph(anchors, g, &measure)
    }

    fn construct_loop_through_anchor_nodes_in_graph(
        &self,
        anchors: &[EdgeID],
        graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        if anchors.is_empty() || anchors.iter().any(|anchor| !graph.node_exists(*anchor)) {
            return None;
        }
        if anchors.len() == 1 {
            return graph
                .neighbors(anchors[0])
                .into_iter()
                .filter_map(|next| self.construct_loop([anchors[0], next], graph, measure))
                .min_by_key(|(_, cost)| OrderedFloat(*cost));
        }

        let heuristic_scale = self.flow_graph_heuristic_scale(graph, measure);
        let mut path_cache = HashMap::new();

        [false, true]
            .into_iter()
            .filter_map(|reverse| {
                let mut ordered = anchors.to_vec();
                if reverse {
                    ordered.reverse();
                }
                ordered.push(ordered[0]);

                let mut loop_edges = Vec::new();
                let mut total_cost = 0.0;
                for (&start, &end) in ordered.iter().tuple_windows() {
                    let (mut part, cost) = self.construct_part_of_loop_cached(
                        [start, end],
                        graph,
                        measure,
                        heuristic_scale,
                        &mut path_cache,
                    )?;
                    if part.len() < 2 {
                        return None;
                    }
                    total_cost += cost;
                    part.pop();
                    loop_edges.extend(part);
                }

                let loop_edges = self.remove_redundant_same_face_edges(&loop_edges);
                self.check_loop(&loop_edges).ok()?;
                Some((loop_edges, total_cost))
            })
            .min_by_key(|(_, cost)| OrderedFloat(*cost))
    }

    pub fn construct_loop_with_anchors(
        &self,
        anchors: &[[EdgeID; 2]],
        direction: PrincipalDirection,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let flow_graph = &self.flow_graphs.as_ref()?[direction as usize];
        let available = self.available_flow_graphs()?;
        let g = &available[direction as usize];

        let anchor_nodes = anchors
            .iter()
            .filter_map(|&[e1, e2]| self.orient_anchor_pair([e1, e2], flow_graph, &measure))
            .flatten()
            .collect_vec();

        self.construct_loop_through_anchor_nodes_in_graph(&anchor_nodes, g, &measure)
    }

    pub fn construct_loop_with_anchors_and_locked_segments(
        &self,
        anchors: &[[EdgeID; 2]],
        direction: PrincipalDirection,
        locked_segments: &[(Vec<EdgeID>, f64)],
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64, Vec<(Vec<EdgeID>, f64)>)> {
        let flow_graph = &self.flow_graphs.as_ref()?[direction as usize];
        let available = self.available_flow_graphs()?;
        let graph = &available[direction as usize];

        let anchor_nodes = anchors
            .iter()
            .filter_map(|&[e1, e2]| self.orient_anchor_pair([e1, e2], flow_graph, &measure))
            .flatten()
            .collect_vec();

        self.construct_loop_through_anchor_nodes_with_locked_segments_in_graph(
            &anchor_nodes,
            graph,
            locked_segments,
            &measure,
        )
    }

    fn construct_loop_through_anchor_nodes_with_locked_segments_in_graph(
        &self,
        anchor_nodes: &[EdgeID],
        graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        locked_segments: &[(Vec<EdgeID>, f64)],
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64, Vec<(Vec<EdgeID>, f64)>)> {
        if anchor_nodes.is_empty()
            || anchor_nodes
                .iter()
                .any(|anchor| !graph.node_exists(*anchor))
        {
            return None;
        }

        if anchor_nodes.len() == 1 {
            let (edges, cost) = graph
                .neighbors(anchor_nodes[0])
                .into_iter()
                .filter_map(|next| self.construct_loop([anchor_nodes[0], next], graph, measure))
                .min_by_key(|(_, cost)| OrderedFloat(*cost))?;
            return Some((edges.clone(), cost, vec![(edges, cost)]));
        }

        let mut path_cache = HashMap::new();
        let heuristic_scale = self.flow_graph_heuristic_scale(graph, measure);
        let mut segments = Vec::with_capacity(anchor_nodes.len());
        let mut total_cost = 0.0;

        for i in 0..anchor_nodes.len() {
            let start = anchor_nodes[i];
            let end = anchor_nodes[(i + 1) % anchor_nodes.len()];

            let (part, cost) = if i < locked_segments.len() {
                let (part, cost) = locked_segments[i].clone();
                if part.first().copied() != Some(start) || part.last().copied() != Some(end) {
                    return None;
                }
                (part, cost)
            } else {
                self.construct_part_of_loop_cached(
                    [start, end],
                    graph,
                    measure,
                    heuristic_scale,
                    &mut path_cache,
                )?
            };

            if part.len() < 2 {
                return None;
            }
            total_cost += cost;
            segments.push((part, cost));
        }

        let mut loop_edges = Vec::new();
        for (part, _) in &segments {
            let mut part = part.clone();
            part.pop();
            loop_edges.extend(part);
        }

        let loop_edges = self.remove_redundant_same_face_edges(&loop_edges);
        self.check_loop(&loop_edges).ok()?;
        Some((loop_edges, total_cost, segments))
    }

    pub fn construct_unbounded_loop(
        &self,
        [e1, e2]: [EdgeID; 2],
        direction: PrincipalDirection,
        _flow_graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let available = self.available_flow_graphs()?;
        let g = &available[direction as usize];

        // `construct_loop` validates the result via `check_loop`, so invalid or
        // self-intersecting candidates are rejected.
        self.construct_loop([e1, e2], g, &measure)
    }

    pub fn sample_loops(
        &self,
        n: usize,
        axis: PrincipalDirection,
        measure: impl Fn(f64) -> OrderedFloat<f64> + std::marker::Sync + std::marker::Send,
        score: impl Fn((&[EdgeID], f64)) -> f64 + std::marker::Sync + std::marker::Send,
    ) -> Vec<Vec<EdgeID>> {
        if let Some(flow_graphs) = &self.flow_graphs {
            (0..n.pow(4))
                .map(|_| {
                    let e1 = self
                        .mesh_ref
                        .edge_ids()
                        .into_iter()
                        .choose(&mut rng())
                        .unwrap();
                    let e2 = self.mesh_ref.next(e1);
                    [e1, e2]
                })
                .sorted_by_key(|&[e1, e2]| {
                    let n1 = flow_graphs[axis as usize].node_to_index(&e1).unwrap();
                    let n2 = flow_graphs[axis as usize].node_to_index(&e2).unwrap();
                    OrderedFloat(measure(
                        flow_graphs[axis as usize].get_weight(n1, n2).to_owned(),
                    ))
                })
                .take(n.pow(2))
                .iter_into_par()
                .filter_map(|es| {
                    self.construct_unbounded_loop(es, axis, &flow_graphs[axis as usize], &measure)
                })
                .collect::<Vec<_>>()
                .into_iter()
                .sorted_by_key(|(path, s)| OrderedFloat(score((path, *s))))
                .take(n)
                .map(|(x, _)| x)
                .collect::<Vec<_>>()
        } else {
            vec![vec![]; n]
        }
    }
}
