//! Loop sampling and loop-query algorithms.

use crate::PropertyViolationError;
use crate::loops::{Loop, LoopID};
use dualcube_types::prelude::*;
use grapff::Grapff;
use orx_parallel::*;
use rand::{rng, seq::IteratorRandom};
use slotmap::SlotMap;
use std::borrow::Cow;
use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::time::Instant;

pub type FlowGraph = grapff::fixed::FixedGraph<EdgeID, f64>;

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
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

pub struct LoopSampler<'a> {
    mesh_ref: &'a Mesh<INPUT>,
    loops: &'a SlotMap<LoopID, Loop>,
    occupied: &'a ids::SecMap<EDGE, INPUT, Vec<LoopID>>,
    flow_graphs: Option<&'a [FlowGraph; 3]>,
}

impl<'a> LoopSampler<'a> {
    #[must_use]
    pub fn new(
        mesh_ref: &'a Mesh<INPUT>,
        loops: &'a SlotMap<LoopID, Loop>,
        occupied: &'a ids::SecMap<EDGE, INPUT, Vec<LoopID>>,
        flow_graphs: Option<&'a [FlowGraph; 3]>,
    ) -> Self {
        Self {
            mesh_ref,
            loops,
            occupied,
            flow_graphs,
        }
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

    pub fn get_loops_in_direction(&self, direction: Direction) -> Vec<LoopID> {
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

    pub fn count_loops_in_direction(&self, direction: Direction) -> usize {
        self.get_loops_in_direction(direction).len()
    }

    pub fn loop_to_direction(&self, loop_id: LoopID) -> Direction {
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
        let timer = Instant::now();
        let edges = lewp;

        if edges.is_empty() {
            info!(
                "b_loops::check_loop: rejected reason=empty elapsed_ms={:.3}",
                elapsed_ms(timer)
            );
            return Err(PropertyViolationError::UnknownError);
        }

        // Check if none of the edges are already occupied
        for (pair_index, edge_pair) in Self::cycled_windows(edges).into_iter().enumerate() {
            if self.is_occupied(edge_pair).is_some() {
                info!("check_loop: rejected, edge pair {edge_pair:?} is already occupied");
                info!(
                    "b_loops::check_loop: rejected reason=occupied edges={} pair_index={} elapsed_ms={:.3}",
                    edges.len(),
                    pair_index,
                    elapsed_ms(timer)
                );
                return Err(PropertyViolationError::UnknownError);
            }
        }

        // Check if the loop contains any duplicates
        let mut seen = HashSet::new();
        for (edge_index, &edge) in edges.iter().enumerate() {
            if !seen.insert(edge) {
                info!("check_loop: rejected, edge {edge:?} occurs more than once");
                info!(
                    "b_loops::check_loop: rejected reason=duplicate edges={} edge_index={} elapsed_ms={:.3}",
                    edges.len(),
                    edge_index,
                    elapsed_ms(timer)
                );
                return Err(PropertyViolationError::UnknownError);
            }
        }

        // Check if the loop is valid.
        // A loop should alternate between edges that are twins, and edges that share a face.
        // If `alternate` is true, then the next edge should be a twin of the current edge.
        let mut alternate = self.mesh_ref.twin(edges[0]) == edges[1];
        for (pair_index, edge_pair) in Self::cycled_windows(edges).into_iter().enumerate() {
            if alternate {
                if self.mesh_ref.twin(edge_pair[0]) != edge_pair[1] {
                    info!(
                        "check_loop: rejected, expected {:?} and {:?} to be twin edges",
                        edge_pair[0], edge_pair[1]
                    );
                    info!(
                        "b_loops::check_loop: rejected reason=not_twins edges={} pair_index={} elapsed_ms={:.3}",
                        edges.len(),
                        pair_index,
                        elapsed_ms(timer)
                    );
                    return Err(PropertyViolationError::UnknownError);
                }
                assert!(self.mesh_ref.twin(edge_pair[0]) == edge_pair[1]);
                alternate = false;
            } else {
                if self.mesh_ref.face(edge_pair[0]) != self.mesh_ref.face(edge_pair[1]) {
                    info!(
                        "check_loop: rejected, expected {:?} and {:?} to share a face",
                        edge_pair[0], edge_pair[1]
                    );
                    info!(
                        "b_loops::check_loop: rejected reason=not_same_face edges={} pair_index={} elapsed_ms={:.3}",
                        edges.len(),
                        pair_index,
                        elapsed_ms(timer)
                    );
                    return Err(PropertyViolationError::UnknownError);
                }
                assert!(self.mesh_ref.face(edge_pair[0]) == self.mesh_ref.face(edge_pair[1]));
                alternate = true;
            }
        }

        info!(
            "b_loops::check_loop: ok edges={} elapsed_ms={:.3}",
            edges.len(),
            elapsed_ms(timer)
        );
        Ok(())
    }

    pub fn construct_part_of_loop(
        &self,
        [e1, e2]: [EdgeID; 2],
        domain: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let timer = Instant::now();
        let heuristic_timer = Instant::now();
        let heuristic_scale = self.flow_graph_heuristic_scale(domain, measure);
        let heuristic_ms = elapsed_ms(heuristic_timer);
        let result = self.construct_part_of_loop_cached(
            [e1, e2],
            domain,
            measure,
            heuristic_scale,
            &mut HashMap::new(),
        );
        info!(
            "b_loops::construct_part_of_loop: from={e1:?} to={e2:?} found={} path_edges={:?} cost={:?} heuristic_scale={:.6} heuristic_ms={:.3} elapsed_ms={:.3}",
            result.is_some(),
            result.as_ref().map(|(path, _)| path.len()),
            result.as_ref().map(|(_, cost)| *cost),
            heuristic_scale,
            heuristic_ms,
            elapsed_ms(timer)
        );
        result
    }

    fn construct_part_of_loop_cached(
        &self,
        [e1, e2]: [EdgeID; 2],
        domain: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
        heuristic_scale: f64,
        path_cache: &mut HashMap<(EdgeID, EdgeID), Option<(Vec<EdgeID>, f64)>>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let timer = Instant::now();
        if let Some(cached) = path_cache.get(&(e1, e2)) {
            info!(
                "b_loops::construct_part_of_loop_cached: cache=hit from={e1:?} to={e2:?} found={} path_edges={:?} cost={:?} elapsed_ms={:.3}",
                cached.is_some(),
                cached.as_ref().map(|(path, _)| path.len()),
                cached.as_ref().map(|(_, cost)| *cost),
                elapsed_ms(timer)
            );
            return cached.clone();
        }

        let shortest_path_timer = Instant::now();
        let result = domain
            .shortest_path_heuristic(e1, e2, measure, |(node, goal)| {
                self.twin_aware_flow_graph_heuristic(node, goal, heuristic_scale)
            })
            .map(|(solution, cost)| (solution, *cost));
        let shortest_path_ms = elapsed_ms(shortest_path_timer);
        path_cache.insert((e1, e2), result.clone());
        info!(
            "b_loops::construct_part_of_loop_cached: cache=miss from={e1:?} to={e2:?} found={} path_edges={:?} cost={:?} shortest_path_ms={:.3} cache_size={} elapsed_ms={:.3}",
            result.is_some(),
            result.as_ref().map(|(path, _)| path.len()),
            result.as_ref().map(|(_, cost)| *cost),
            shortest_path_ms,
            path_cache.len(),
            elapsed_ms(timer)
        );
        result
    }

    fn flow_graph_heuristic_scale(
        &self,
        graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> f64 {
        let timer = Instant::now();
        let edges = graph.edges();
        let edge_count = edges.len();
        let mut paid_edge_count = 0usize;
        let mut skipped_zero_length = 0usize;
        let mut min_scale = f64::INFINITY;

        for (from, to, weight) in edges {
            if self.mesh_ref.twin(from) == to {
                continue;
            }

            paid_edge_count += 1;
            let geometric_length =
                (self.mesh_ref.position(to) - self.mesh_ref.position(from)).norm();
            if geometric_length > 1e-12 {
                min_scale = min_scale.min((*measure(weight) / geometric_length).max(0.0));
            } else {
                skipped_zero_length += 1;
            }
        }

        let scale = min_scale.min(1.0);
        info!(
            "b_loops::flow_graph_heuristic_scale: graph_edges={} paid_edges={} skipped_zero_length={} scale={:.6} elapsed_ms={:.3}",
            edge_count,
            paid_edge_count,
            skipped_zero_length,
            scale,
            elapsed_ms(timer)
        );
        scale
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
        let timer = Instant::now();
        let incoming_timer = Instant::now();
        let graph_edges = graph.edges();
        let graph_edge_count = graph_edges.len();
        let mut incoming: HashMap<EdgeID, Vec<(EdgeID, f64)>> = HashMap::new();
        for (from, to, weight) in graph_edges {
            incoming
                .entry(to)
                .or_default()
                .push((from, *measure(weight)));
        }
        let incoming_ms = elapsed_ms(incoming_timer);

        let dijkstra_timer = Instant::now();
        let mut dist = HashMap::from([(start, 0.0)]);
        let mut next_to_target = HashMap::new();
        let mut heap = BinaryHeap::from([DijkstraEntry {
            cost: Reverse(OrderedFloat(0.0)),
            insertion_order: Reverse(0),
            node: start,
        }]);
        let mut counter = 1usize;
        let mut popped = 0usize;
        let mut stale_popped = 0usize;
        let mut relaxations = 0usize;

        while let Some(DijkstraEntry {
            cost: Reverse(cost),
            node,
            ..
        }) = heap.pop()
        {
            popped += 1;
            if *cost > dist[&node] {
                stale_popped += 1;
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
                    relaxations += 1;
                }
            }
        }
        let dijkstra_ms = elapsed_ms(dijkstra_timer);

        let candidate_timer = Instant::now();
        let mut candidate_count = 0usize;
        let mut missing_directed_weight_count = 0usize;
        let mut missing_path_count = 0usize;
        let mut invalid_loop_count = 0usize;
        let mut valid_loop_count = 0usize;
        let mut best: Option<(Vec<EdgeID>, f64)> = None;

        'neighbors: for neighbor in graph.neighbors(start) {
            candidate_count += 1;
            let Some(directed_weight) = graph.get_directed_weight(start, neighbor) else {
                missing_directed_weight_count += 1;
                continue;
            };
            let edge_cost = *measure(directed_weight);
            let Some(&path_cost) = dist.get(&neighbor) else {
                missing_path_count += 1;
                continue;
            };

            let mut path = vec![neighbor];
            let mut current = neighbor;
            while current != start {
                let Some(&next) = next_to_target.get(&current) else {
                    missing_path_count += 1;
                    continue 'neighbors;
                };
                current = next;
                path.push(current);
            }

            let short = self.remove_redundant_same_face_edges(&path);
            if self.check_loop(&short).is_err() {
                invalid_loop_count += 1;
                continue;
            }

            valid_loop_count += 1;
            let candidate_cost = edge_cost + path_cost;
            if best
                .as_ref()
                .is_none_or(|&(_, best_cost)| candidate_cost < best_cost)
            {
                best = Some((short, candidate_cost));
            }
        }
        let candidate_ms = elapsed_ms(candidate_timer);

        info!(
            "b_loops::best_loop_from_start_dijkstra: start={start:?} found={} graph_edges={} incoming_nodes={} reachable_nodes={} popped={} stale_popped={} relaxations={} candidates={} valid={} invalid={} missing_weight={} missing_path={} incoming_ms={:.3} dijkstra_ms={:.3} candidate_ms={:.3} elapsed_ms={:.3}",
            best.is_some(),
            graph_edge_count,
            incoming.len(),
            dist.len(),
            popped,
            stale_popped,
            relaxations,
            candidate_count,
            valid_loop_count,
            invalid_loop_count,
            missing_directed_weight_count,
            missing_path_count,
            incoming_ms,
            dijkstra_ms,
            candidate_ms,
            elapsed_ms(timer)
        );

        best
    }

    pub fn construct_loop(
        &self,
        edges: [EdgeID; 2],
        domain: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let heuristic_scale = self.flow_graph_heuristic_scale(domain, measure);
        self.construct_loop_with_heuristic_scale(edges, domain, measure, heuristic_scale)
    }

    fn construct_loop_with_heuristic_scale(
        &self,
        [e1, e2]: [EdgeID; 2],
        domain: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
        heuristic_scale: f64,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let timer = Instant::now();
        if !domain.node_exists(e1) || !domain.node_exists(e2) {
            info!(
                "b_loops::construct_loop: rejected reason=missing_node e1={e1:?} e2={e2:?} elapsed_ms={:.3}",
                elapsed_ms(timer)
            );
            return None;
        }

        if domain.node_to_index(&e1).is_some() && domain.node_to_index(&e2).is_some() {
            let orientation_timer = Instant::now();
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
                (None, None) => {
                    info!(
                        "b_loops::construct_loop: rejected reason=no_directed_edge e1={e1:?} e2={e2:?} orientation_ms={:.3} elapsed_ms={:.3}",
                        elapsed_ms(orientation_timer),
                        elapsed_ms(timer)
                    );
                    return None;
                }
            };
            let orientation_ms = elapsed_ms(orientation_timer);

            let shortest_path_timer = Instant::now();
            let Some((solution, path_cost)) =
                domain.shortest_path_heuristic(n2, n1, measure, |(node, goal)| {
                    self.twin_aware_flow_graph_heuristic(node, goal, heuristic_scale)
                })
            else {
                info!(
                    "b_loops::construct_loop: rejected reason=no_return_path e1={e1:?} e2={e2:?} n1={n1:?} n2={n2:?} edge_cost={:.6} heuristic_scale={:.6} orientation_ms={:.3} shortest_path_ms={:.3} elapsed_ms={:.3}",
                    edge_cost,
                    heuristic_scale,
                    orientation_ms,
                    elapsed_ms(shortest_path_timer),
                    elapsed_ms(timer)
                );
                return None;
            };
            let shortest_path_ms = elapsed_ms(shortest_path_timer);
            let path_cost = *path_cost;
            let cost = edge_cost + path_cost;
            let original_edges = solution.len();

            let simplify_timer = Instant::now();
            let short = self.remove_redundant_same_face_edges(&solution);
            let simplify_ms = elapsed_ms(simplify_timer);

            let check_timer = Instant::now();
            if self.check_loop(&short).is_err() {
                info!(
                    "b_loops::construct_loop: rejected reason=invalid_loop e1={e1:?} e2={e2:?} n1={n1:?} n2={n2:?} original_edges={} simplified_edges={} cost={:.6} orientation_ms={:.3} shortest_path_ms={:.3} simplify_ms={:.3} check_ms={:.3} elapsed_ms={:.3}",
                    original_edges,
                    short.len(),
                    cost,
                    orientation_ms,
                    shortest_path_ms,
                    simplify_ms,
                    elapsed_ms(check_timer),
                    elapsed_ms(timer)
                );
                return None;
            }
            let check_ms = elapsed_ms(check_timer);

            info!(
                "b_loops::construct_loop: ok e1={e1:?} e2={e2:?} n1={n1:?} n2={n2:?} original_edges={} simplified_edges={} edge_cost={:.6} path_cost={:.6} cost={:.6} heuristic_scale={:.6} orientation_ms={:.3} shortest_path_ms={:.3} simplify_ms={:.3} check_ms={:.3} elapsed_ms={:.3}",
                original_edges,
                short.len(),
                edge_cost,
                path_cost,
                cost,
                heuristic_scale,
                orientation_ms,
                shortest_path_ms,
                simplify_ms,
                check_ms,
                elapsed_ms(timer)
            );

            Some((short, cost))
        } else {
            info!(
                "b_loops::construct_loop: rejected reason=missing_index e1={e1:?} e2={e2:?} elapsed_ms={:.3}",
                elapsed_ms(timer)
            );
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

    fn available_flow_graph(&self, direction: Direction) -> Option<Cow<'_, FlowGraph>> {
        let timer = Instant::now();
        let Some(flow_graphs) = self.flow_graphs.as_ref() else {
            info!(
                "b_loops::available_flow_graph: rejected reason=no_flow_graphs direction={direction:?} elapsed_ms={:.3}",
                elapsed_ms(timer)
            );
            return None;
        };
        let graph = &flow_graphs[direction as usize];

        let occupied_timer = Instant::now();
        let occupied = self.occupied_edgepairs();
        let blocked_nodes = self
            .occupied
            .iter()
            .filter(|(_, loops)| {
                loops
                    .iter()
                    .any(|&loop_id| self.loops[loop_id].direction == direction)
            })
            .count();
        let occupied_ms = elapsed_ms(occupied_timer);

        if occupied.is_empty() && blocked_nodes == 0 {
            info!(
                "b_loops::available_flow_graph: direction={direction:?} mode=borrowed loops={} occupied_pairs=0 blocked_nodes=0 occupied_ms={:.3} elapsed_ms={:.3}",
                self.loops.len(),
                occupied_ms,
                elapsed_ms(timer)
            );
            return Some(Cow::Borrowed(graph));
        }

        let filter_edges = |edge: (&EdgeID, &EdgeID)| !occupied.contains(&(*edge.0, *edge.1));
        let edge_filter_timer = Instant::now();
        let edge_filtered: Cow<'_, FlowGraph> = if occupied.is_empty() {
            Cow::Borrowed(graph)
        } else {
            Cow::Owned(graph.filter_edges(filter_edges))
        };
        let edge_filter_ms = elapsed_ms(edge_filter_timer);

        let filter_nodes = |&node: &EdgeID| {
            !self.occupied.get(&node).is_some_and(|loops| {
                loops
                    .iter()
                    .any(|&loop_id| self.loops[loop_id].direction == direction)
            })
        };
        let node_filter_timer = Instant::now();
        let filtered = if blocked_nodes == 0 {
            edge_filtered
        } else {
            Cow::Owned(edge_filtered.filter_nodes(filter_nodes))
        };
        let node_filter_ms = elapsed_ms(node_filter_timer);

        info!(
            "b_loops::available_flow_graph: direction={direction:?} mode=filtered loops={} occupied_pairs={} blocked_nodes={} edge_filter_ms={:.3} node_filter_ms={:.3} occupied_ms={:.3} elapsed_ms={:.3}",
            self.loops.len(),
            occupied.len(),
            blocked_nodes,
            edge_filter_ms,
            node_filter_ms,
            occupied_ms,
            elapsed_ms(timer)
        );

        Some(filtered)
    }

    /// Compute a simple shortest closed loop through one graph node.
    pub fn construct_loop_from_start(
        &self,
        start: EdgeID,
        direction: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let timer = Instant::now();
        let available_timer = Instant::now();
        let Some(available) = self.available_flow_graph(direction) else {
            info!(
                "b_loops::construct_loop_from_start: rejected reason=no_available_graph start={start:?} direction={direction:?} available_ms={:.3} elapsed_ms={:.3}",
                elapsed_ms(available_timer),
                elapsed_ms(timer)
            );
            return None;
        };
        let available_ms = elapsed_ms(available_timer);
        let g = &*available;
        if !g.node_exists(start) {
            info!(
                "b_loops::construct_loop_from_start: rejected reason=start_not_available start={start:?} direction={direction:?} available_ms={:.3} elapsed_ms={:.3}",
                available_ms,
                elapsed_ms(timer)
            );
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
        direction: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let timer = Instant::now();
        let available_timer = Instant::now();
        let Some(available) = self.available_flow_graph(direction) else {
            info!(
                "b_loops::construct_loop_through_anchor_nodes: rejected reason=no_available_graph direction={direction:?} anchors={} available_ms={:.3} elapsed_ms={:.3}",
                anchors.len(),
                elapsed_ms(available_timer),
                elapsed_ms(timer)
            );
            return None;
        };
        let g = &*available;
        self.construct_loop_through_anchor_nodes_in_graph(anchors, g, &measure)
    }

    fn construct_loop_through_anchor_nodes_in_graph(
        &self,
        anchors: &[EdgeID],
        graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let timer = Instant::now();
        let missing_anchor_count = anchors
            .iter()
            .filter(|anchor| !graph.node_exists(**anchor))
            .count();
        if anchors.is_empty() || missing_anchor_count > 0 {
            info!(
                "b_loops::construct_loop_through_anchor_nodes_in_graph: rejected reason=invalid_anchors anchors={} missing_anchors={} elapsed_ms={:.3}",
                anchors.len(),
                missing_anchor_count,
                elapsed_ms(timer)
            );
            return None;
        }
        if anchors.len() == 1 {
            let neighbor_timer = Instant::now();
            let neighbors = graph.neighbors(anchors[0]);
            let neighbor_ms = elapsed_ms(neighbor_timer);
            let mut best: Option<(Vec<EdgeID>, f64)> = None;
            let mut candidate_count = 0usize;
            let mut valid_count = 0usize;
            let candidate_timer = Instant::now();
            for next in neighbors.iter().copied() {
                candidate_count += 1;
                if let Some(candidate) = self.construct_loop([anchors[0], next], graph, measure) {
                    valid_count += 1;
                    if best
                        .as_ref()
                        .is_none_or(|&(_, best_cost)| candidate.1 < best_cost)
                    {
                        best = Some(candidate);
                    }
                }
            }
            let candidate_ms = elapsed_ms(candidate_timer);
            info!(
                "b_loops::construct_loop_through_anchor_nodes_in_graph: single_anchor={:?} found={} neighbors={} candidates={} valid={} path_edges={:?} cost={:?} neighbor_ms={:.3} candidate_ms={:.3} elapsed_ms={:.3}",
                anchors[0],
                best.is_some(),
                neighbors.len(),
                candidate_count,
                valid_count,
                best.as_ref().map(|(path, _)| path.len()),
                best.as_ref().map(|(_, cost)| *cost),
                neighbor_ms,
                candidate_ms,
                elapsed_ms(timer)
            );
            return best;
        }

        let heuristic_timer = Instant::now();
        let heuristic_scale = self.flow_graph_heuristic_scale(graph, measure);
        let heuristic_ms = elapsed_ms(heuristic_timer);
        let mut path_cache = HashMap::new();
        let mut best: Option<(Vec<EdgeID>, f64)> = None;
        let mut valid_orientation_count = 0usize;

        for reverse in [false, true] {
            let orientation_timer = Instant::now();
            let mut ordered = anchors.to_vec();
            if reverse {
                ordered.reverse();
            }
            ordered.push(ordered[0]);

            let mut loop_edges = Vec::new();
            let mut total_cost = 0.0;
            let mut segment_count = 0usize;
            let mut failed_reason: Option<&str> = None;
            let mut failed_segment_index: Option<usize> = None;
            let segment_timer = Instant::now();
            for (segment_index, (&start, &end)) in ordered.iter().tuple_windows().enumerate() {
                segment_count += 1;
                let Some((mut part, cost)) = self.construct_part_of_loop_cached(
                    [start, end],
                    graph,
                    measure,
                    heuristic_scale,
                    &mut path_cache,
                ) else {
                    failed_reason = Some("no_path");
                    failed_segment_index = Some(segment_index);
                    break;
                };
                if part.len() < 2 {
                    failed_reason = Some("short_path");
                    failed_segment_index = Some(segment_index);
                    break;
                }
                total_cost += cost;
                part.pop();
                loop_edges.extend(part);
            }
            let segment_ms = elapsed_ms(segment_timer);

            if let Some(reason) = failed_reason {
                info!(
                    "b_loops::construct_loop_through_anchor_nodes_in_graph: orientation=done reverse={} accepted=false reason={} anchors={} segments_done={} failed_segment={:?} cache_size={} segment_ms={:.3} elapsed_ms={:.3}",
                    reverse,
                    reason,
                    anchors.len(),
                    segment_count,
                    failed_segment_index,
                    path_cache.len(),
                    segment_ms,
                    elapsed_ms(orientation_timer)
                );
                continue;
            }

            let simplify_timer = Instant::now();
            let loop_edges = self.remove_redundant_same_face_edges(&loop_edges);
            let simplify_ms = elapsed_ms(simplify_timer);
            let check_timer = Instant::now();
            if self.check_loop(&loop_edges).is_err() {
                info!(
                    "b_loops::construct_loop_through_anchor_nodes_in_graph: orientation=done reverse={} accepted=false reason=invalid_loop anchors={} segments={} loop_edges={} cost={:.6} cache_size={} segment_ms={:.3} simplify_ms={:.3} check_ms={:.3} elapsed_ms={:.3}",
                    reverse,
                    anchors.len(),
                    segment_count,
                    loop_edges.len(),
                    total_cost,
                    path_cache.len(),
                    segment_ms,
                    simplify_ms,
                    elapsed_ms(check_timer),
                    elapsed_ms(orientation_timer)
                );
                continue;
            }
            let check_ms = elapsed_ms(check_timer);

            valid_orientation_count += 1;
            info!(
                "b_loops::construct_loop_through_anchor_nodes_in_graph: orientation=done reverse={} accepted=true anchors={} segments={} loop_edges={} cost={:.6} cache_size={} segment_ms={:.3} simplify_ms={:.3} check_ms={:.3} elapsed_ms={:.3}",
                reverse,
                anchors.len(),
                segment_count,
                loop_edges.len(),
                total_cost,
                path_cache.len(),
                segment_ms,
                simplify_ms,
                check_ms,
                elapsed_ms(orientation_timer)
            );

            if best
                .as_ref()
                .is_none_or(|&(_, best_cost)| total_cost < best_cost)
            {
                best = Some((loop_edges, total_cost));
            }
        }

        info!(
            "b_loops::construct_loop_through_anchor_nodes_in_graph: anchors={} found={} valid_orientations={} path_edges={:?} cost={:?} heuristic_scale={:.6} heuristic_ms={:.3} cache_size={} elapsed_ms={:.3}",
            anchors.len(),
            best.is_some(),
            valid_orientation_count,
            best.as_ref().map(|(path, _)| path.len()),
            best.as_ref().map(|(_, cost)| *cost),
            heuristic_scale,
            heuristic_ms,
            path_cache.len(),
            elapsed_ms(timer)
        );

        best
    }

    pub fn construct_loop_with_anchors(
        &self,
        anchors: &[[EdgeID; 2]],
        direction: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        let timer = Instant::now();
        let Some(flow_graphs) = self.flow_graphs.as_ref() else {
            info!(
                "b_loops::construct_loop_with_anchors: rejected reason=no_flow_graphs direction={direction:?} anchors={} elapsed_ms={:.3}",
                anchors.len(),
                elapsed_ms(timer)
            );
            return None;
        };
        let flow_graph = &flow_graphs[direction as usize];
        let available_timer = Instant::now();
        let Some(available) = self.available_flow_graph(direction) else {
            info!(
                "b_loops::construct_loop_with_anchors: rejected reason=no_available_graph direction={direction:?} anchors={} available_ms={:.3} elapsed_ms={:.3}",
                anchors.len(),
                elapsed_ms(available_timer),
                elapsed_ms(timer)
            );
            return None;
        };
        let g = &*available;

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
        direction: Direction,
        locked_segments: &[(Vec<EdgeID>, f64)],
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64, Vec<(Vec<EdgeID>, f64)>)> {
        let timer = Instant::now();
        let Some(flow_graphs) = self.flow_graphs.as_ref() else {
            info!(
                "b_loops::construct_loop_with_anchors_and_locked_segments: rejected reason=no_flow_graphs direction={direction:?} anchors={} locked_segments={} elapsed_ms={:.3}",
                anchors.len(),
                locked_segments.len(),
                elapsed_ms(timer)
            );
            return None;
        };
        let flow_graph = &flow_graphs[direction as usize];
        let available_timer = Instant::now();
        let Some(available) = self.available_flow_graph(direction) else {
            info!(
                "b_loops::construct_loop_with_anchors_and_locked_segments: rejected reason=no_available_graph direction={direction:?} anchors={} locked_segments={} available_ms={:.3} elapsed_ms={:.3}",
                anchors.len(),
                locked_segments.len(),
                elapsed_ms(available_timer),
                elapsed_ms(timer)
            );
            return None;
        };
        let graph = &*available;

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
        let timer = Instant::now();
        let missing_anchor_count = anchor_nodes
            .iter()
            .filter(|anchor| !graph.node_exists(**anchor))
            .count();
        if anchor_nodes.is_empty() || missing_anchor_count > 0 {
            info!(
                "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: rejected reason=invalid_anchors anchor_nodes={} missing_anchors={} locked_segments={} elapsed_ms={:.3}",
                anchor_nodes.len(),
                missing_anchor_count,
                locked_segments.len(),
                elapsed_ms(timer)
            );
            return None;
        }

        if anchor_nodes.len() == 1 {
            let neighbor_timer = Instant::now();
            let neighbors = graph.neighbors(anchor_nodes[0]);
            let neighbor_ms = elapsed_ms(neighbor_timer);
            let mut best: Option<(Vec<EdgeID>, f64)> = None;
            let mut candidate_count = 0usize;
            let mut valid_count = 0usize;
            let candidate_timer = Instant::now();
            for next in neighbors.iter().copied() {
                candidate_count += 1;
                if let Some(candidate) =
                    self.construct_loop([anchor_nodes[0], next], graph, measure)
                {
                    valid_count += 1;
                    if best
                        .as_ref()
                        .is_none_or(|&(_, best_cost)| candidate.1 < best_cost)
                    {
                        best = Some(candidate);
                    }
                }
            }
            let candidate_ms = elapsed_ms(candidate_timer);
            let Some((edges, cost)) = best else {
                info!(
                    "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: rejected reason=no_single_anchor_loop anchor={:?} neighbors={} candidates={} valid={} locked_segments={} neighbor_ms={:.3} candidate_ms={:.3} elapsed_ms={:.3}",
                    anchor_nodes[0],
                    neighbors.len(),
                    candidate_count,
                    valid_count,
                    locked_segments.len(),
                    neighbor_ms,
                    candidate_ms,
                    elapsed_ms(timer)
                );
                return None;
            };
            info!(
                "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: single_anchor={:?} found=true neighbors={} candidates={} valid={} path_edges={} cost={:.6} locked_segments={} neighbor_ms={:.3} candidate_ms={:.3} elapsed_ms={:.3}",
                anchor_nodes[0],
                neighbors.len(),
                candidate_count,
                valid_count,
                edges.len(),
                cost,
                locked_segments.len(),
                neighbor_ms,
                candidate_ms,
                elapsed_ms(timer)
            );
            return Some((edges.clone(), cost, vec![(edges, cost)]));
        }

        let mut path_cache = HashMap::new();
        let heuristic_timer = Instant::now();
        let heuristic_scale = self.flow_graph_heuristic_scale(graph, measure);
        let heuristic_ms = elapsed_ms(heuristic_timer);
        let mut segments = Vec::with_capacity(anchor_nodes.len());
        let mut total_cost = 0.0;
        let mut locked_segment_count = 0usize;
        let mut computed_segment_count = 0usize;
        let mut locked_segment_ms = 0.0;
        let mut computed_segment_ms = 0.0;

        for i in 0..anchor_nodes.len() {
            let start = anchor_nodes[i];
            let end = anchor_nodes[(i + 1) % anchor_nodes.len()];

            let segment_timer = Instant::now();
            let (part, cost) = if i < locked_segments.len() {
                let (part, cost) = locked_segments[i].clone();
                locked_segment_count += 1;
                locked_segment_ms += elapsed_ms(segment_timer);
                if part.first().copied() != Some(start) || part.last().copied() != Some(end) {
                    info!(
                        "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: rejected reason=locked_segment_mismatch segment={} start={start:?} end={end:?} part_start={:?} part_end={:?} anchor_nodes={} locked_segments={} heuristic_ms={:.3} elapsed_ms={:.3}",
                        i,
                        part.first(),
                        part.last(),
                        anchor_nodes.len(),
                        locked_segments.len(),
                        heuristic_ms,
                        elapsed_ms(timer)
                    );
                    return None;
                }
                (part, cost)
            } else {
                let result = self.construct_part_of_loop_cached(
                    [start, end],
                    graph,
                    measure,
                    heuristic_scale,
                    &mut path_cache,
                );
                computed_segment_count += 1;
                computed_segment_ms += elapsed_ms(segment_timer);
                let Some(result) = result else {
                    info!(
                        "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: rejected reason=no_path segment={} start={start:?} end={end:?} anchor_nodes={} locked_segments={} computed_segments={} cache_size={} heuristic_ms={:.3} locked_segment_ms={:.3} computed_segment_ms={:.3} elapsed_ms={:.3}",
                        i,
                        anchor_nodes.len(),
                        locked_segments.len(),
                        computed_segment_count,
                        path_cache.len(),
                        heuristic_ms,
                        locked_segment_ms,
                        computed_segment_ms,
                        elapsed_ms(timer)
                    );
                    return None;
                };
                result
            };

            if part.len() < 2 {
                info!(
                    "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: rejected reason=short_segment segment={} part_edges={} anchor_nodes={} locked_segments={} cache_size={} heuristic_ms={:.3} locked_segment_ms={:.3} computed_segment_ms={:.3} elapsed_ms={:.3}",
                    i,
                    part.len(),
                    anchor_nodes.len(),
                    locked_segments.len(),
                    path_cache.len(),
                    heuristic_ms,
                    locked_segment_ms,
                    computed_segment_ms,
                    elapsed_ms(timer)
                );
                return None;
            }
            total_cost += cost;
            segments.push((part, cost));
        }

        let assembly_timer = Instant::now();
        let mut loop_edges = Vec::new();
        for (part, _) in &segments {
            let mut part = part.clone();
            part.pop();
            loop_edges.extend(part);
        }
        let assembly_ms = elapsed_ms(assembly_timer);

        let simplify_timer = Instant::now();
        let loop_edges = self.remove_redundant_same_face_edges(&loop_edges);
        let simplify_ms = elapsed_ms(simplify_timer);
        let check_timer = Instant::now();
        if self.check_loop(&loop_edges).is_err() {
            info!(
                "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: rejected reason=invalid_loop anchor_nodes={} locked_segments={} locked_used={} computed={} loop_edges={} cost={:.6} cache_size={} heuristic_ms={:.3} locked_segment_ms={:.3} computed_segment_ms={:.3} assembly_ms={:.3} simplify_ms={:.3} check_ms={:.3} elapsed_ms={:.3}",
                anchor_nodes.len(),
                locked_segments.len(),
                locked_segment_count,
                computed_segment_count,
                loop_edges.len(),
                total_cost,
                path_cache.len(),
                heuristic_ms,
                locked_segment_ms,
                computed_segment_ms,
                assembly_ms,
                simplify_ms,
                elapsed_ms(check_timer),
                elapsed_ms(timer)
            );
            return None;
        }
        let check_ms = elapsed_ms(check_timer);

        info!(
            "b_loops::construct_loop_through_anchor_nodes_with_locked_segments_in_graph: ok anchor_nodes={} locked_segments={} locked_used={} computed={} loop_edges={} cost={:.6} cache_size={} heuristic_scale={:.6} heuristic_ms={:.3} locked_segment_ms={:.3} computed_segment_ms={:.3} assembly_ms={:.3} simplify_ms={:.3} check_ms={:.3} elapsed_ms={:.3}",
            anchor_nodes.len(),
            locked_segments.len(),
            locked_segment_count,
            computed_segment_count,
            loop_edges.len(),
            total_cost,
            path_cache.len(),
            heuristic_scale,
            heuristic_ms,
            locked_segment_ms,
            computed_segment_ms,
            assembly_ms,
            simplify_ms,
            check_ms,
            elapsed_ms(timer)
        );
        Some((loop_edges, total_cost, segments))
    }

    pub fn construct_unbounded_loop(
        &self,
        [e1, e2]: [EdgeID; 2],
        direction: Direction,
        flow_graph: &grapff::fixed::FixedGraph<EdgeID, f64>,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
        heuristic_scale: f64,
    ) -> Option<(Vec<EdgeID>, f64)> {
        // Use the shared base flow graph directly. Filtering/cloning the graph per
        // sampled candidate is expensive; `check_loop` and the final reconstruction
        // reject candidates that collide with existing loops.
        let _ = direction;
        self.construct_loop_with_heuristic_scale([e1, e2], flow_graph, &measure, heuristic_scale)
    }

    pub fn sample_loops(
        &self,
        n: usize,
        axis: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64> + Sync + Send,
        score: impl Fn((&[EdgeID], f64)) -> f64 + Sync + Send,
    ) -> Vec<Vec<EdgeID>> {
        let timer = Instant::now();
        if n == 0 {
            return Vec::new();
        }

        let Some(flow_graphs) = &self.flow_graphs else {
            info!(
                "b_loops::sample_loops: rejected reason=no_flow_graphs axis={axis:?} requested={} elapsed_ms={:.3}",
                n,
                elapsed_ms(timer)
            );
            return vec![vec![]; n];
        };

        let candidate_pool = n.pow(4);
        let selected_target = n.pow(2);

        let generation_timer = Instant::now();
        let anchors = (0..candidate_pool)
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
            .collect_vec();
        let generation_ms = elapsed_ms(generation_timer);

        let sort_timer = Instant::now();
        let selected = anchors
            .into_iter()
            .sorted_by_key(|&[e1, e2]| {
                let n1 = flow_graphs[axis as usize].node_to_index(&e1).unwrap();
                let n2 = flow_graphs[axis as usize].node_to_index(&e2).unwrap();
                OrderedFloat(measure(
                    flow_graphs[axis as usize].get_weight(n1, n2).to_owned(),
                ))
            })
            .take(selected_target)
            .collect_vec();
        let sort_ms = elapsed_ms(sort_timer);
        let selected_count = selected.len();

        let construct_timer = Instant::now();
        let heuristic_scale =
            self.flow_graph_heuristic_scale(&flow_graphs[axis as usize], &measure);
        let constructed = selected
            .into_iter()
            .iter_into_par()
            .filter_map(|es| {
                self.construct_unbounded_loop(
                    es,
                    axis,
                    &flow_graphs[axis as usize],
                    &measure,
                    heuristic_scale,
                )
            })
            .collect::<Vec<_>>();
        let construct_ms = elapsed_ms(construct_timer);
        let constructed_count = constructed.len();

        let score_timer = Instant::now();
        let result = constructed
            .into_iter()
            .sorted_by_key(|(path, s)| OrderedFloat(score((path, *s))))
            .take(n)
            .map(|(x, _)| x)
            .collect::<Vec<_>>();
        let score_ms = elapsed_ms(score_timer);

        let _ = (
            candidate_pool,
            selected_count,
            constructed_count,
            generation_ms,
            sort_ms,
            construct_ms,
            score_ms,
            timer,
        );

        result
    }
}
