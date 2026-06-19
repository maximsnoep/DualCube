//! Compatibility wrappers for loop state and sampling.
//!
//! The loop data model and algorithms live in `dualcube-dual`. `Solution`
//! still exposes the old methods so GUI/CLI code can migrate gradually.

use crate::prelude::*;

impl Solution {
    fn loop_state_mut(&mut self) -> LoopStateMut<'_> {
        LoopStateMut {
            loops: &mut self.loops,
            occupied: &mut self.occupied,
            last_loop: &mut self.last_loop,
        }
    }

    fn loop_sampler(&self) -> LoopSampler<'_> {
        LoopSampler::new(
            self.mesh_ref.as_ref(),
            &self.loops,
            &self.occupied,
            self.flow_graphs.as_deref(),
        )
    }

    pub fn recompute_occupied(&mut self) {
        self.loop_state_mut().recompute_occupied();
    }

    pub fn del_loop(&mut self, loop_id: LoopID) {
        self.loop_state_mut().del_loop(loop_id);
    }

    pub fn add_loop(&mut self, loop_: Loop) -> LoopID {
        self.loop_state_mut().add_loop(loop_)
    }

    pub fn get_coordinates_of_loop_in_edge(&self, loop_id: LoopID, edge: EdgeID) -> Vector3D {
        self.loop_sampler()
            .get_coordinates_of_loop_in_edge(loop_id, edge)
    }

    pub fn get_loops_in_direction(&self, direction: Direction) -> Vec<LoopID> {
        self.loop_sampler().get_loops_in_direction(direction)
    }

    pub fn count_loops_in_direction(&self, direction: Direction) -> usize {
        self.loop_sampler().count_loops_in_direction(direction)
    }

    pub fn loop_to_direction(&self, loop_id: LoopID) -> Direction {
        self.loop_sampler().loop_to_direction(loop_id)
    }

    pub fn get_pairs_of_loop(&self, loop_id: LoopID) -> Vec<[EdgeID; 2]> {
        self.loop_sampler().get_pairs_of_loop(loop_id)
    }

    pub fn get_pairs_of_sequence(&self, sequence: &[EdgeID]) -> Vec<[EdgeID; 2]> {
        self.loop_sampler().get_pairs_of_sequence(sequence)
    }

    pub fn cycled_windows(sequence: &[EdgeID]) -> Vec<[EdgeID; 2]> {
        LoopSampler::cycled_windows(sequence)
    }

    pub fn is_occupied(&self, edge_pair: [EdgeID; 2]) -> Option<LoopID> {
        self.loop_sampler().is_occupied(edge_pair)
    }

    pub fn loops_on_edge(&self, edge: EdgeID) -> Vec<LoopID> {
        self.loop_sampler().loops_on_edge(edge)
    }

    pub fn occupied_edgepairs(&self) -> HashSet<(EdgeID, EdgeID)> {
        self.loop_sampler().occupied_edgepairs()
    }

    pub fn check_loop(&self, loop_edges: &[EdgeID]) -> Result<(), PropertyViolationError> {
        self.loop_sampler().check_loop(loop_edges)
    }

    pub fn construct_part_of_loop(
        &self,
        edges: [EdgeID; 2],
        domain: &FlowGraph,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        self.loop_sampler()
            .construct_part_of_loop(edges, domain, measure)
    }

    pub fn construct_loop(
        &self,
        edges: [EdgeID; 2],
        domain: &FlowGraph,
        measure: &impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        self.loop_sampler().construct_loop(edges, domain, measure)
    }

    pub fn construct_loop_from_start(
        &self,
        start: EdgeID,
        direction: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        self.loop_sampler()
            .construct_loop_from_start(start, direction, measure)
    }

    pub fn construct_loop_through_anchor_nodes(
        &self,
        anchors: &[EdgeID],
        direction: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        self.loop_sampler()
            .construct_loop_through_anchor_nodes(anchors, direction, measure)
    }

    pub fn construct_loop_with_anchors(
        &self,
        anchors: &[[EdgeID; 2]],
        direction: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64)> {
        self.loop_sampler()
            .construct_loop_with_anchors(anchors, direction, measure)
    }

    pub fn construct_loop_with_anchors_and_locked_segments(
        &self,
        anchors: &[[EdgeID; 2]],
        direction: Direction,
        locked_segments: &[(Vec<EdgeID>, f64)],
        measure: impl Fn(f64) -> OrderedFloat<f64>,
    ) -> Option<(Vec<EdgeID>, f64, Vec<(Vec<EdgeID>, f64)>)> {
        self.loop_sampler()
            .construct_loop_with_anchors_and_locked_segments(
                anchors,
                direction,
                locked_segments,
                measure,
            )
    }

    pub fn construct_unbounded_loop(
        &self,
        edges: [EdgeID; 2],
        direction: Direction,
        flow_graph: &FlowGraph,
        measure: impl Fn(f64) -> OrderedFloat<f64>,
        heuristic_scale: f64,
    ) -> Option<(Vec<EdgeID>, f64)> {
        self.loop_sampler().construct_unbounded_loop(
            edges,
            direction,
            flow_graph,
            measure,
            heuristic_scale,
        )
    }

    pub fn sample_loops(
        &self,
        n: usize,
        axis: Direction,
        measure: impl Fn(f64) -> OrderedFloat<f64> + Sync + Send,
        score: impl Fn((&[EdgeID], f64)) -> f64 + Sync + Send,
    ) -> Vec<Vec<EdgeID>> {
        self.loop_sampler().sample_loops(n, axis, measure, score)
    }
}
