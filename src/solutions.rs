use crate::{
    dual::{Dual, Orientation, PropertyViolationError, RegionID, SegmentID},
    graph::Graaf,
    layout::Layout,
    polycube::{Polycube, PolycubeFaceID},
    to_principal_direction, EdgeID, EmbeddedMesh, FaceID, PrincipalDirection,
};
use bevy_egui::egui::emath::align;
use hutspot::{geom::Vector3D, timer::Timer};
use itertools::Itertools;
use ordered_float::OrderedFloat;
use rand::distributions::Distribution;
use rand::{
    distributions::WeightedIndex,
    seq::{IteratorRandom, SliceRandom},
    thread_rng,
};
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use serde::{Deserialize, Serialize};
use slotmap::{SecondaryMap, SlotMap};
use std::io::Write;
use std::{
    collections::{HashMap, HashSet},
    hash::Hash,
    path::PathBuf,
    sync::Arc,
};

slotmap::new_key_type! {
    pub struct LoopID;
}

// A loop forms the basis of the dual structure.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Loop {
    // A loop is defined by a sequence of half-edges.
    pub edges: Vec<EdgeID>,
    // the direction or labeling associated with the loop
    pub direction: PrincipalDirection,
}

impl Loop {
    pub fn contains_pair(&self, needle: (EdgeID, EdgeID)) -> bool {
        hutspot::math::wrap_pairs(&self.edges).into_iter().any(|(a, b)| a == needle.0 && b == needle.1)
    }

    fn find_edge(&self, needle: EdgeID) -> usize {
        self.edges.iter().position(|&e| e == needle).unwrap()
    }

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

    pub fn occupied(loops: &SlotMap<LoopID, Self>) -> SecondaryMap<EdgeID, Vec<LoopID>> {
        let mut occupied: SecondaryMap<EdgeID, Vec<LoopID>> = SecondaryMap::new();
        for loop_id in loops.keys() {
            for &edge in &loops[loop_id].edges {
                if !occupied.contains_key(edge) {
                    occupied.insert(edge, vec![]);
                }
                occupied.get_mut(edge).unwrap().push(loop_id);
            }
        }
        occupied
    }
}

#[derive(Debug, Clone)]
pub struct Solution {
    pub mesh_ref: Arc<EmbeddedMesh>,
    pub loops: SlotMap<LoopID, Loop>,
    occupied: SecondaryMap<EdgeID, Vec<LoopID>>,
    pub dual: Result<Dual, PropertyViolationError>,
    pub polycube: Option<Polycube>,
    pub layout: Option<Result<Layout, PropertyViolationError>>,

    pub alignment_per_triangle: SecondaryMap<FaceID, f64>,
    pub alignment: Option<f64>,
    pub orthogonality_per_patch: SecondaryMap<PolycubeFaceID, f64>,
    pub orthogonality: Option<f64>,
}

impl Solution {
    pub fn new(mesh_ref: Arc<EmbeddedMesh>) -> Self {
        Self {
            mesh_ref,
            loops: SlotMap::with_key(),
            occupied: SecondaryMap::new(),
            dual: Err(PropertyViolationError::default()),
            polycube: None,
            layout: None,
            alignment_per_triangle: SecondaryMap::new(),
            alignment: None,
            orthogonality_per_patch: SecondaryMap::new(),
            orthogonality: None,
        }
    }

    pub fn del_loop(&mut self, loop_id: LoopID) {
        for &e in &self.loops[loop_id].edges.clone() {
            if let Some(v) = self.occupied.get_mut(e) {
                v.retain(|&l| l != loop_id);
                if v.is_empty() {
                    self.occupied.remove(e);
                }
            }
        }

        self.loops.remove(loop_id);
    }

    pub fn add_loop(&mut self, l: Loop) -> LoopID {
        let loop_id = self.loops.insert(l);

        for e in self.loops[loop_id].edges.clone() {
            if !self.occupied.contains_key(e) {
                self.occupied.insert(e, vec![]);
            }
            self.occupied.get_mut(e).unwrap().push(loop_id);
        }

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
        let (startpoint, endpoint) = self.mesh_ref.endpoints(e);
        let p = self.mesh_ref.position(startpoint);
        let q = self.mesh_ref.position(endpoint);
        // compute the coordinates
        p + offset * (q - p)
    }

    pub fn count_loops_in_direction(&self, direction: PrincipalDirection) -> usize {
        self.loops.iter().filter(|(_, l)| l.direction == direction).count()
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
            .filter_map(|w| if self.mesh_ref.twin(w[0]) == w[1] { None } else { Some([w[0], w[1]]) })
            .collect()
    }

    pub fn is_occupied(&self, [e1, e2]: [EdgeID; 2]) -> Option<LoopID> {
        if let Some(loops_e1) = self.occupied.get(e1) {
            if let Some(loops_e2) = self.occupied.get(e2) {
                for &loop_e1 in loops_e1 {
                    for &loop_e2 in loops_e2 {
                        if loop_e1 == loop_e2 && (self.loops[loop_e1].contains_pair((e1, e2)) || self.loops[loop_e1].contains_pair((e2, e1))) {
                            return Some(loop_e1);
                        }
                    }
                }
            }
        }
        None
    }

    pub fn loops_on_edge(&self, edge: EdgeID) -> Vec<LoopID> {
        self.occupied.get(edge).cloned().unwrap_or_default()
    }

    pub fn occupied_edgepairs(&self) -> HashSet<[EdgeID; 2]> {
        self.occupied
            .iter()
            .flat_map(|(edge_id, loops_on_edge)| {
                self.mesh_ref
                    .nexts(edge_id)
                    .iter()
                    .flat_map(|&neighbor_id| {
                        if loops_on_edge.iter().any(|loop_on_edge| self.loops_on_edge(neighbor_id).contains(loop_on_edge)) {
                            vec![[edge_id, neighbor_id], [neighbor_id, edge_id]]
                        } else {
                            vec![]
                        }
                    })
                    .collect_vec()
            })
            .collect()
    }

    pub fn validate_loops(&self) {
        // For all loops, check that the loops is valid
        for (_, lewp) in &self.loops {
            let edges = &lewp.edges;

            // Loop should alternate between edges that are twins, and edges that are sharing a face.
            // If alternate is true, then the next edge should be a twin of the current edge
            let mut alternate = self.mesh_ref.twin(edges[0]) == edges[1];
            for edge_pair in edges.windows(2) {
                if alternate {
                    assert!(self.mesh_ref.twin(edge_pair[0]) == edge_pair[1]);
                    alternate = false;
                } else {
                    assert!(self.mesh_ref.face(edge_pair[0]) == self.mesh_ref.face(edge_pair[1]));
                    alternate = true;
                }
            }

            // Loop should be closed
            // Check the last edge with the first edge, should be closing, depending on alternate they should be twins or sharing a face
            if alternate {
                assert!(self.mesh_ref.twin(edges[edges.len() - 1]) == edges[0]);
            } else {
                assert!(self.mesh_ref.face(edges[edges.len() - 1]) == self.mesh_ref.face(edges[0]));
            }
        }
    }

    pub fn reconstruct_solution(&mut self) {
        self.dual = Err(PropertyViolationError::default());
        self.polycube = None;
        self.layout = None;

        self.validate_loops();
        self.compute_dual();
        self.compute_polycube();
        self.compute_layout();
        // self.resize_polycube();
        self.compute_alignment();
        self.compute_orthogonality();

        if let Some(Ok(layout)) = &self.layout {
            for i in 0..100 {
                let old_score = self.alignment.unwrap_or_default() + self.orthogonality.unwrap_or_default();
                let mut sol_copy = self.clone();

                if let Some(Ok(sol_layout)) = &mut sol_copy.layout {
                    let corner = sol_layout.vert_to_corner.left_values().choose(&mut thread_rng()).unwrap().to_owned();
                    if sol_layout.improve(corner).is_ok() {
                        sol_copy.compute_alignment();
                        let new_score = sol_copy.alignment.unwrap_or_default() + sol_copy.orthogonality.unwrap_or_default();
                        // Calculate percentage improvement
                        let improvement = (new_score - old_score) / old_score;
                        // Print the improvement, color green if positive, red if negative
                        if improvement > 0.0 {
                            println!("{i} : {:.3} to {:.3} \x1b[32m({:+.2}%)\x1b[0m", old_score, new_score, improvement * 100.);
                        } else {
                            println!("{i} : {:.3} to {:.3} \x1b[31m({:+.2}%)\x1b[0m", old_score, new_score, improvement * 100.);
                        }
                        if new_score > old_score {
                            *self = sol_copy;
                        }
                    }
                }
            }
        }
    }

    pub fn compute_dual(&mut self) {
        self.dual = Dual::from(self.mesh_ref.clone(), &self.loops);
    }

    pub fn compute_polycube(&mut self) {
        self.polycube = None;
        if let Ok(dual) = &self.dual {
            let polycube = Polycube::from_dual(dual);
            self.polycube = Some(polycube);
        }
    }

    pub fn compute_layout(&mut self) {
        self.layout = None;
        if let (Ok(dual), Some(polycube)) = (&self.dual, &self.polycube) {
            let layout = Layout::embed(dual, polycube);
            self.layout = Some(layout);
        }
    }

    pub fn resize_polycube(&mut self) {
        if let (Ok(dual), Some(polycube), Some(Ok(layout))) = (&self.dual, &mut self.polycube, &self.layout) {
            polycube.resize(dual, Some(layout));
        }
    }

    pub fn compute_alignment(&mut self) {
        self.alignment_per_triangle.clear();
        self.alignment = None;

        if let (Some(Ok(layout)), Some(polycube)) = (&self.layout, &self.polycube) {
            let total_area: f64 = layout.granulated_mesh.faces.keys().map(|f| layout.granulated_mesh.area(f)).sum();
            let mut total_score = 0.0;

            for (&patch, patch_faces) in &layout.face_to_patch {
                let mapped_normal = polycube.structure.normal(patch).normalize();
                for &triangle_id in &patch_faces.faces {
                    let actual_normal = layout.granulated_mesh.normal(triangle_id);
                    let score = (1. - (1. + std::f64::consts::PI.mul_add(2., -(4. * actual_normal.angle(&mapped_normal))).exp()).recip()).powi(2);
                    self.alignment_per_triangle.insert(triangle_id, score);
                    total_score += score * layout.granulated_mesh.area(triangle_id) / total_area;
                }
            }
            self.alignment = Some(total_score);
        }
    }

    pub fn compute_orthogonality(&mut self) {
        self.orthogonality_per_patch.clear();
        self.orthogonality = None;

        if let (Some(Ok(layout)), Some(polycube)) = (&self.layout, &self.polycube) {
            let total_area: f64 = layout.granulated_mesh.faces.keys().map(|f| layout.granulated_mesh.area(f)).sum();
            let mut total_score = 0.0;

            for (&patch, patch_faces) in &layout.face_to_patch {
                let patch_score: f64 = polycube
                    .structure
                    .edges(patch)
                    .iter()
                    .map(|&patch_edge| {
                        let path = &layout.edge_to_path[&patch_edge];
                        let next_path = &layout.edge_to_path[&polycube.structure.next(patch_edge)];
                        let a = layout.granulated_mesh.position(*path.first().unwrap());
                        let b = layout.granulated_mesh.position(*path.last().unwrap());
                        let c = layout.granulated_mesh.position(*next_path.last().unwrap());
                        (a - b).angle(&(c - b)).sin().powi(2)
                    })
                    .sum::<f64>()
                    / 4.;
                self.orthogonality_per_patch.insert(patch, patch_score);
                let patch_area: f64 = patch_faces.faces.iter().map(|&f| layout.granulated_mesh.area(f)).sum();
                total_score += patch_score * (patch_area / total_area);
            }
            self.orthogonality = Some(total_score);
        }
    }

    pub fn mutate_add_loop(&self, nr_loops: usize, flow_graphs: &[Graaf<[EdgeID; 2], (f64, f64, f64)>; 3]) -> Option<Self> {
        (0..nr_loops)
            .into_par_iter()
            .flat_map(move |_| {
                let timer = Timer::new();
                let mut new_solution = self.clone();
                for _ in 0..3 {
                    let direction = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z]
                        .choose(&mut rand::thread_rng())
                        .unwrap()
                        .to_owned();

                    if let Some(new_loop) = self.construct_loop(direction, flow_graphs) {
                        new_solution.add_loop(Loop { edges: new_loop, direction });
                    }
                }

                new_solution.reconstruct_solution();

                if new_solution.dual.is_ok() && new_solution.polycube.is_some() && new_solution.layout.is_some() {
                    timer.report(&format!(
                        "Found solution\t[alignment: {:.2}]\t[orthogonality: {:.2}]",
                        new_solution.alignment.unwrap(),
                        new_solution.orthogonality.unwrap()
                    ));
                    return Some(new_solution);
                }

                timer.report("Invalid solution :/");
                None
            })
            .max_by_key(|solution| OrderedFloat(solution.alignment.unwrap_or_default()))
    }

    pub fn mutate_del_loop(&self, nr_loops: usize) -> Option<Self> {
        self.loops
            .keys()
            .choose_multiple(&mut rand::thread_rng(), nr_loops)
            .into_par_iter()
            .flat_map(|loop_id| {
                let mut real_solution = self.clone();
                real_solution.del_loop(loop_id);

                let timer = Timer::new();
                real_solution.reconstruct_solution();

                if real_solution.dual.is_ok() && real_solution.polycube.is_some() && real_solution.layout.is_some() {
                    timer.report(&format!(
                        "Found solution\t[alignment: {:.2}]\t[orthogonality: {:.2}]",
                        real_solution.alignment.unwrap(),
                        real_solution.orthogonality.unwrap()
                    ));
                    return Some(real_solution);
                } else {
                    timer.report("Invalid solution :/");
                    return None;
                }
            })
            .max_by_key(|solution| OrderedFloat(solution.alignment.unwrap_or_default()))
    }

    pub fn construct_valid_loop_graph(&self, direction: PrincipalDirection) -> Option<Graaf<SegmentID, f64>> {
        if let Ok(dual) = &self.dual {
            let nodes = dual.loop_structure.edge_ids().into_iter().collect_vec();
            let edges = dual
                .loop_structure
                .edge_ids()
                .into_iter()
                .flat_map(|edge| {
                    let label = dual.segment_to_direction(edge);
                    if label == direction {
                        return vec![];
                    }

                    let twin = dual.loop_structure.twin(edge);
                    let mut valid_neighbors = vec![twin];

                    let neighbors = dual.loop_structure.nexts(edge);
                    let face = [vec![edge], neighbors.clone()].into_iter().flatten().collect_vec();

                    // For each neighbor, figure out their label and orientation.
                    let face_with_labels = face
                        .into_iter()
                        .map(|segment| {
                            let label = dual.segment_to_direction(segment);
                            // TODO: probably fix the mask
                            let orientation = dual.segment_to_orientation(segment);
                            (segment, label, orientation)
                        })
                        .collect_vec();

                    // We are splitting the loop region in two, with a directed edge.
                    // Left = Lower
                    // Right = Upper

                    for i in 1..face_with_labels.len() {
                        // Check for (3) Within each loop region boundary, no two loop segments have the same axis label and side label.
                        // in other words: if I would cut the loop region like this, would it invalidate (3)?

                        // The left side is everything (including current edge) up until and including the neighbor (that will be split)
                        let left_side = face_with_labels[..=i].to_vec();
                        // Check if the left side is valid
                        let mut px = left_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::X && orientation == Orientation::Forwards)
                            .count();

                        let mut py = left_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Y && orientation == Orientation::Forwards)
                            .count();

                        let mut pz = left_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Z && orientation == Orientation::Forwards)
                            .count();

                        let mut mx = left_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::X && orientation == Orientation::Backwards)
                            .count();

                        let mut my = left_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Y && orientation == Orientation::Backwards)
                            .count();

                        let mut mz = left_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Z && orientation == Orientation::Backwards)
                            .count();

                        match direction {
                            PrincipalDirection::X => mx += 1,
                            PrincipalDirection::Y => my += 1,
                            PrincipalDirection::Z => mz += 1,
                        }

                        let left_valid = px <= 1 && py <= 1 && pz <= 1 && mx <= 1 && my <= 1 && mz <= 1;
                        if left_valid {
                            assert!(px + py + pz + mx + my + mz <= 6);
                        }

                        // The right side is the neighbor (that will be split) and everything after the neighbor (that will be split), and the current edge
                        let right_side = face_with_labels[i..].iter().copied().chain(face_with_labels[..1].iter().copied()).collect_vec();
                        // Check if the right side is valid
                        let mut px = right_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::X && orientation == Orientation::Forwards)
                            .count();

                        let mut py = right_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Y && orientation == Orientation::Forwards)
                            .count();

                        let mut pz = right_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Z && orientation == Orientation::Forwards)
                            .count();

                        let mut mx = right_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::X && orientation == Orientation::Backwards)
                            .count();

                        let mut my = right_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Y && orientation == Orientation::Backwards)
                            .count();

                        let mut mz = right_side
                            .iter()
                            .filter(|&&(_, label, orientation)| label == PrincipalDirection::Z && orientation == Orientation::Backwards)
                            .count();

                        match direction {
                            PrincipalDirection::X => px += 1,
                            PrincipalDirection::Y => py += 1,
                            PrincipalDirection::Z => pz += 1,
                        }

                        let right_valid = px <= 1 && py <= 1 && pz <= 1 && mx <= 1 && my <= 1 && mz <= 1;

                        // If both sides are valid, we can add the edge.
                        if left_valid && right_valid {
                            // Add the edge
                            valid_neighbors.push(face_with_labels[i].0);
                        }
                    }

                    valid_neighbors.into_iter().map(move |neighbor| (edge, neighbor, 1.)).collect_vec()
                })
                .collect_vec();

            let graph = Graaf::from(nodes.clone(), edges.clone());

            return Some(graph);
        }

        None
    }

    pub fn find_some_valid_loops_through_region(&self, region_id: RegionID, direction: PrincipalDirection) -> Option<Vec<Vec<SegmentID>>> {
        if let Ok(dual) = &self.dual {
            let region_segments = dual.loop_structure.edges(region_id);
            let full_graph = self.construct_valid_loop_graph(direction)?;
            let ccs = full_graph
                .cc()
                .into_iter()
                .filter(|cc| cc.len() > 1)
                .filter(|cc| cc.iter().any(|&edge| dual.loop_structure.face(edge) == region_id))
                .collect_vec();
            assert!(ccs.len() == 1);
            let reachable_nodes = ccs[0].clone();
            let filtered_edges = full_graph
                .edges()
                .into_iter()
                .filter(|(from, to, _)| reachable_nodes.contains(from) && reachable_nodes.contains(to))
                .collect_vec();
            let filtered_graph = Graaf::from(reachable_nodes.clone(), filtered_edges);

            let mut constraint_map = HashMap::new();

            for (index, region) in dual.loop_structure.face_ids().into_iter().enumerate() {
                for segment in dual
                    .loop_structure
                    .edges(region)
                    .into_iter()
                    .filter(|segment| reachable_nodes.contains(segment))
                {
                    constraint_map.insert(filtered_graph.node_to_index(&segment).unwrap(), index);
                }
            }

            // Find all "shortest" cycles starting from the starting segment to every other reachable segment.
            // e.g. starting segment u, any reachable segment v
            // find the shortest path u->v, and shortest path v->u, and combine them to form a cycle
            // Filter out cycles that invalidate any of the constraints
            let mut cycles = vec![];
            for segment in region_segments.iter().filter(|&segment| reachable_nodes.contains(segment)) {
                let u = filtered_graph.node_to_index(segment).unwrap();
                for other_segment in &reachable_nodes {
                    let v = filtered_graph.node_to_index(other_segment).unwrap();
                    let shortest_path = filtered_graph.shortest_path(u, v, &|x| x).unwrap().1;
                    // remove last element of shortest path, as it is the same as the first element of shortest_path_back
                    let shortest_path = shortest_path.iter().copied().take(shortest_path.len() - 1).collect_vec();
                    let shortest_path_back = filtered_graph.shortest_path(v, u, &|x| x).unwrap().1;
                    // remove last element of shortest path, as it is the same as the first element of shortest_path_back
                    let shortest_path_back = shortest_path_back.iter().copied().take(shortest_path_back.len() - 1).collect_vec();
                    let cycle = [shortest_path, shortest_path_back].concat();
                    cycles.push(cycle);
                }
            }
            println!("Total cycles: {}", cycles.len());

            // Filter out duplicates
            let cycles = cycles
                .into_iter()
                .unique_by(|cycle| {
                    let mut sorted = cycle.clone();
                    sorted.sort();
                    sorted
                })
                .collect_vec();

            // convert NodeIndex to SegmentID
            let cycles = cycles
                .iter()
                .map(|cycle| cycle.iter().map(|&edge| filtered_graph.index_to_node(edge).unwrap().to_owned()).collect_vec())
                .collect_vec();

            // Filter such that all cycles are of length larger than 2
            let cycles = cycles.into_iter().filter(|cycle| cycle.len() > 2).collect_vec();
            println!("Number of cycles after filtering length: {}", cycles.len());

            // Filter such that all cycles do not traverse the same loop region twice
            let cycles = cycles
                .into_iter()
                .filter(|cycle| {
                    let regions = cycle.iter().map(|&edge| dual.loop_structure.face(edge)).collect_vec();
                    let unique_regions = regions.iter().unique().count();
                    unique_regions * 2 == regions.len()
                })
                .collect_vec();
            println!("Number of cycles after filtering duplicate regions: {}", cycles.len());

            return Some(cycles);
        }
        None
    }

    pub fn construct_guaranteed_loop(
        &self,
        region_id: RegionID,
        selected_edges: [EdgeID; 2],
        direction: PrincipalDirection,
        flow_graphs: &[Graaf<[EdgeID; 2], (f64, f64, f64)>; 3],
    ) -> Vec<Vec<EdgeID>> {
        let mut candidate_paths = vec![];

        if let Ok(dual) = &self.dual {
            if let Some(cycle_to_edges) = self.find_some_valid_loops_through_region(region_id, direction) {
                candidate_paths = cycle_to_edges
                    .into_par_iter()
                    .flat_map(|chosen_cycle| {
                        let mut augmented_cycle = chosen_cycle.clone();

                        // Find a path that goes exactly through the selected segments of the cycle.

                        let all_segments = chosen_cycle
                            .iter()
                            .map(|&edge| dual.loop_structure.nexts(edge))
                            .flatten()
                            .collect::<HashSet<_>>();
                        let blocked_segments = all_segments
                            .iter()
                            .copied()
                            .filter(|&segment| !augmented_cycle.contains(&segment))
                            .collect::<HashSet<_>>();
                        let blocked_edges = blocked_segments
                            .into_iter()
                            .flat_map(|segment| dual.segment_to_edges(segment))
                            .collect::<HashSet<_>>();

                        let mut blocked_edges2 = HashSet::new();
                        // make sure every two segments in chosen_cycle are pairs, it could be the case that the 1st and nth segment are a pair
                        if augmented_cycle[0] == dual.loop_structure.twin(chosen_cycle[augmented_cycle.len() - 1]) {
                            augmented_cycle.push(augmented_cycle[0]);
                            augmented_cycle.remove(0);
                        }

                        // Find all directed edges passing through the selected segments (chosen_cycle)
                        for selected_segment_pair in augmented_cycle.windows(2) {
                            let (segment1, segment2) = (selected_segment_pair[0], selected_segment_pair[1]);
                            // we consider this pair only if segment1 is the first segment, and twin of segment2
                            if segment1 != dual.loop_structure.twin(segment2) {
                                continue;
                            }

                            let edges = dual.segment_to_edges(segment1);
                            // their edges are the same, but we only consider segment1, as its the first segment

                            // for every two edges, find the third edge of the passed triangle
                            for edge_pair in edges.windows(2) {
                                let (e1, e2) = (edge_pair[0], edge_pair[1]);

                                if e1 == self.mesh_ref.twin(e2) {
                                    continue;
                                }

                                let triangle = self.mesh_ref.face(e1);
                                assert!(triangle == self.mesh_ref.face(e2));

                                let third_edge = self.mesh_ref.edges(triangle).into_iter().find(|&e| e != e1 && e != e2).unwrap();

                                // figure out if the third edge is adjacent to this segment (or to its twin)
                                let this_region = dual.loop_structure.face(segment1);
                                let this_region_verts = &dual.loop_structure.faces[this_region].verts;
                                let (vert1, vert2) = self.mesh_ref.endpoints(third_edge);
                                if this_region_verts.contains(&vert1) && this_region_verts.contains(&vert2) {
                                    // this is edge is inside the region adjacent to the segment
                                    // this means that we allow traversal FROM this edge, into the segment/loop
                                    // this means we do NOT allow traversal FROM the segment/loop, into this edge

                                    // as such, we block the edges from the segment/loop to this edge
                                    blocked_edges2.insert([third_edge, e1]);
                                    blocked_edges2.insert([third_edge, e2]);
                                } else {
                                    // this edge is outside the region adjacent to the segment
                                    // this means that we allow traversal FROM the segment/loop, into this edge
                                    // this means we do NOT allow traversal FROM this edge, into the segment/loop

                                    // as such, we block the edges from this edge to the segment/loop

                                    blocked_edges2.insert([e1, third_edge]);
                                    blocked_edges2.insert([e2, third_edge]);
                                }
                            }
                        }

                        let filter = |(a, b): (&[EdgeID; 2], &[EdgeID; 2])| {
                            !blocked_edges.contains(&a[0])
                                && !blocked_edges.contains(&a[1])
                                && !blocked_edges.contains(&b[0])
                                && !blocked_edges.contains(&b[1])
                                && !blocked_edges2.contains(a)
                                && !blocked_edges2.contains(b)
                        };

                        let g_original = &flow_graphs[direction as usize];
                        let g = g_original.filter(filter);

                        // between 5 and 15
                        let alpha = rand::random::<f64>();

                        let measure = |(a, b, c): (f64, f64, f64)| alpha * a.powi(10) + (1. - alpha) * b.powi(10);

                        let [e1, e2] = selected_edges;

                        let a = g.node_to_index(&[e1, e2]).unwrap();
                        let b = g.node_to_index(&[e2, e1]).unwrap();
                        let mut option_a = g
                            .shortest_cycle(a, &measure)
                            .unwrap_or_default()
                            .into_iter()
                            .flat_map(|node_index| g.index_to_node(node_index).unwrap().to_owned())
                            .collect_vec();
                        let mut option_b = g
                            .shortest_cycle(b, &measure)
                            .unwrap_or_default()
                            .into_iter()
                            .flat_map(|node_index| g.index_to_node(node_index).unwrap().to_owned())
                            .collect_vec();

                        assert!(option_a.len() % 2 == 0);
                        assert!(option_b.len() % 2 == 0);

                        let mut option_a_valid = true;
                        let mut option_b_valid = true;

                        // If path has duplicates, the option is invalid
                        option_a_valid = option_a.iter().unique().count() == option_a.len();
                        option_b_valid = option_b.iter().unique().count() == option_b.len();

                        // If path has less than 5 edges, the option is invalid
                        option_a_valid = option_a.len() >= 5;
                        option_b_valid = option_b.len() >= 5;

                        if !option_a_valid && !option_b_valid {
                            return None;
                        }

                        if option_a_valid && !option_b_valid {
                            return Some(option_a);
                        }

                        if !option_a_valid && option_b_valid {
                            return Some(option_b);
                        }

                        let weight_option_a: f64 = option_a
                            .chunks(2)
                            .map(|window| [window[0], window[1]])
                            .collect_vec()
                            .windows(2)
                            .map(|pairs| {
                                let pair1 = pairs[0];
                                let pair2 = pairs[1];

                                let pair1_node = g.node_to_index(&pair1).unwrap();
                                let pair2_node = g.node_to_index(&pair2).unwrap();

                                let weight = g.get_weight(pair1_node, pair2_node);
                                measure(weight)
                            })
                            .sum();

                        let weight_option_b: f64 = option_b
                            .chunks(2)
                            .map(|window| [window[0], window[1]])
                            .collect_vec()
                            .windows(2)
                            .map(|pairs| {
                                let pair1 = pairs[0];
                                let pair2 = pairs[1];

                                let pair1_node = g.node_to_index(&pair1).unwrap();
                                let pair2_node = g.node_to_index(&pair2).unwrap();

                                let weight = g.get_weight(pair1_node, pair2_node);
                                measure(weight)
                            })
                            .sum();

                        let best_option = if weight_option_a > weight_option_b { option_b } else { option_a };

                        Some(best_option)
                    })
                    .collect();
            }
        }

        println!("Found {} candidate paths", candidate_paths.len());

        candidate_paths
    }

    pub fn construct_loop(&self, direction: PrincipalDirection, flow_graphs: &[Graaf<[EdgeID; 2], (f64, f64, f64)>; 3]) -> Option<Vec<EdgeID>> {
        let occupied = self.occupied_edgepairs();
        let filter = |(a, b): (&[EdgeID; 2], &[EdgeID; 2])| {
            !occupied.contains(a)
                && !occupied.contains(b)
                && [a[0], a[1], b[0], b[1]].iter().all(|&edge| {
                    self.loops_on_edge(edge)
                        .iter()
                        .filter(|&&loop_id| self.loop_to_direction(loop_id) == direction)
                        .count()
                        == 0
                })
        };

        let g_original = &flow_graphs[direction as usize];
        let g = g_original.filter(filter);

        // between 5 and 15
        let alpha = rand::random::<f64>();

        let measure = |(a, b, c): (f64, f64, f64)| alpha * a.powi(10) + (1. - alpha) * b.powi(10);

        let edges = self.mesh_ref.edges.keys().collect_vec();
        // map edges to alignment of their two neighboring triangles in a vec of tuples
        let alignments = edges
            .iter()
            .map(|&edge| {
                let twin = self.mesh_ref.twin(edge);
                let triangles = [self.mesh_ref.face(edge), self.mesh_ref.face(twin)];
                let alignment = triangles
                    .iter()
                    .map(|&triangle| self.alignment_per_triangle.get(triangle).unwrap())
                    .sum::<f64>()
                    / triangles.len() as f64;
                (1. - alignment) * 0.8 + 0.2
            })
            .collect_vec();

        let dist = WeightedIndex::new(&alignments).unwrap();

        let mut rng = thread_rng();

        // select randomly an edge with skew to lower alignment
        let e1 = edges[dist.sample(&mut rng)];

        // Starting edges.
        // let e1 = self.mesh_ref.random_edges(1).first().unwrap().to_owned();
        let e2 = self.mesh_ref.next(e1);
        let a = g.node_to_index(&[e1, e2]).unwrap();
        let b = g.node_to_index(&[e2, e1]).unwrap();
        let option_a = g
            .shortest_cycle(a, &measure)
            .unwrap_or_default()
            .into_iter()
            .flat_map(|node_index| g.index_to_node(node_index).unwrap().to_owned())
            .collect_vec();
        let option_b = g
            .shortest_cycle(b, &measure)
            .unwrap_or_default()
            .into_iter()
            .flat_map(|node_index| g.index_to_node(node_index).unwrap().to_owned())
            .collect_vec();

        // The path may contain self intersections. We can remove these.
        // If duplicated vertices are present, remove everything between them.
        let mut cleaned_option_a = vec![];
        for edge_id in option_a {
            if cleaned_option_a.contains(&edge_id) {
                cleaned_option_a = cleaned_option_a.into_iter().take_while(|&x| x != edge_id).collect_vec();
            }
            cleaned_option_a.push(edge_id);
        }

        let mut cleaned_option_b = vec![];
        for edge_id in option_b {
            if cleaned_option_b.contains(&edge_id) {
                cleaned_option_b = cleaned_option_b.into_iter().take_while(|&x| x != edge_id).collect_vec();
            }
            cleaned_option_b.push(edge_id);
        }

        let best_option = if cleaned_option_a.len() > cleaned_option_b.len() {
            cleaned_option_a
        } else {
            cleaned_option_b
        };

        // If the best option is empty, we have no valid path.
        if best_option.len() < 5 {
            return None;
        }

        return Some(best_option);
    }

    pub fn write_to_flag(&self, path: &PathBuf) -> std::io::Result<()> {
        let mut file = std::fs::File::create(path)?;

        if let Some(Ok(layout)) = &self.layout {
            let map = layout
                .granulated_mesh
                .face_ids()
                .into_iter()
                .enumerate()
                .map(|(index, face_id)| (face_id, index))
                .collect::<HashMap<_, _>>();
            let mut labels = vec![-1; layout.granulated_mesh.face_ids().len()];

            for &patch_id in layout.face_to_patch.keys() {
                let normal = (layout.polycube_ref.structure.normal(patch_id) as Vector3D).normalize();
                let label = match to_principal_direction(normal) {
                    (PrincipalDirection::X, Orientation::Forwards) => 0,
                    (PrincipalDirection::X, Orientation::Backwards) => 1,
                    (PrincipalDirection::Y, Orientation::Forwards) => 2,
                    (PrincipalDirection::Y, Orientation::Backwards) => 3,
                    (PrincipalDirection::Z, Orientation::Forwards) => 4,
                    (PrincipalDirection::Z, Orientation::Backwards) => 5,
                };
                for &face_id in &layout.face_to_patch[&patch_id].faces {
                    labels[map[&face_id]] = label;
                }
            }

            let labels_out = labels.iter().map(|&x| x.to_string()).collect::<Vec<_>>().join("\n");
            write!(file, "{labels_out}").unwrap();

            return Ok(());
        }
        Err(std::io::Error::new(std::io::ErrorKind::Other, "No layout available"))
    }

    pub fn write_to_obj(&self, path: &PathBuf) -> std::io::Result<()> {
        if let Some(Ok(layout)) = &self.layout {
            layout.granulated_mesh.write_to_obj(path)?;
            return Ok(());
        }
        Err(std::io::Error::new(std::io::ErrorKind::Other, "No layout available"))
    }

    pub fn export(&self, path_obj: &PathBuf, path_flag: &PathBuf) -> std::io::Result<()> {
        self.write_to_obj(path_obj)?;
        self.write_to_flag(path_flag)?;
        Ok(())
    }
}
