use crate::{
    dual::{Dual, IntersectionID, PrincipalDirection, PropertyViolationError, RegionID, SegmentID, Side, ZoneID},
    graph::Graaf,
    layout::Layout,
    polycube::{Polycube, PolycubeFaceID, PolycubeVertID},
    EdgeID, EmbeddedMesh, FaceID,
};
use bevy_egui::egui::debug_text::print;
use douconel::douconel_embedded::HasPosition;
use hutspot::{consts::PI, geom::Vector3D, timer::Timer};
use itertools::Itertools;
use ordered_float::OrderedFloat;
use petgraph::algo::tarjan_scc;
use rand::distributions::Distribution;
use rand::{
    distributions::WeightedIndex,
    seq::{IteratorRandom, SliceRandom},
    thread_rng,
};
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use serde::{Deserialize, Serialize};
use slotmap::{SecondaryMap, SlotMap};
use std::{
    collections::{HashMap, HashSet},
    hash::Hash,
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

    pub fn occupied(loops: &SlotMap<LoopID, Loop>) -> SecondaryMap<EdgeID, Vec<LoopID>> {
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
    pub dual: Result<Dual, PropertyViolationError<IntersectionID>>,
    pub polycube: Option<Polycube>,
    pub layout: Option<Result<Layout, PropertyViolationError<PolycubeVertID>>>,

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

        // for [e0, e1] in self.get_pairs_of_loop(loop_id) {
        //     if let Some(l) = self.is_occupied([e0, e1]) {
        //         assert!(l == loop_id, "Loop: {loop_id:?} already occupied by {l:?} on edge {:?}", [e0, e1]);
        //     }
        // }

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

    pub fn reconstruct_solution(&mut self, smoothen: bool) {
        self.dual = Err(PropertyViolationError::default());
        self.polycube = None;
        self.layout = None;

        self.compute_dual();
        self.compute_polycube();
        self.compute_layout(smoothen);
        self.resize_polycube();
        self.compute_alignment();
        self.compute_orthogonality();
    }

    pub fn smoothen(&mut self) {
        self.layout.as_mut().unwrap().as_mut().unwrap().smoothening();
        self.layout.as_mut().unwrap().as_mut().unwrap().assign_patches().unwrap();
        self.resize_polycube();
        self.compute_alignment();
        self.compute_orthogonality();
    }

    pub fn compute_dual(&mut self) {
        self.dual = Dual::from(self.mesh_ref.clone(), &self.loops);
    }

    pub fn compute_polycube(&mut self) {
        if let Ok(dual) = &self.dual {
            let polycube = Polycube::from_dual(dual);
            self.polycube = Some(polycube);
        } else {
            self.polycube = None;
        }
        self.layout = None;
    }

    pub fn compute_layout(&mut self, smoothen: bool) {
        self.layout = None;
        if let (Ok(dual), Some(polycube)) = (&self.dual, &self.polycube) {
            for _ in 0..5 {
                let layout = Layout::embed(dual, polycube, smoothen);
                match &layout {
                    Ok(ok_layout) => {
                        let (score_a, score_b) = ok_layout.score();
                        let score = score_a + score_b;
                        if score_a < 0.1 {
                            self.layout = Some(layout);
                            return;
                        }
                    }
                    Err(e) => {
                        println!("Failed to embed layout: {:?}", e);
                    }
                }
            }
        }
    }

    pub fn resize_polycube(&mut self) {
        if let (Ok(dual), Some(Ok(layout)), Some(polycube)) = (&mut self.dual, &self.layout, &mut self.polycube) {
            let (adjacency, adjacency_backwards) = &dual.adjacency;
            let topological_sort = hutspot::graph::topological_sort::<ZoneID>(&dual.zones.clone().into_iter().map(|(id, _)| id).collect_vec(), |z| {
                adjacency.get(&z).cloned().unwrap_or_default().into_iter().collect_vec()
            });

            if topological_sort.is_none() {
                return;
            }

            for zone in dual.zones.values_mut() {
                zone.coordinate = None;
            }

            for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                let mut topo = topological_sort
                    .clone()
                    .unwrap()
                    .into_iter()
                    .filter(|&z| dual.zones[z].direction == direction)
                    .collect_vec();

                let mut zero_is_set = false;

                for &zone_id in &topo {
                    let dependencies = adjacency_backwards
                        .get(&zone_id)
                        .cloned()
                        .unwrap_or_default()
                        .iter()
                        .filter_map(|&z| dual.zones[z].coordinate)
                        .collect_vec();

                    let average_coord = hutspot::math::calculate_average_f64(
                        dual.zones[zone_id]
                            .regions
                            .iter()
                            .filter_map(|&r| polycube.region_to_vertex.get_by_left(&r))
                            .filter_map(|&v| layout.vert_to_corner.get_by_left(&v))
                            .map(|&c| layout.granulated_mesh.position(c)[direction as usize]),
                    );

                    if dependencies.is_empty() {
                        if zero_is_set {
                            continue;
                        }
                        zero_is_set = true;
                        dual.zones[zone_id].coordinate = Some(average_coord);
                    } else {
                        let max_dependency = dependencies.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();
                        let new_coord = (average_coord).max(max_dependency + 0.1);
                        dual.zones[zone_id].coordinate = Some(new_coord);
                    };
                }

                topo.reverse();

                for &zone_id in &topo {
                    if dual.zones[zone_id].coordinate.is_none() {
                        let dependencies = adjacency
                            .get(&zone_id)
                            .cloned()
                            .unwrap_or_default()
                            .iter()
                            .filter_map(|&z| dual.zones[z].coordinate)
                            .collect_vec();

                        let average_coord = hutspot::math::calculate_average_f64(
                            dual.zones[zone_id]
                                .regions
                                .iter()
                                .flat_map(|&r| polycube.region_to_vertex.get_by_left(&r))
                                .flat_map(|&v| layout.vert_to_corner.get_by_left(&v))
                                .map(|&c| layout.granulated_mesh.position(c)[direction as usize]),
                        );

                        assert!(!dependencies.is_empty());
                        let min_dependency = dependencies.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();
                        let new_coord = (average_coord).min(min_dependency - 0.1);
                        dual.zones[zone_id].coordinate = Some(new_coord);
                    }
                }
            }

            for region_id in dual.loop_structure.face_ids() {
                let coordinate = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|d| {
                    dual.zones
                        .iter()
                        .find(|(_, z)| z.regions.contains(&region_id) && z.direction == d)
                        .unwrap()
                        .1
                        .coordinate
                        .unwrap()
                });
                let vertex = polycube.region_to_vertex.get_by_left(&region_id).unwrap().to_owned();
                polycube.structure.verts[vertex].set_position(Vector3D::new(coordinate[0], coordinate[1], coordinate[2]));
            }
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

                new_solution.reconstruct_solution(false);

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
                real_solution.reconstruct_solution(false);

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

    pub fn find_valid_loops_through_region(&self, region_id: RegionID, direction: PrincipalDirection) -> Option<Vec<Vec<SegmentID>>> {
        if let Ok(dual) = &self.dual {
            let nodes = dual.loop_structure.edge_ids().into_iter().collect_vec();
            let edges = dual
                .loop_structure
                .edge_ids()
                .into_iter()
                .flat_map(|edge| {
                    let label = dual.segment_to_label(edge);
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
                            let label = dual.segment_to_label(segment);
                            // TODO: probably fix the mask
                            let side = dual.segment_to_side(segment, [0, 0, 0]);
                            (segment, label, side)
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
                            .filter(|&&(_, label, side)| label == PrincipalDirection::X && side == Side::Upper)
                            .count();

                        let mut py = left_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Y && side == Side::Upper)
                            .count();

                        let mut pz = left_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Z && side == Side::Upper)
                            .count();

                        let mut mx = left_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::X && side == Side::Lower)
                            .count();

                        let mut my = left_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Y && side == Side::Lower)
                            .count();

                        let mut mz = left_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Z && side == Side::Lower)
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
                            .filter(|&&(_, label, side)| label == PrincipalDirection::X && side == Side::Upper)
                            .count();

                        let mut py = right_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Y && side == Side::Upper)
                            .count();

                        let mut pz = right_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Z && side == Side::Upper)
                            .count();

                        let mut mx = right_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::X && side == Side::Lower)
                            .count();

                        let mut my = right_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Y && side == Side::Lower)
                            .count();

                        let mut mz = right_side
                            .iter()
                            .filter(|&&(_, label, side)| label == PrincipalDirection::Z && side == Side::Lower)
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

            let ccs = graph.cc();

            println!("nubmer of nodes: {}", nodes.len());
            println!("number of edges: {}", edges.len());

            println!("Found {} connected components", ccs.len());

            // find the cc with our region
            // all segments of this region
            let region_segments = dual.loop_structure.edges(region_id);
            let region_cc = ccs.iter().filter(|&cc| cc.iter().any(|&edge| region_segments.contains(&edge))).collect_vec();
            println!("Found {} connected components with region", region_cc.len());

            let region_cc_full = region_cc.into_iter().filter(|cc| cc.len() > 1).collect_vec();
            assert!(region_cc_full.len() == 1);

            let reachable = region_cc_full[0];

            // filter
            let nodes = nodes.into_iter().filter(|node| reachable.contains(&node)).collect_vec();

            let edges = edges
                .into_iter()
                .filter(|(from, to, _)| reachable.contains(&from) && reachable.contains(&to))
                .collect_vec();

            let graph = Graaf::from(nodes.clone(), edges.clone());

            let mut aux_map = HashMap::new();

            let all_regions = dual.loop_structure.face_ids().into_iter().collect_vec();

            for &segment in reachable {
                let segment_region = dual.loop_structure.face(segment);
                let index_region = all_regions.iter().position(|&r| r == segment_region).unwrap();

                aux_map.insert(graph.node_to_index(&segment).unwrap(), index_region);
            }

            let mut cycles = vec![];
            for segment in region_segments {
                println!("Finding cycles for segment {:?}", segment);
                let mut new_cycles = vec![];
                if let Some(index) = graph.node_to_index(&segment) {
                    new_cycles = graph.all_cycles(index, aux_map.clone());
                }
                println!("Found {} cycles for segment {:?}", new_cycles.len(), segment);

                cycles.extend(new_cycles);
            }

            println!("number of nodes: {}", nodes.len());
            println!("number of edges: {}", edges.len());

            println!("Found {} cycles", cycles.len());

            let cycle_to_edges = cycles
                .iter()
                .map(|cycle| cycle.iter().map(|&edge| graph.index_to_node(edge).unwrap().to_owned()).collect_vec())
                .collect_vec();

            // filter out all cycles with length 2
            let cycle_to_edges = cycle_to_edges.into_iter().filter(|cycle| cycle.len() > 2).collect_vec();

            println!("Found {} cycles length more than 2", cycles.len());

            // filter out all cycles that traverse the same loop region twice
            let cycle_to_edges = cycle_to_edges
                .into_iter()
                .filter(|cycle| {
                    let regions = cycle.iter().map(|&edge| dual.loop_structure.face(edge)).collect_vec();
                    let unique_regions = regions.iter().unique().count();

                    unique_regions * 2 == regions.len()
                })
                .collect_vec();

            println!("Found {} valid cycles", cycle_to_edges.len());

            // find all cycles that contain the region (selected)
            let cycle_to_edges = cycle_to_edges
                .into_iter()
                .filter(|cycle| cycle.iter().any(|&edge| dual.loop_structure.face(edge) == region_id))
                .collect_vec();

            println!("Found {} valid cycles through selected region", cycle_to_edges.len());

            return Some(cycle_to_edges);
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
            if let Some(cycle_to_edges) = self.find_valid_loops_through_region(region_id, direction) {
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
}
