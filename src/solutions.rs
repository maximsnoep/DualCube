use crate::{
    dual::{Dual, IntersectionID, PrincipalDirection, PropertyViolationError, ZoneID},
    graph::Graaf,
    layout::Layout,
    polycube::{Polycube, PolycubeFaceID, PolycubeVertID},
    EdgeID, EmbeddedMesh, FaceID,
};
use douconel::douconel_embedded::HasPosition;
use hutspot::{geom::Vector3D, timer::Timer};
use itertools::Itertools;
use ordered_float::OrderedFloat;
use rand::seq::{IteratorRandom, SliceRandom};
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use serde::{Deserialize, Serialize};
use slotmap::{SecondaryMap, SlotMap};
use std::{collections::HashSet, sync::Arc};

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

        for [e0, e1] in self.get_pairs_of_loop(loop_id) {
            if let Some(l) = self.is_occupied([e0, e1]) {
                assert!(l == loop_id, "Loop: {loop_id:?} already occupied by {l:?} on edge {:?}", [e0, e1]);
            }
        }

        loop_id
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

    pub fn reconstruct_solution(&mut self) {
        self.compute_dual();
        self.compute_polycube();
        self.compute_layout();
        println!("Smoothening done...");
        println!("Resizing polycube...");

        if let Some(Ok(layout)) = &mut self.layout {
            layout.smoothening();
        }
        println!("Done with smoothening (real)");

        // self.resize_polycube();
        // println!("Resizing done...");
        // println!("Computing alignment...");
        // self.compute_alignment();
        // println!("Alignment done...");
        // println!("Computing orthogonality...");
        // self.compute_orthogonality();
        // println!("Orthogonality done...");
    }

    pub fn compute_dual(&mut self) {
        if self.count_loops_in_direction(PrincipalDirection::X) > 0
            && self.count_loops_in_direction(PrincipalDirection::Y) > 0
            && self.count_loops_in_direction(PrincipalDirection::Z) > 0
        {
            let dual = Dual::from(self.mesh_ref.clone(), &self.loops);
            self.dual = dual;
        } else {
            self.dual = Err(PropertyViolationError::MissingDirection);
        }
        self.polycube = None;
        self.layout = None;
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

    pub fn compute_layout(&mut self) {
        self.layout = None;
        if let (Ok(dual), Some(polycube)) = (&self.dual, &self.polycube) {
            for _ in 0..5 {
                let layout = Layout::embed(dual, polycube);
                match &layout {
                    Ok(ok_layout) => {
                        let (score_a, score_b) = ok_layout.score();
                        // let score = score_a + score_b;
                        // if score_a < 0.1 {
                        self.layout = Some(layout);
                        return;
                        // }
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
                    let score = 1. - (1. + std::f64::consts::PI.mul_add(2., -(4. * actual_normal.angle(&mapped_normal))).exp()).recip();
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
                let direction = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z]
                    .choose(&mut rand::thread_rng())
                    .unwrap()
                    .to_owned();

                let timer = Timer::new();

                if let Some(new_loop) = self.construct_loop(direction, flow_graphs) {
                    let mut new_solution = self.clone();
                    new_solution.add_loop(Loop { edges: new_loop, direction });
                    new_solution.reconstruct_solution();

                    if new_solution.dual.is_ok() && new_solution.polycube.is_some() && new_solution.layout.is_some() {
                        timer.report(&format!(
                            "Found solution\t[alignment: {:.2}]\t[orthogonality: {:.2}]",
                            new_solution.alignment.unwrap(),
                            new_solution.orthogonality.unwrap()
                        ));
                        return Some(new_solution);
                    }
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
        let alpha = rand::random::<i32>() % 10 + 5;
        let beta = rand::random::<i32>() % 10 + 5;

        let measure = |(a, b, c): (f64, f64, f64)| a.powi(alpha) + b.powi(beta) + c.powi(beta);

        // Starting edges.
        let e1 = self.mesh_ref.random_edges(1).first().unwrap().to_owned();
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
