use crate::{
    dual::{to_principal_direction, Dual, IntersectionID, PrincipalDirection, PropertyViolationError},
    graph::Graaf,
    layout::Layout,
    polycube::{Polycube, PolycubeFaceID, PolycubeVertID},
    EdgeID, EmbeddedMesh, FaceID,
};
use hutspot::geom::Vector3D;
use itertools::Itertools;
use log::{info, warn};
use ordered_float::OrderedFloat;
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
    pub polycube: Option<Arc<Polycube>>,
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

    pub fn compute_dual(&mut self) {
        if self.count_loops_in_direction(PrincipalDirection::X) > 0
            && self.count_loops_in_direction(PrincipalDirection::Y) > 0
            && self.count_loops_in_direction(PrincipalDirection::Z) > 0
        {
            let dual = Dual::from(self.mesh_ref.clone(), &self.loops);
            info!("Dual computed. Validity: {:?}", dual.is_ok());
            self.dual = dual;
        } else {
            warn!("Dual not computed. Not enough loops in each direction.");
            self.dual = Err(PropertyViolationError::MissingDirection);
        }
        self.polycube = None;
        self.layout = None;
    }

    pub fn compute_polycube(&mut self) {
        if let Ok(dual) = &self.dual {
            let polycube = Polycube::from_dual(dual);
            info!("Polycube computed.");
            self.polycube = Some(Arc::new(polycube));
        } else {
            warn!("Polycube not computed. Dual not found.");
            self.polycube = None;
        }
        self.layout = None;
    }

    pub fn compute_layout(&mut self) {
        if let (Ok(dual), Some(polycube)) = (&self.dual, &self.polycube) {
            for _ in 0..10 {
                let layout = Layout::embed(dual, polycube);
                if let Ok(ok_layout) = &layout {
                    let (score_a, score_b) = ok_layout.score();
                    let score = score_a + score_b;
                    println!("Found: {score_a:?} + {score_b:?} = {score:?}");
                    if score_a < 0.1 {
                        self.layout = Some(layout);
                        return;
                    }
                }
            }
            self.layout = None;
        } else {
            warn!("Layout not computed. Dual and/or polycube not found.");
            self.layout = None;
        }
    }

    pub fn compute_alignment(&mut self) {
        self.alignment_per_triangle.clear();
        self.alignment = None;

        if let (Some(Ok(layout)), Some(polycube)) = (&self.layout, &self.polycube) {
            let total_area = layout.granulated_mesh.faces.keys().map(|f| layout.granulated_mesh.area(f)).sum::<f64>();
            let mut total_score = 0.0;

            for &patch in layout.face_to_patch.keys() {
                let mapped_normal = (polycube.structure.normal(patch) as Vector3D).normalize();
                for &triangle_id in &layout.face_to_patch[&patch].faces {
                    let normal = layout.granulated_mesh.normal(triangle_id);
                    let angle = normal.angle(&mapped_normal);
                    let score = 1. - (1. + f64::exp(std::f64::consts::PI.mul_add(2., -(4. * angle)))).powi(-1);
                    self.alignment_per_triangle.insert(triangle_id, score);
                    let area = layout.granulated_mesh.area(triangle_id);
                    let normalized_score = score * (area / total_area);
                    total_score += normalized_score;
                }
            }

            println!("Total alignment score: {total_score:?}");
            self.alignment = Some(total_score);
        }
    }

    pub fn compute_orthogonality(&mut self) {
        if let (Some(Ok(layout)), Some(polycube)) = (&self.layout, &self.polycube) {
            let mut total_score = 0.0;
            let total_area = layout.granulated_mesh.faces.keys().map(|f| layout.granulated_mesh.area(f)).sum::<f64>();

            for &patch in layout.face_to_patch.keys() {
                let mut patch_orthogonality = 0.0;
                for patch_edge in polycube.structure.edges(patch) {
                    let path = &layout.edge_to_path[&patch_edge];
                    let next_path = &layout.edge_to_path[&polycube.structure.next(patch_edge)];

                    let a = path.first().unwrap().to_owned();
                    let b = path.last().unwrap().to_owned();
                    assert!(*next_path.first().unwrap() == b);
                    let c = next_path.last().unwrap().to_owned();

                    let a_pos = layout.granulated_mesh.position(a);
                    let b_pos = layout.granulated_mesh.position(b);
                    let c_pos = layout.granulated_mesh.position(c);

                    let ba = a_pos - b_pos;
                    let bc = c_pos - b_pos;
                    let angle = ba.angle(&bc);
                    let score = angle.sin().powi(2);
                    patch_orthogonality += score;
                }
                let patch_score = patch_orthogonality / polycube.structure.edges(patch).len() as f64;

                let patch_area = layout.face_to_patch[&patch].faces.iter().map(|&f| layout.granulated_mesh.area(f)).sum::<f64>();
                let normalized_score = patch_score * (patch_area / total_area);
                self.orthogonality_per_patch.insert(patch, patch_score);
                total_score += normalized_score;
            }

            println!("Total orthogonality score: {total_score:?}");
            self.orthogonality = Some(total_score);
        }
    }

    pub fn mutate_add_loop(&self, nr_loops: usize, alpha: i32, beta: i32, flow_graphs: &[Graaf<[EdgeID; 2], (f64, f64, f64)>; 3]) -> Self {
        let mut solutions = vec![];

        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            for _ in 0..nr_loops {
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
                    println!("Path is empty and/or invalid.");
                    continue;
                }

                let mut real_solution = self.clone();

                real_solution.add_loop(Loop {
                    edges: best_option,
                    direction: direction,
                });

                real_solution.compute_dual();
                real_solution.compute_polycube();
                real_solution.compute_layout();
                real_solution.compute_alignment();
                real_solution.compute_orthogonality();
                if real_solution.dual.is_ok() && real_solution.polycube.is_some() && real_solution.layout.is_some() {
                    solutions.push(real_solution);
                }
            }
        }

        let best_solution = solutions
            .into_iter()
            .max_by_key(|solution| OrderedFloat(solution.alignment.unwrap_or_default()))
            .unwrap();

        return best_solution;
    }
}
