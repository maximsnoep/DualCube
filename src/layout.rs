use crate::dual::{Dual, PropertyViolationError};
use crate::polycube::{Polycube, PolycubeEdgeID, PolycubeFaceID, PolycubeVertID};
use crate::{EmbeddedMesh, FaceID, PrincipalDirection, VertID};
use bimap::BiHashMap;
use douconel::douconel_embedded::HasPosition;
use hutspot::consts::PI;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use rand::seq::SliceRandom;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};

#[derive(Clone, Debug, PartialEq, Eq, Hash, Copy)]
pub enum NodeType {
    Vertex(VertID),
    Face(FaceID),
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Patch {
    // A patch is defined by a set of faces
    pub faces: HashSet<FaceID>,
}

#[derive(Clone, Debug)]
pub struct Layout {
    // TODO: make this an actual (arc) reference
    pub polycube_ref: Polycube,
    // TODO: make this an actual (arc) reference
    pub dual_ref: Dual,

    // mapping:
    pub granulated_mesh: EmbeddedMesh,
    pub vert_to_corner: BiHashMap<PolycubeVertID, VertID>,
    pub edge_to_path: HashMap<PolycubeEdgeID, Vec<VertID>>,
    pub face_to_patch: HashMap<PolycubeFaceID, Patch>,
}

impl Layout {
    pub fn embed(dual_ref: &Dual, polycube_ref: &Polycube) -> Result<Self, PropertyViolationError> {
        let mut layout = Self {
            polycube_ref: polycube_ref.clone(),
            dual_ref: dual_ref.clone(),

            granulated_mesh: (*dual_ref.mesh_ref).clone(),

            vert_to_corner: BiHashMap::new(),
            face_to_patch: HashMap::new(),
            edge_to_path: HashMap::new(),
        };
        layout.place_vertices();
        layout.place_paths()?;
        layout.assign_patches();
        Ok(layout)
    }

    fn place_vertices(&mut self) {
        // Clear the mapping
        self.vert_to_corner.clear();

        // Find a candidate location for each region
        // We simply take the average centroid of all faces in the region, weighted by the area of the faces
        let mut region_to_candidate = HashMap::new();
        for (region_id, region_obj) in &self.dual_ref.loop_structure.faces {
            // Get all vertices in the region
            let verts = &region_obj.verts;

            // Compute for each face its centroid and its area
            let center = verts
                .iter()
                .map(|&vert| {
                    let pos = self.dual_ref.mesh_ref.position(vert);
                    let distance = verts.iter().map(|&v| self.dual_ref.mesh_ref.position(v).metric_distance(&pos)).sum::<f64>();
                    (vert, distance)
                })
                .min_by_key(|(_, distance)| OrderedFloat(*distance))
                .unwrap()
                .0;

            let center_pos = self.dual_ref.mesh_ref.position(center);
            region_to_candidate.insert(region_id, center_pos);
        }

        // Find a candidate location for each zone (only a single coordinate, either X, Y or Z, depending on the type of zone
        // We simply take the coordinate that minimizes the Hausdorf distance to the candidate locations of the regions in the zone
        let mut zone_to_candidate = HashMap::new();
        for (zone_id, zone_obj) in &self.dual_ref.zones {
            let zone_type = zone_obj.direction;

            // Get all coordinates of the regions in the zone
            let candidates = zone_obj
                .regions
                .iter()
                .map(|&region_id| region_to_candidate[&region_id][zone_type as usize])
                .collect_vec();

            // Find the coordinate that minimizes the worst distance to all candidates, do this in N steps
            let n = 5;
            let min = candidates.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();
            let max = candidates.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();
            let steps = (0..n).map(|i| (max - min).mul_add(f64::from(i) / f64::from(n), min)).collect_vec();
            let mut best_step = min;
            let mut best_worst_distance = f64::INFINITY;
            for step in steps {
                let distances: Vec<f64> = candidates.iter().map(|&x| (step - x).abs()).collect();
                let worst_distance = distances.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();
                if worst_distance < best_worst_distance {
                    best_worst_distance = worst_distance;
                    best_step = step;
                }
            }

            zone_to_candidate.insert(zone_id, best_step);
        }

        // Find the actual vertex in the subsurface that is closest to the candidate location (by combining the three candidate coordinates of corresponding zones)
        for (region_id, region_obj) in &self.dual_ref.loop_structure.faces {
            let target = Vector3D::from(
                [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z]
                    .map(|direction| zone_to_candidate[&self.dual_ref.region_to_zone(region_id, direction)]),
            );

            let vertices = &region_obj.verts;
            let best_vertex = vertices
                .iter()
                .map(|&v| (v, self.dual_ref.mesh_ref.position(v)))
                .map(|(v, pos)| (v, pos.metric_distance(&target)))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap()
                .0;

            self.vert_to_corner
                .insert(self.polycube_ref.region_to_vertex.get_by_left(&region_id).unwrap().to_owned(), best_vertex);
        }
    }

    // TODO: Make this robust
    fn place_paths(&mut self) -> Result<(), PropertyViolationError> {
        let primal = &self.polycube_ref;

        let primal_vertices = primal
            .structure
            .vert_ids()
            .iter()
            .map(|&x| self.vert_to_corner.get_by_left(&x).unwrap().to_owned())
            .collect_vec();
        let mut occupied_vertices = HashSet::new();
        let mut occupied_edges = HashSet::new();

        self.edge_to_path = HashMap::new();

        let mut edge_queue = primal.structure.edges.keys().collect_vec();

        edge_queue.shuffle(&mut rand::thread_rng());

        let mut edge_queue = VecDeque::from(edge_queue);

        let mut first_separating_edge = None;
        let mut is_maximal = false;

        let mut counter = 0;

        while let Some(edge_id) = edge_queue.pop_front() {
            //println!("Edge queue: {}", edge_queue.len());

            // if already found (because of twin), skip
            if self.edge_to_path.contains_key(&edge_id) {
                continue;
            }

            if counter > 1000 {
                return Err(PropertyViolationError::UnknownError);
            }
            counter += 1;

            // check if edge is separating (in combination with the edges already done)
            let covered_edges = self.edge_to_path.keys().chain([&edge_id]).collect::<HashSet<_>>();

            let ccs = hutspot::graph::find_ccs(&primal.structure.faces.keys().collect_vec(), |face_id| {
                primal
                    .structure
                    .fneighbors(face_id)
                    .into_iter()
                    .filter(|&n_id| !covered_edges.contains(&primal.structure.edge_between_faces(face_id, n_id).unwrap().0))
                    .collect()
            });

            //println!("cc: {:?} == {:?}", cc.len(), primal.structure.faces.len());

            if !is_maximal && first_separating_edge == Some(edge_id) {
                is_maximal = true;
            }

            if ccs.len() != 1 && !is_maximal {
                // separating edge -> add to the end of the queue
                if first_separating_edge.is_none() {
                    first_separating_edge = Some(edge_id);
                }
                edge_queue.push_back(edge_id);
                continue;
            }

            let (u_new, v_new) = primal.structure.endpoints(edge_id);

            let (u, v) = (
                self.vert_to_corner.get_by_left(&u_new).unwrap().to_owned(),
                self.vert_to_corner.get_by_left(&v_new).unwrap().to_owned(),
            );

            // Find edge in `u_new`
            let edges_done_in_u_new = primal
                .structure
                .outgoing(u_new)
                .into_iter()
                .filter(|&e| self.edge_to_path.contains_key(&e) || e == edge_id)
                .collect_vec();

            let mut blocked_faces = HashSet::new();
            // If this is 3 or larger, this means we must make sure the new edge is placed inbetween existing edges, in the correct order
            if edges_done_in_u_new.len() >= 3 {
                // Find the edge that is "above" the new edge
                let edge_id_position = edges_done_in_u_new.iter().position(|&e| e == edge_id).unwrap();
                let above = (edge_id_position + 1) % edges_done_in_u_new.len();
                let below = (edge_id_position + edges_done_in_u_new.len() - 1) % edges_done_in_u_new.len();
                // find above edge in the granulated mesh
                let above_edge_id = edges_done_in_u_new[above];
                let above_edge_obj = self.edge_to_path.get(&above_edge_id).unwrap();
                let above_edge_start = above_edge_obj[0];
                assert!(above_edge_start == u);
                let above_edge_start_plus_one = above_edge_obj[1];
                let above_edge_real_edge = self.granulated_mesh.edge_between_verts(above_edge_start, above_edge_start_plus_one).unwrap().0;
                // find below edge in the granulated mesh
                let below_edge_id = edges_done_in_u_new[below];
                let below_edge_obj = self.edge_to_path.get(&below_edge_id).unwrap();
                let below_edge_start = below_edge_obj[0];
                assert!(below_edge_start == u);
                let below_edge_start_plus_one = below_edge_obj[1];
                let below_edge_real_edge = self.granulated_mesh.edge_between_verts(below_edge_start, below_edge_start_plus_one).unwrap().0;
                // so starting from below edge, we insert all faces up until the above edge
                let all_edges = self
                    .granulated_mesh
                    .outgoing(u)
                    .into_iter()
                    .flat_map(|e| [e, self.granulated_mesh.twin(e)])
                    .collect_vec();
                let allowed_edges = all_edges
                    .into_iter()
                    .cycle()
                    .skip_while(|&e| e != below_edge_real_edge)
                    .skip(1)
                    .take_while(|&e| e != above_edge_real_edge)
                    .collect_vec();
                let allowed_faces = allowed_edges.into_iter().map(|e| self.granulated_mesh.face(e)).collect_vec();
                assert!(!allowed_faces.is_empty());
                for face_id in self.granulated_mesh.star(u) {
                    if !allowed_faces.contains(&face_id) {
                        blocked_faces.insert(face_id);
                    }
                }
            }

            let twin_id = primal.structure.twin(edge_id);
            // Find edge in `v_new`
            let edges_done_in_v_new = primal
                .structure
                .outgoing(v_new)
                .into_iter()
                .filter(|&e| self.edge_to_path.contains_key(&e) || e == twin_id)
                .collect_vec();

            // If this is 3 or larger, this means we must make sure the new edge is placed inbetween existing edges, in the correct order
            if edges_done_in_v_new.len() >= 3 {
                // Find the edge that is "above" the new edge
                let edge_id_position = edges_done_in_v_new.iter().position(|&e| e == twin_id).unwrap();
                let above = (edge_id_position + 1) % edges_done_in_v_new.len();
                let below = (edge_id_position + edges_done_in_v_new.len() - 1) % edges_done_in_v_new.len();
                // find above edge in the granulated mesh
                let above_edge_id = edges_done_in_v_new[above];
                let above_edge_obj = self.edge_to_path.get(&above_edge_id).unwrap();
                let above_edge_start = above_edge_obj[0];
                assert!(above_edge_start == v);
                let above_edge_start_plus_one = above_edge_obj[1];
                let above_edge_real_edge = self.granulated_mesh.edge_between_verts(above_edge_start, above_edge_start_plus_one).unwrap().0;
                // find below edge in the granulated mesh
                let below_edge_id = edges_done_in_v_new[below];
                let below_edge_obj = self.edge_to_path.get(&below_edge_id).unwrap();
                let below_edge_start = below_edge_obj[0];
                assert!(below_edge_start == v);
                let below_edge_start_plus_one = below_edge_obj[1];
                let below_edge_real_edge = self.granulated_mesh.edge_between_verts(below_edge_start, below_edge_start_plus_one).unwrap().0;
                // so starting from below edge, we insert all faces up until the above edge
                let all_edges = self
                    .granulated_mesh
                    .outgoing(v)
                    .into_iter()
                    .flat_map(|e| [e, self.granulated_mesh.twin(e)])
                    .collect_vec();
                let allowed_edges = all_edges
                    .into_iter()
                    .cycle()
                    .skip_while(|&e| e != below_edge_real_edge)
                    .skip(1)
                    .take_while(|&e| e != above_edge_real_edge)
                    .collect_vec();
                let allowed_faces = allowed_edges.into_iter().map(|e| self.granulated_mesh.face(e)).collect_vec();
                assert!(!allowed_faces.is_empty());
                for face_id in self.granulated_mesh.star(v) {
                    if !allowed_faces.contains(&face_id) {
                        blocked_faces.insert(face_id);
                    }
                }
            }

            let mut blocked_vertices = HashSet::new();
            for &blocked_face in &blocked_faces {
                blocked_vertices.extend(self.granulated_mesh.corners(blocked_face));
            }

            // shortest path from u to v
            let n_function = |node: NodeType| match node {
                NodeType::Face(f_id) => {
                    let f_neighbors: Vec<NodeType> = {
                        // Disallow blocked faces
                        if blocked_faces.contains(&f_id) {
                            return vec![];
                        }
                        // Only allowed if the edge between the two faces is not occupied.
                        let blocked = |f1: FaceID, f2: FaceID| {
                            let (edge_id, _) = self.granulated_mesh.edge_between_faces(f1, f2).unwrap();
                            let (u, v) = self.granulated_mesh.endpoints(edge_id);
                            occupied_edges.contains(&(u, v))
                        };
                        self.granulated_mesh
                            .fneighbors(f_id)
                            .into_iter()
                            .filter(|&n_id| !blocked(f_id, n_id))
                            .map(NodeType::Face)
                            .collect_vec()
                    };
                    let v_neighbors = self.granulated_mesh.corners(f_id).into_iter().map(NodeType::Vertex).collect_vec();
                    [v_neighbors, f_neighbors].concat()
                }
                NodeType::Vertex(v_id) => {
                    // Disallow vertices of blocked faces (unless it is the start or end vertex)
                    if blocked_vertices.contains(&v_id) && v_id != u && v_id != v {
                        return vec![];
                    }
                    // Only allowed if the vertex is not occupied
                    if (occupied_vertices.contains(&v_id) && v_id != u && v_id != v) || (primal_vertices.contains(&v_id) && v_id != u && v_id != v) {
                        return vec![];
                    }
                    let v_neighbors = self.granulated_mesh.vneighbors(v_id).into_iter().map(NodeType::Vertex).collect_vec();
                    let f_neighbors = self.granulated_mesh.star(v_id).into_iter().map(NodeType::Face).collect_vec();
                    [v_neighbors, f_neighbors].concat()
                }
            };
            // neighbors of u using n_function
            let nodetype_to_pos = |node: NodeType| match node {
                NodeType::Face(f_id) => self.granulated_mesh.centroid(f_id),
                NodeType::Vertex(v_id) => self.granulated_mesh.position(v_id),
            };

            let endpoints = self.polycube_ref.structure.endpoints(edge_id);
            let direction_vector = self.polycube_ref.structure.position(endpoints.1) - self.polycube_ref.structure.position(endpoints.0);
            // Should be constant in two of the three directions
            let direction = {
                let (x, y, z) = (direction_vector.x.abs(), direction_vector.y.abs(), direction_vector.z.abs());
                if x > y && x > z {
                    PrincipalDirection::X
                } else if y > x && y > z {
                    PrincipalDirection::Y
                } else {
                    PrincipalDirection::Z
                }
            };

            if direction == PrincipalDirection::X {
                assert!(direction_vector.y < 1e-3 && direction_vector.z < 1e-3);
            } else if direction == PrincipalDirection::Y {
                assert!(direction_vector.x < 1e-3 && direction_vector.z < 1e-3);
            } else {
                assert!(direction_vector.x < 1e-3 && direction_vector.y < 1e-3);
            }

            // ALL EDGES THAT DEVIATE IN THE DIRECTION OF THE EDGE ARE PENALIZED

            let curvature_function = |a_pos: Vector3D, b_pos: Vector3D| {
                let a_to_b = b_pos - a_pos;
                let res = a_to_b.angle(&direction_vector).min(PI - a_to_b.angle(&direction_vector)) + 1.0;
                1.
            };

            let w_function = |a: NodeType, b: NodeType| {
                OrderedFloat(curvature_function(nodetype_to_pos(a), nodetype_to_pos(b)) * nodetype_to_pos(a).metric_distance(&nodetype_to_pos(b)))
            };

            let h_function = |a: NodeType, b: NodeType| OrderedFloat(nodetype_to_pos(a).metric_distance(&nodetype_to_pos(b)));

            let mut granulated_path = vec![];
            if let Some((path, _)) = hutspot::graph::find_shortest_path_astar(NodeType::Vertex(u), NodeType::Vertex(v), n_function, w_function, h_function) {
                let mut last_f_ids_maybe: Option<[FaceID; 3]> = None;
                for node in path {
                    match node {
                        NodeType::Vertex(v_id) => {
                            granulated_path.push((v_id, false));
                            last_f_ids_maybe = None;
                        }
                        NodeType::Face(f_id) => {
                            let new_v_pos = self.granulated_mesh.centroid(f_id);
                            let (new_v_id, new_f_ids) = self.granulated_mesh.split_face(f_id);
                            self.granulated_mesh.verts[new_v_id].set_position(new_v_pos);

                            if let Some(last_f_ids) = last_f_ids_maybe {
                                for last_f_id in last_f_ids {
                                    for new_f_id in new_f_ids {
                                        if let Some((edge_id, _)) = self.granulated_mesh.edge_between_faces(last_f_id, new_f_id) {
                                            let mid_v_pos = self.granulated_mesh.midpoint(edge_id);
                                            let (mid_v_id, _) = self.granulated_mesh.split_edge(edge_id);
                                            self.granulated_mesh.verts[mid_v_id].set_position(mid_v_pos);
                                            granulated_path.push((mid_v_id, false));
                                        }
                                    }
                                }
                            }

                            last_f_ids_maybe = Some(new_f_ids);
                            granulated_path.push((new_v_id, true));
                        }
                    }
                }
            };

            for triple in granulated_path.windows(3) {
                if triple[1].1 {
                    assert!(!triple[0].1 && !triple[2].1);
                    let (v0, v1, v2) = (triple[0].0, triple[1].0, triple[2].0);
                    // Get pos of v0 and v2
                    let (pos0, pos2) = (self.granulated_mesh.position(v0), self.granulated_mesh.position(v2));
                    // Set pos of v1 to be the midpoint of v0 and v2
                    let pos1 = (pos0 + pos2) / 2.0;
                    self.granulated_mesh.verts[v1].set_position(pos1);
                }
            }

            let granulated_path = granulated_path.into_iter().map(|(v_id, _)| v_id).collect_vec();

            if granulated_path.is_empty() {
                return Err(PropertyViolationError::PathEmpty);
            }

            for &v_id in &granulated_path {
                occupied_vertices.insert(v_id);
            }

            for edgepair in granulated_path.windows(2) {
                let (u, v) = (edgepair[0], edgepair[1]);
                occupied_edges.insert((u, v));
                occupied_edges.insert((v, u));
            }
            self.edge_to_path.insert(edge_id, granulated_path.clone());

            // for the twin, we insert the reverse
            let mut rev_path = granulated_path;
            rev_path.reverse();
            self.edge_to_path.insert(primal.structure.twin(edge_id), rev_path);

            counter = 0;
        }
        Ok(())
    }

    pub fn assign_patches(&mut self) {
        // Get all blocked edges (ALL PATHS)
        let blocked = self
            .edge_to_path
            .values()
            .flat_map(|path| path.windows(2))
            .map(|verts| self.granulated_mesh.edge_between_verts(verts[0], verts[1]).unwrap())
            .flat_map(|(a, b)| {
                vec![
                    (self.granulated_mesh.face(a), self.granulated_mesh.face(b)),
                    (self.granulated_mesh.face(b), self.granulated_mesh.face(a)),
                ]
            })
            .collect::<HashSet<_>>();

        // Get all face neighbors, but filter out neighbors blocked by the blocked edges
        let face_to_neighbors = self
            .granulated_mesh
            .faces
            .keys()
            .map(|face_id| {
                (
                    face_id,
                    self.granulated_mesh
                        .fneighbors(face_id)
                        .into_iter()
                        .filter(|&neighbor_id| !blocked.contains(&(face_id, neighbor_id)))
                        .collect_vec(),
                )
            })
            .collect::<HashMap<_, _>>();

        // Find all patches (should be equal to the number of faces in the polycube)
        let patches = hutspot::graph::find_ccs(&self.granulated_mesh.faces.keys().collect_vec(), |face_id: FaceID| {
            face_to_neighbors[&face_id].clone()
        });
        assert!(patches.len() == self.polycube_ref.structure.face_ids().len());

        // Every path should be part of exactly TWO patches (on both sides)
        let mut path_to_ccs: HashMap<PolycubeEdgeID, [usize; 2]> = HashMap::new();
        for (path_id, path) in &self.edge_to_path {
            // Loop segment should simply have only two connected components (one for each side)
            // We do not check all its edges, but only the first one (since they should all be the same)
            let arbitrary_edge = self.granulated_mesh.edge_between_verts(path[0], path[1]).unwrap().0;
            // Edge has two faces
            let (face1, face2) = (
                self.granulated_mesh.face(arbitrary_edge),
                self.granulated_mesh.face(self.granulated_mesh.twin(arbitrary_edge)),
            );

            let cc1 = patches.iter().position(|cc| cc.contains(&face1)).unwrap();
            let cc2 = patches.iter().position(|cc| cc.contains(&face2)).unwrap();
            assert_ne!(cc1, cc2);
            path_to_ccs.insert(*path_id, (cc1, cc2).into());
        }

        // For every patch, get the connected component that is shared among its paths
        for &face_id in &self.polycube_ref.structure.face_ids() {
            let paths = self.polycube_ref.structure.edges(face_id);

            // Select an arbitrary path
            let arbitrary_path = paths[0];
            let [cc1, cc2] = path_to_ccs[&arbitrary_path];

            // Check whether all paths share the same connected component
            let cc1_shared = paths.iter().all(|&path| path_to_ccs[&path].contains(&cc1));
            let cc2_shared = paths.iter().all(|&path| path_to_ccs[&path].contains(&cc2));
            assert!(cc1_shared ^ cc2_shared);

            let faces = if cc1_shared { patches[cc1].clone() } else { patches[cc2].clone() };
            self.face_to_patch.insert(face_id, Patch { faces });
        }
    }

    // Try to improve the layou by doing the following:
    // 1. Change the placement of 1 corner
    // 2. Re-compute paths for this 1 corner (all its adjacent paths)
    // 3. Re-compute patches for this 1 corner (all its adjacent patches)
    pub fn improve(&mut self, corner: PolycubeVertID) -> Result<(), PropertyViolationError> {
        // Change the placement of this corner
        // Get all vertices in the adjacent patches
        // Select a random vertex that is not occupied
        let occupied = self.vert_to_corner.right_values().collect::<HashSet<_>>();
        let adjacent_patches = self.polycube_ref.structure.star(corner);
        let adjacent_patches_vertices = adjacent_patches
            .iter()
            .flat_map(|&patch| self.face_to_patch[&patch].faces.iter())
            .flat_map(|&face| self.granulated_mesh.corners(face))
            .collect::<HashSet<_>>();
        let candidates = adjacent_patches_vertices.into_iter().filter(|v| !occupied.contains(v)).collect_vec();
        let new_vertex = candidates.choose(&mut rand::thread_rng());
        if new_vertex.is_none() {
            return Ok(());
        }
        self.vert_to_corner.insert(corner, new_vertex.unwrap().to_owned());

        // Get all paths that are adjacent to this corner
        let edges = self.polycube_ref.structure.outgoing(corner);
        for edge in edges {
            self.edge_to_path.remove(&edge);
        }

        // Get all patches that are adjacent to this corner
        let faces = self.polycube_ref.structure.star(corner);
        for face in faces {
            self.face_to_patch.remove(&face);
        }

        // TODO: actually recompute only for changed paths..
        self.place_paths()?;

        self.assign_patches();

        Ok(())
    }
}
