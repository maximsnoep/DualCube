use crate::dual::{to_principal_direction, Dual, PrincipalDirection, PropertyViolationError};
use crate::polycube::{Polycube, PolycubeEdgeID, PolycubeFaceID, PolycubeVertID};
use crate::{EmbeddedMesh, FaceID, VertID};
use bimap::BiHashMap;
use douconel::douconel_embedded::HasPosition;
use hutspot::consts::PI;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use log::warn;
use ordered_float::OrderedFloat;
use rand::seq::SliceRandom;
use serde::{Deserialize, Serialize};
use std::cmp::Reverse;
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

#[derive(Clone, Debug, Default)]
pub struct Layout {
    pub polycube_ref: Polycube,

    // mapping:
    pub granulated_mesh: EmbeddedMesh,
    pub vert_to_corner: BiHashMap<PolycubeVertID, VertID>,
    pub edge_to_path: HashMap<PolycubeEdgeID, Vec<VertID>>,
    pub face_to_patch: HashMap<PolycubeFaceID, Patch>,
}

impl Layout {
    pub fn embed(dual_ref: &Dual, polycube_ref: &Polycube, smoothen: bool) -> Result<Self, PropertyViolationError<PolycubeVertID>> {
        let mut layout = Self {
            polycube_ref: polycube_ref.clone(),
            granulated_mesh: (*dual_ref.mesh_ref).clone(),

            vert_to_corner: BiHashMap::new(),
            face_to_patch: HashMap::new(),
            edge_to_path: HashMap::new(),
        };
        layout.place_vertices(dual_ref)?;
        layout.place_paths()?;
        if smoothen {
            layout.smoothening();
        }
        layout.assign_patches()?;
        Ok(layout)
    }

    fn place_vertices(&mut self, dual: &Dual) -> Result<(), PropertyViolationError<PolycubeVertID>> {
        // find the best candidate position for each region
        let mut region_to_candidate = HashMap::new();
        for (region_id, region_obj) in &dual.loop_structure.faces {
            // Naive labeling of the vertices in the region
            let naive_labeling = region_obj.verts.iter().map(|&v| {
                let p = dual.mesh_ref.position(v);
                let n = dual.mesh_ref.vert_normal(v);
                let (d, s) = to_principal_direction(n);
                (p, (d, s))
            });

            let boundary = dual
                .loop_structure
                .corners(region_id)
                .iter()
                .map(|&intersection_id| {
                    let polycube_face = self.polycube_ref.intersection_to_face.get_by_left(&intersection_id).unwrap().to_owned();
                    let face_normal = self.polycube_ref.structure.normal(polycube_face);
                    let (d, s) = to_principal_direction(face_normal);
                    (d, s)
                })
                .collect_vec();

            // find the vertex, that minimizes the worst distance to each

            let boundary_average = boundary
                .iter()
                .filter_map(|(d, s)| {
                    let filtered_verts = naive_labeling.clone().filter(|(_, (d2, s2))| d == d2 && s == s2);
                    let res = hutspot::math::calculate_average_f64(filtered_verts.map(|(p, _)| p));
                    if res[0].is_nan() || res[1].is_nan() || res[2].is_nan() {
                        return None;
                    }
                    Some(res)
                })
                .collect_vec();

            if boundary_average.is_empty() {
                return Err(PropertyViolationError::UnknownError);
            }

            let region_average = hutspot::math::calculate_average_f64(boundary_average.clone().into_iter());

            region_to_candidate.insert(region_id, region_average);
        }

        // for each zone, find a coordinate that minimizes Hausdorf distance to each subsurface in the zone
        let mut zone_to_coordinate = HashMap::new();
        for (zone_id, zone_obj) in &dual.zones {
            let direction_of_zone = zone_obj.direction;

            // get all DIRECTION coordinates of the vertices of the subsurfaces in this zone
            let subsurface_coordinates = zone_obj
                .regions
                .iter()
                .map(|&region_id| region_to_candidate[&region_id][direction_of_zone as usize])
                .collect_vec();

            // get min and max
            let min = subsurface_coordinates.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();
            let max = subsurface_coordinates.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();

            // find the coordinate that minimizes the worst distance to all subsurfaces
            let mut best_coordinate = min;
            let mut best_worst_distance = f64::INFINITY;
            for x in (0..10).map(|i| min + (max - min) * (i as f64) / 10.0) {
                let distance_to_each_subsurface = subsurface_coordinates.iter().map(|&y| (x - y).abs());
                let worst_distance = distance_to_each_subsurface.max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
                if worst_distance < best_worst_distance {
                    best_worst_distance = worst_distance;
                    best_coordinate = x;
                }
            }

            zone_to_coordinate.insert(zone_id, best_coordinate);
        }

        for vert_id in self.polycube_ref.structure.vert_ids() {
            let region_id = self.polycube_ref.region_to_vertex.get_by_right(&vert_id).unwrap().to_owned();
            let coordinates = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z]
                .map(|d| zone_to_coordinate[&dual.zones.iter().find(|(_, z)| z.regions.contains(&region_id) && z.direction == d).unwrap().0]);
            let target = Vector3D::new(coordinates[0], coordinates[1], coordinates[2]);

            let vertices_in_subsurface = dual.loop_structure.faces[region_id].verts.clone();
            let best_vertex_in_subsurface = vertices_in_subsurface
                .iter()
                .map(|&v| (v, dual.mesh_ref.position(v)))
                .min_by(|(_, a), (_, b)| (a - target).norm().partial_cmp(&(b - target).norm()).unwrap())
                .unwrap()
                .0;

            self.vert_to_corner.insert(vert_id, best_vertex_in_subsurface);
        }

        Ok(())
    }

    fn place_paths(&mut self) -> Result<(), PropertyViolationError<PolycubeVertID>> {
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
            if counter > 10000 {
                return Err(PropertyViolationError::UnknownError);
            }
            counter += 1;
            //println!("Edge queue: {}", edge_queue.len());

            // if already found (because of twin), skip
            if self.edge_to_path.contains_key(&edge_id) {
                continue;
            }

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

            let w_function = |a: NodeType, b: NodeType| OrderedFloat(nodetype_to_pos(a).metric_distance(&nodetype_to_pos(b)));

            let mut granulated_path = vec![];
            if let Some((path, _)) = hutspot::graph::find_shortest_path_astar(NodeType::Vertex(u), NodeType::Vertex(v), n_function, w_function, w_function) {
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

            let lowerbound_path = hutspot::graph::find_shortest_path_astar(
                u,
                v,
                |a| self.granulated_mesh.vneighbors(a),
                |a, b| ordered_float::OrderedFloat(self.granulated_mesh.distance(a, b)),
                |a, b| ordered_float::OrderedFloat(self.granulated_mesh.distance(a, b)),
            )
            .unwrap()
            .0;
            let lowerbound_score = lowerbound_path
                .windows(2)
                .map(|pair| self.granulated_mesh.distance(pair[0], pair[1]))
                .sum::<f64>();

            let actual_score = granulated_path
                .windows(2)
                .map(|pair| self.granulated_mesh.distance(pair[0], pair[1]))
                .sum::<f64>();

            let ratio = (actual_score / lowerbound_score) - 1.0;
            if ratio > 0.5 {
                return Err(PropertyViolationError::PathTooLong);
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
        }
        Ok(())
    }

    pub fn assign_patches(&mut self) -> Result<(), PropertyViolationError<PolycubeVertID>> {
        let polycube = &self.polycube_ref;

        // Get all blocked edges (ALL PATHS)
        let blocked = self
            .edge_to_path
            .values()
            .flat_map(|path| path.windows(2))
            .map(|edgepair| {
                self.granulated_mesh
                    .edge_between_verts(edgepair[0], edgepair[1])
                    .unwrap_or_else(|| panic!("Edge must exist: {edgepair:?}"))
            })
            .flat_map(|(a, b)| {
                vec![
                    (self.granulated_mesh.face(a), self.granulated_mesh.face(b)),
                    (self.granulated_mesh.face(b), self.granulated_mesh.face(a)),
                ]
            })
            .collect::<HashSet<_>>();

        // Get all face neighbors, but filter out neighbors blocked by the blocked edges
        let face_neighbors: HashMap<FaceID, Vec<FaceID>> = self
            .granulated_mesh
            .faces
            .keys()
            .map(|face_id: FaceID| {
                (
                    face_id,
                    self.granulated_mesh
                        .fneighbors(face_id)
                        .into_iter()
                        .filter(|&neighbor_id| !blocked.contains(&(face_id, neighbor_id)))
                        .collect(),
                )
            })
            .collect();

        // Make a neighborhood function that blocks all edges that are part of the loop segments of this face
        let nfunction = |face_id: FaceID| face_neighbors[&face_id].clone();

        // Find all connected components (should be equal to the number of faces)
        let ccs = hutspot::graph::find_ccs(&self.granulated_mesh.faces.keys().collect_vec(), nfunction);
        if ccs.len() != polycube.structure.face_ids().len() {
            warn!(
                "Number of faces ({:?}) must be equal to number of connected components ({:?})",
                polycube.structure.face_ids().len(),
                ccs.len()
            );
            return Err(PropertyViolationError::PatchesMissing);
        }

        // Map from some faces to adjacent paths
        let mut face_to_path: HashMap<FaceID, HashSet<_>> = HashMap::new();
        for (path_id, path) in &self.edge_to_path {
            for vertexpair in path.windows(2) {
                let edgepair = self.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap();
                let facepair = (self.granulated_mesh.face(edgepair.0), self.granulated_mesh.face(edgepair.1));
                face_to_path.entry(facepair.0).or_default().insert(path_id);
                face_to_path.entry(facepair.1).or_default().insert(path_id);
            }
        }

        // For each connected component, assign a patch
        // We can find the correct patch, by checking which paths are part of the connected component
        // So first map each connected component, to the paths
        let face_and_cc = ccs
            .into_iter()
            .map(|cc| {
                (
                    cc.clone(),
                    cc.into_iter()
                        .flat_map(|face_id| face_to_path.get(&face_id).cloned().unwrap_or_default())
                        .collect::<HashSet<_>>(),
                )
            })
            .filter_map(|(cc, paths)| {
                polycube
                    .structure
                    .face_ids()
                    .into_iter()
                    .find(|&patch_id| {
                        // check that all paths of this patch are part of the connected component
                        polycube.structure.edges(patch_id).iter().all(|&path_id| paths.contains(&path_id))
                    })
                    .map(|face_id| (face_id, cc))
            })
            .map(|(face_id, subsurface)| {
                (
                    face_id,
                    Patch {
                        faces: subsurface.into_iter().collect(),
                    },
                )
            })
            .collect_vec();

        if face_and_cc.len() != polycube.structure.face_ids().len() {
            warn!(
                "Number of faces ({:?}) must be equal to number of patches ({:?})",
                polycube.structure.face_ids().len(),
                face_and_cc.len()
            );
            return Err(PropertyViolationError::PatchesMissing);
        }

        for (face_id, patch) in face_and_cc {
            self.face_to_patch.insert(face_id, patch);
        }

        Ok(())
    }

    pub fn score(&self) -> (f64, f64) {
        let total_area = self
            .granulated_mesh
            .faces
            .keys()
            .map(|face_id| self.granulated_mesh.vector_area(face_id).norm())
            .sum::<f64>();
        let weighted_angle = self
            .face_to_patch
            .iter()
            .flat_map(|(polycube_face_id, patch)| {
                patch.faces.iter().map(|&face_id| {
                    let target_normal = self.polycube_ref.structure.normal(*polycube_face_id);
                    let face_area = self.granulated_mesh.vector_area(face_id).norm();
                    let face_normal = self.granulated_mesh.normal(face_id);
                    let angle = (target_normal.angle(&face_normal) / 2.).sin();
                    (face_area / total_area) * angle
                })
            })
            .sum::<f64>();

        let paths_lowerbound = self
            .edge_to_path
            .iter()
            .map(|(edge_id, actual_path)| {
                let (u, v) = self.polycube_ref.structure.endpoints(*edge_id);
                let (u_vert, v_vert) = (
                    self.vert_to_corner.get_by_left(&u).unwrap().to_owned(),
                    self.vert_to_corner.get_by_left(&v).unwrap().to_owned(),
                );
                let lowerbound_path = hutspot::graph::find_shortest_path_astar(
                    u_vert,
                    v_vert,
                    |a| self.granulated_mesh.vneighbors(a),
                    |a, b| ordered_float::OrderedFloat(self.granulated_mesh.distance(a, b)),
                    |a, b| ordered_float::OrderedFloat(self.granulated_mesh.distance(a, b)),
                )
                .unwrap()
                .0;
                let lowerbound_score = lowerbound_path
                    .windows(2)
                    .map(|pair| self.granulated_mesh.distance(pair[0], pair[1]))
                    .sum::<f64>();

                let actual_score = actual_path.windows(2).map(|pair| self.granulated_mesh.distance(pair[0], pair[1])).sum::<f64>();

                let ratio = (actual_score / lowerbound_score) - 1.0;
                ratio * (1. / self.edge_to_path.len() as f64)
            })
            .sum::<f64>();

        (paths_lowerbound, weighted_angle)
    }

    pub fn smoothening(&mut self) {
        // We smoothen each path until path is geodesic
        // https://dl.acm.org/doi/pdf/10.1145/3414685.3417839

        let keys = self.edge_to_path.keys().copied().collect_vec();
        for path_id in keys {
            // Calculate alpha for each vertex in the path.
            // A path is geodesic if alpha is > 180 degrees (pi) for each vertex
            // Any vertex with alpha < 180 degrees is a candidate for smoothing
            let mut wedges = priority_queue::PriorityQueue::new();

            //
            fn calculate_alpha(a: VertID, b: VertID, c: VertID, mesh: &EmbeddedMesh) -> f64 {
                let face = mesh.face_with_verts(&[a, b, c]).unwrap();
                let normal = mesh.normal(face);
                let (a_pos, b_pos, c_pos) = (mesh.position(a), mesh.position(b), mesh.position(c));
                hutspot::geom::calculate_clockwise_angle(b_pos, a_pos, c_pos, normal)
            }

            // Every vertex is defined by a wedge of 3 vertices
            let path = self.edge_to_path.get(&path_id).unwrap();
            for pair in path.windows(3) {
                // First we find the correct side (shortest angle/alpha) of the wedge
                // There are two wedges between a, b, c
                let (a, b, c) = (pair[0], pair[1], pair[2]);
                let (wedge1, wedge2) = self.granulated_mesh.wedges(a, b, c);

                if self.granulated_mesh.distance(a, b) < 1e-3 || self.granulated_mesh.distance(b, c) < 1e-3 {
                    continue;
                }

                let alpha_wedge1 = wedge1
                    .clone()
                    .into_iter()
                    .tuple_windows()
                    .map(|(v1, v2)| calculate_alpha(v1, b, v2, &self.granulated_mesh))
                    .sum::<f64>();
                let alpha_wedge2 = wedge2
                    .clone()
                    .into_iter()
                    .tuple_windows()
                    .map(|(v1, v2)| calculate_alpha(v1, b, v2, &self.granulated_mesh))
                    .sum::<f64>();

                if alpha_wedge1 < alpha_wedge2 {
                    wedges.push((wedge1, b, false), Reverse(OrderedFloat(alpha_wedge1)));
                } else {
                    wedges.push((wedge2, b, true), Reverse(OrderedFloat(alpha_wedge2)));
                }
            }

            if let Some(worst_wedge) = wedges.peek() {
                if worst_wedge.1 .0.abs() < PI * 0.85 {
                    println!("Smoothing path {:?} (worst wedge: {:?})", path_id, worst_wedge.1);

                    while let Some(((mut wedge, b, reverse), cost)) = wedges.pop() {
                        let path = self.edge_to_path.get(&path_id).unwrap();
                        if !path.contains(&wedge[0]) || !path.contains(&wedge[wedge.len() - 1]) {
                            continue;
                        }

                        // Now we apply flips on the wedge
                        let mut changed = true;
                        while changed {
                            changed = false;
                            for i in 1..wedge.len() - 1 {
                                let (ni_prev, ni_cur, ni_next) = (wedge[i - 1], wedge[i], wedge[i + 1]);
                                let angle =
                                    calculate_alpha(ni_next, ni_cur, b, &self.granulated_mesh) + calculate_alpha(b, ni_cur, ni_prev, &self.granulated_mesh);
                                println!("{i}: Angle: {angle:?}");
                                if angle < 0.85 * std::f64::consts::PI
                                    && self.granulated_mesh.distance(ni_prev, ni_cur) > 1e-3
                                    && self.granulated_mesh.distance(ni_cur, ni_next) > 1e-3
                                {
                                    // We flip the triangle
                                    if let Some(new_v) = self.granulated_mesh.splip_edge(ni_cur, b) {
                                        wedge[i] = new_v;
                                        changed = new_v != ni_cur && new_v != b;
                                    }
                                }
                            }
                        }

                        // Now we update the path
                        if reverse {
                            wedge.reverse();
                        }

                        println!(" adding wedge: {:?}", wedge);

                        let path = self.edge_to_path.get(&path_id).unwrap().clone();

                        println!(" path id: {:?}", path_id);
                        println!(" path: {:?}", path);

                        for pair in path.windows(2) {
                            // does the edge exist?
                            let edgepair = self.granulated_mesh.edge_between_verts(pair[0], pair[1]);
                            assert!(edgepair.is_some(), "Edge not found: {:?} {:?} path id: {:?}", pair[0], pair[1], path_id);
                        }

                        let path_before_wedge = path.iter().take_while(|&&v| v != *wedge.first().unwrap()).copied().collect_vec();
                        let path_after_wedge = path.iter().skip_while(|&&v| v != *wedge.last().unwrap()).skip(1).copied().collect_vec();

                        let new_path = path_before_wedge.into_iter().chain(wedge).chain(path_after_wedge.into_iter()).collect_vec();

                        let old_length = path.windows(2).map(|pair| self.granulated_mesh.distance(pair[0], pair[1])).sum::<f64>();
                        let new_length = new_path.windows(2).map(|pair| self.granulated_mesh.distance(pair[0], pair[1])).sum::<f64>();

                        println!("Old length: {:?}, new length: {:?}", old_length, new_length);

                        self.edge_to_path.insert(path_id, new_path.clone());
                        self.edge_to_path
                            .insert(self.polycube_ref.structure.twin(path_id), new_path.iter().rev().copied().collect_vec());
                    }

                    return;
                }
            }
        }
    }
}
