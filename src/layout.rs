use crate::dual::{Dual, PrincipalDirection, PropertyViolationError};
use crate::polycube::{Polycube, PolycubeEdgeID, PolycubeFaceID, PolycubeVertID};
use crate::{EmbeddedMesh, FaceID, VertID};
use bimap::BiHashMap;
use douconel::douconel_embedded::HasPosition;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

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
    pub polycube_ref: Arc<Polycube>,

    // mapping:
    pub granulated_mesh: EmbeddedMesh,
    pub vert_to_corner: BiHashMap<PolycubeVertID, VertID>,
    pub edge_to_path: HashMap<PolycubeEdgeID, Vec<VertID>>,
    pub face_to_patch: HashMap<PolycubeFaceID, Patch>,
}

impl Layout {
    pub fn embed(dual_ref: &Dual, polycube_ref: &Arc<Polycube>) -> Result<Self, PropertyViolationError<PolycubeVertID>> {
        let mut layout = Self {
            polycube_ref: polycube_ref.clone(),
            granulated_mesh: (*dual_ref.mesh_ref).clone(),
            vert_to_corner: BiHashMap::new(),
            face_to_patch: HashMap::new(),
            edge_to_path: HashMap::new(),
        };

        layout.place_vertices(dual_ref);
        layout.place_paths();
        layout.assign_patches()?;
        Ok(layout)
    }

    fn place_vertices(&mut self, dual: &Dual) {
        // for each zone, find a coordinate that minimizes Hausdorf distance to each subsurface in the zone
        let mut zone_to_coordinate = HashMap::new();
        for (zone_id, zone_obj) in &dual.zones {
            let direction_of_zone = zone_obj.direction;
            let subsurfaces_in_this_zone = zone_obj.regions.clone();

            // get all DIRECTION coordinates of the vertices of the subsurfaces in this zone
            let coordinates = subsurfaces_in_this_zone
                .iter()
                .map(|&subsurface_id| {
                    dual.loop_structure.faces[subsurface_id]
                        .verts
                        .iter()
                        .map(|&vert_id| dual.mesh_ref.position(vert_id)[direction_of_zone as usize])
                        .collect_vec()
                })
                .collect_vec();

            // get min and max
            let min = coordinates.iter().flatten().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();
            let max = coordinates.iter().flatten().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap().to_owned();

            // find the coordinate that minimizes the worst distance to all subsurfaces
            let mut best_coordinate = min;
            let mut best_distance = f64::INFINITY;
            for x in (0..100).map(|i| min + (max - min) * (i as f64) / 100.0) {
                let smallest_distance_to_each_subsurface = coordinates
                    .iter()
                    .map(|c| c.iter().map(|&y| (x - y).abs()).min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap());
                let distance = smallest_distance_to_each_subsurface.max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
                if distance < best_distance {
                    best_distance = distance;
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
    }

    fn place_paths(&mut self) {
        let primal = self.polycube_ref.as_ref();

        // Start at a random vertex
        let start_vertex = primal.structure.vert_ids()[0];

        let mut spanning_tree_edges = VecDeque::new();

        let mut queue = VecDeque::new();
        queue.push_back(start_vertex);
        let mut visited = HashSet::new();
        visited.insert(start_vertex);

        while let Some(vertex_id) = queue.pop_front() {
            for neighbor_id in primal.structure.vneighbors(vertex_id) {
                if visited.contains(&neighbor_id) {
                    continue;
                }
                let (edge_id, _) = primal.structure.edge_between_verts(vertex_id, neighbor_id).unwrap();
                spanning_tree_edges.push_back(edge_id);
                queue.push_back(neighbor_id);
                visited.insert(neighbor_id);
            }
        }

        for edge_id in primal.structure.edge_ids() {
            if !spanning_tree_edges.contains(&edge_id) {
                spanning_tree_edges.push_back(edge_id);
            }
        }

        let primal_vertices = primal
            .structure
            .vert_ids()
            .iter()
            .map(|&x| self.vert_to_corner.get_by_left(&x).unwrap().to_owned())
            .collect_vec();
        let mut occupied_vertices = HashSet::new();
        let mut occupied_edges = HashSet::new();

        self.edge_to_path = HashMap::new();

        while let Some(edge_id) = spanning_tree_edges.pop_front() {
            // if already found (because of twin), skip
            if self.edge_to_path.contains_key(&edge_id) {
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

            let w_function = |a: NodeType, b: NodeType| {
                let a_pos = nodetype_to_pos(a);
                let b_pos = nodetype_to_pos(b);
                OrderedFloat(a_pos.metric_distance(&b_pos))
            };

            let mut granulated_path = vec![];
            if let Some((path, _)) = hutspot::graph::find_shortest_path(NodeType::Vertex(u), NodeType::Vertex(v), n_function, w_function) {
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

            //println!("granulated path: {granulated_path:?}");

            if granulated_path.is_empty() {
                println!("!\n!\n!\nEmpty path found\n!\n!\n");
                break;
            }

            for &v_id in &granulated_path {
                occupied_vertices.insert(v_id);
            }

            for edgepair in granulated_path.windows(2) {
                let (u, v) = (edgepair[0], edgepair[1]);
                occupied_edges.insert((u, v));
                occupied_edges.insert((v, u));
            }

            // if granulated_path.is_empty() {
            //     panic!("Empty path found");
            // }

            self.edge_to_path.insert(edge_id, granulated_path.clone());

            // for the twin, we insert the reverse
            let mut rev_path = granulated_path;
            rev_path.reverse();
            self.edge_to_path.insert(primal.structure.twin(edge_id), rev_path);
        }
    }

    fn assign_patches(&mut self) -> Result<(), PropertyViolationError<PolycubeVertID>> {
        let polycube = self.polycube_ref.as_ref();
        // Get all blocked edges (ALL PATHS)

        let blocked = self
            .edge_to_path
            .values()
            .flat_map(|path| path.windows(2))
            .map(|edgepair| self.granulated_mesh.edge_between_verts(edgepair[0], edgepair[1]).unwrap())
            .flat_map(|(a, b)| {
                vec![
                    (self.granulated_mesh.face(a), self.granulated_mesh.face(b)),
                    (self.granulated_mesh.face(b), self.granulated_mesh.face(a)),
                ]
            })
            .collect::<HashSet<_>>();

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
            println!("{}, {} ", ccs.len(), polycube.structure.face_ids().len());
            println!("here... 999");
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
            println!("here... 123");
            return Err(PropertyViolationError::PatchesMissing);
        }

        for (face_id, patch) in face_and_cc {
            self.face_to_patch.insert(face_id, patch);
        }

        Ok(())
    }
}
