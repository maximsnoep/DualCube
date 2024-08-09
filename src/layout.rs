use crate::dual::PropertyViolationError;
use crate::polycube::{Patch, Polycube};
use crate::EmbeddedMesh;
use douconel::douconel::{EdgeID, FaceID, VertID};
use douconel::douconel_embedded::HasPosition;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq, Eq, Hash, Copy)]
pub enum NodeType {
    Vertex(VertID),
    Face(FaceID),
}

#[derive(Clone, Debug, Default)]
pub struct Layout {
    pub mesh_ref: Arc<EmbeddedMesh>,
    pub polycube: Arc<Polycube>,

    // layout part:
    pub granulated_mesh: Option<EmbeddedMesh>,
    // mapping:
    pub face_to_patch: HashMap<FaceID, Patch>,
    pub edge_to_path: HashMap<EdgeID, Vec<VertID>>,
}

impl Layout {
    pub fn new(mesh_ref: Arc<EmbeddedMesh>, polycube: Arc<Polycube>) -> Self {
        Self {
            mesh_ref,
            polycube,
            granulated_mesh: None,
            face_to_patch: HashMap::new(),
            edge_to_path: HashMap::new(),
        }
    }

    pub fn place_paths(&mut self) {
        self.granulated_mesh = Some(self.mesh_ref.as_ref().clone());

        let primal = self.polycube.as_ref();
        let granulated_mesh = self.granulated_mesh.as_mut().unwrap();

        // Start at a random vertex
        let start_vertex = primal.vert_ids()[0];

        let mut spanning_tree_edges = VecDeque::new();

        let mut queue = VecDeque::new();
        queue.push_back(start_vertex);
        let mut visited = HashSet::new();
        visited.insert(start_vertex);

        while let Some(vertex_id) = queue.pop_front() {
            for neighbor_id in primal.vneighbors(vertex_id) {
                if visited.contains(&neighbor_id) {
                    continue;
                }
                let (edge_id, _) = primal.edge_between_verts(vertex_id, neighbor_id).unwrap();
                spanning_tree_edges.push_back(edge_id);
                queue.push_back(neighbor_id);
                visited.insert(neighbor_id);
            }
        }

        for edge_id in primal.edge_ids() {
            if !spanning_tree_edges.contains(&edge_id) {
                spanning_tree_edges.push_back(edge_id);
            }
        }

        let primal_vertices = primal.vert_ids().iter().map(|&x| primal.verts[x].pointer_primal_vertex).collect_vec();
        let mut occupied_vertices = HashSet::new();
        let mut occupied_edges = HashSet::new();

        self.edge_to_path = HashMap::new();

        while let Some(edge_id) = spanning_tree_edges.pop_front() {
            // if already found (because of twin), skip
            if self.edge_to_path.contains_key(&edge_id) {
                continue;
            }

            let (u_new, v_new) = primal.endpoints(edge_id);
            let (u, v) = (primal.verts[u_new].pointer_primal_vertex, primal.verts[v_new].pointer_primal_vertex);

            // Find edge in `u_new`
            let edges_done_in_u_new = primal
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
                let above_edge_real_edge = granulated_mesh.edge_between_verts(above_edge_start, above_edge_start_plus_one).unwrap().0;
                // find below edge in the granulated mesh
                let below_edge_id = edges_done_in_u_new[below];
                let below_edge_obj = self.edge_to_path.get(&below_edge_id).unwrap();
                let below_edge_start = below_edge_obj[0];
                assert!(below_edge_start == u);
                let below_edge_start_plus_one = below_edge_obj[1];
                let below_edge_real_edge = granulated_mesh.edge_between_verts(below_edge_start, below_edge_start_plus_one).unwrap().0;
                // so starting from below edge, we insert all faces up until the above edge
                let all_edges = granulated_mesh.outgoing(u).into_iter().flat_map(|e| [e, granulated_mesh.twin(e)]).collect_vec();
                let allowed_edges = all_edges
                    .into_iter()
                    .cycle()
                    .skip_while(|&e| e != below_edge_real_edge)
                    .skip(1)
                    .take_while(|&e| e != above_edge_real_edge)
                    .collect_vec();
                let allowed_faces = allowed_edges.into_iter().map(|e| granulated_mesh.face(e)).collect_vec();
                assert!(allowed_faces.len() > 0);
                for face_id in granulated_mesh.star(u) {
                    if !allowed_faces.contains(&face_id) {
                        blocked_faces.insert(face_id);
                    }
                }
            }

            let twin_id = primal.twin(edge_id);
            // Find edge in `v_new`
            let edges_done_in_v_new = primal
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
                let above_edge_real_edge = granulated_mesh.edge_between_verts(above_edge_start, above_edge_start_plus_one).unwrap().0;
                // find below edge in the granulated mesh
                let below_edge_id = edges_done_in_v_new[below];
                let below_edge_obj = self.edge_to_path.get(&below_edge_id).unwrap();
                let below_edge_start = below_edge_obj[0];
                assert!(below_edge_start == v);
                let below_edge_start_plus_one = below_edge_obj[1];
                let below_edge_real_edge = granulated_mesh.edge_between_verts(below_edge_start, below_edge_start_plus_one).unwrap().0;
                // so starting from below edge, we insert all faces up until the above edge
                let all_edges = granulated_mesh.outgoing(v).into_iter().flat_map(|e| [e, granulated_mesh.twin(e)]).collect_vec();
                let allowed_edges = all_edges
                    .into_iter()
                    .cycle()
                    .skip_while(|&e| e != below_edge_real_edge)
                    .skip(1)
                    .take_while(|&e| e != above_edge_real_edge)
                    .collect_vec();
                let allowed_faces = allowed_edges.into_iter().map(|e| granulated_mesh.face(e)).collect_vec();
                assert!(allowed_faces.len() > 0);
                for face_id in granulated_mesh.star(v) {
                    if !allowed_faces.contains(&face_id) {
                        blocked_faces.insert(face_id);
                    }
                }
            }

            let mut blocked_vertices = HashSet::new();
            for &blocked_face in &blocked_faces {
                blocked_vertices.extend(granulated_mesh.corners(blocked_face));
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
                            let (edge_id, _) = granulated_mesh.edge_between_faces(f1, f2).unwrap();
                            let (u, v) = granulated_mesh.endpoints(edge_id);
                            occupied_edges.contains(&(u, v))
                        };
                        granulated_mesh
                            .fneighbors(f_id)
                            .into_iter()
                            .filter(|&n_id| !blocked(f_id, n_id))
                            .map(NodeType::Face)
                            .collect_vec()
                    };
                    let v_neighbors = granulated_mesh.corners(f_id).into_iter().map(NodeType::Vertex).collect_vec();
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
                    let v_neighbors = granulated_mesh.vneighbors(v_id).into_iter().map(NodeType::Vertex).collect_vec();
                    let f_neighbors = granulated_mesh.star(v_id).into_iter().map(NodeType::Face).collect_vec();
                    [v_neighbors, f_neighbors].concat()
                }
            };
            // neighbors of u using n_function
            let nodetype_to_pos = |node: NodeType| match node {
                NodeType::Face(f_id) => granulated_mesh.centroid(f_id),
                NodeType::Vertex(v_id) => granulated_mesh.position(v_id),
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
                            let new_v_pos = granulated_mesh.centroid(f_id);
                            let (new_v_id, new_f_ids) = granulated_mesh.split_face(f_id);
                            granulated_mesh.verts[new_v_id].set_position(new_v_pos);

                            if let Some(last_f_ids) = last_f_ids_maybe {
                                for last_f_id in last_f_ids {
                                    for new_f_id in new_f_ids {
                                        if let Some((edge_id, _)) = granulated_mesh.edge_between_faces(last_f_id, new_f_id) {
                                            let mid_v_pos = granulated_mesh.midpoint(edge_id);
                                            let (mid_v_id, _) = granulated_mesh.split_edge(edge_id);
                                            granulated_mesh.verts[mid_v_id].set_position(mid_v_pos);
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
                    let (pos0, pos2) = (granulated_mesh.position(v0), granulated_mesh.position(v2));
                    // Set pos of v1 to be the midpoint of v0 and v2
                    let pos1 = (pos0 + pos2) / 2.0;
                    granulated_mesh.verts[v1].set_position(pos1);
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
            self.edge_to_path.insert(primal.twin(edge_id), rev_path);
        }
    }

    pub fn assign_patches(&mut self) -> Result<(), PropertyViolationError> {
        let polycube = self.polycube.as_ref();
        let granny = self.granulated_mesh.as_mut().unwrap();

        // Get all blocked edges (ALL PATHS)
        let blocked: HashSet<_> = polycube
            .edge_ids()
            .into_iter()
            .flat_map(|path_id| polycube.edges[path_id].windows(2))
            .map(|edgepair| granny.edge_between_verts(edgepair[0], edgepair[1]).unwrap())
            .flat_map(|(a, b)| vec![(granny.face(a), granny.face(b)), (granny.face(b), granny.face(a))])
            .collect();

        let face_neighbors: HashMap<FaceID, Vec<FaceID>> = granny
            .faces
            .keys()
            .map(|face_id: FaceID| {
                (
                    face_id,
                    granny
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
        let ccs = hutspot::graph::find_ccs(&granny.faces.keys().collect_vec(), nfunction);
        if ccs.len() != polycube.face_ids().len() {
            return Err(PropertyViolationError::PatchesMissing);
        }

        // Map from some faces to adjacent paths
        let mut face_to_path: HashMap<FaceID, HashSet<_>> = HashMap::new();
        for (path_id, path) in &polycube.edges {
            for vertexpair in path.windows(2) {
                let edgepair = granny.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap();
                let facepair = (granny.face(edgepair.0), granny.face(edgepair.1));
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
                    .face_ids()
                    .into_iter()
                    .find(|&patch_id| {
                        // check that all paths of this patch are part of the connected component
                        polycube.edges(patch_id).iter().all(|&path_id| paths.contains(&path_id))
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

        if face_and_cc.len() != polycube.face_ids().len() {
            return Err(PropertyViolationError::PatchesMissing);
        }

        for (face_id, patch) in face_and_cc {
            self.face_to_patch.insert(face_id, patch);
        }

        Ok(())
    }
}
