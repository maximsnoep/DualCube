#![warn(clippy::all, clippy::pedantic, clippy::nursery, clippy::cargo)]

use bevy::prelude::*;
use douconel::douconel::Douconel;
use douconel::douconel::EdgeID;
use douconel::douconel::VertID;
use douconel::douconel_embedded::EmbeddedVertex;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use serde::Deserialize;
use serde::Serialize;
use slotmap::SlotMap;
use std::collections::HashMap;
use std::sync::Arc;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash, Default, Serialize, Deserialize)]
pub enum PrincipalDirection {
    #[default]
    X = 0,
    Y = 1,
    Z = 2,
}
impl PrincipalDirection {
    #[inline]
    pub const fn to_vector(self) -> Vector3D {
        match self {
            Self::X => Vector3D::new(1., 0., 0.),
            Self::Y => Vector3D::new(0., 1., 0.),
            Self::Z => Vector3D::new(0., 0., 1.),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Path {
    // A path is defined by a sequence of half-edges.
    pub edges: Vec<(EdgeID, EdgeID)>,
    // the direction or labeling associated with the path
    pub direction: PrincipalDirection,
}

// #[derive(Default, Clone, Debug, Serialize, Deserialize)]
// pub struct Vertex {
//     pub some_edge: Option<usize>,

//     // Auxiliary data
//     pub position: Vec3,
//     pub normal: Vec3,
//     pub original_face_id: FaceID,
//     pub ordering: Vec<(usize, EdgeID)>,
// }

// #[derive(Default, Clone, Debug, Serialize, Deserialize)]
// pub struct Face {
//     pub some_edge: Option<usize>,

//     // Auxiliary data
//     pub color: Color,
//     pub normal: Vec3,
//     pub dual_position: Option<Vec3>,
//     pub dual_normal: Option<Vec3>,
//     pub original_face: Option<usize>,
// }

// #[derive(Default, Clone, Debug, Serialize, Deserialize)]
// pub struct Edge {
//     pub root: usize,
//     pub face: Option<usize>,
//     pub next: Option<usize>,
//     pub twin: Option<usize>,

//     // Auxiliary data
//     pub label: Option<usize>,
//     pub part_of_path: Option<usize>,
//     pub edges_between: Option<Vec<EdgeID>>,
//     pub edges_between_endpoints: Option<(usize, usize)>,
//     pub direction: Option<PrincipalDirection>,
// }

// Dual structure is an "ABSTRACT PATH REPRESENTATION"
// TODO: turn this into its own struct, then DUAL inherits this
// TODO: maybe give paths actual IDs for robust removal etc?
#[derive(Clone, Debug, Default)]
pub struct Dual {
    pub paths: SlotMap<PathID, Path>,
    pub mesh_ref: Arc<Douconel<EmbeddedVertex, (), ()>>,
    pub new_mesh: Douconel<Intersection, (), ()>,
}

impl Dual {
    pub fn new(mesh_ref: Arc<Douconel<EmbeddedVertex, (), ()>>) -> Self {
        Self {
            paths: SlotMap::with_key(),
            mesh_ref,
            new_mesh: Douconel::default(),
        }
    }

    pub fn verify_nonoverlap(&self) -> bool {
        let edge_to_paths = self.get_edge_to_paths_map();
        for paths in edge_to_paths.values() {
            if paths.len() > 2 {
                return false;
            }
        }
        true
    }

    pub fn get_edge_to_paths_map(&self) -> HashMap<EdgeID, Vec<PathID>> {
        let mut map = HashMap::new();
        for (path_id, path) in &self.paths {
            for &(e0, e1) in &path.edges {
                map.entry(e0).or_insert(Vec::new()).push(path_id);
                map.entry(e1).or_insert(Vec::new()).push(path_id);
            }
        }
        map
    }

    pub fn get_edgepair_to_path_map(&self) -> HashMap<(EdgeID, EdgeID), PathID> {
        let mut map = HashMap::new();
        for (path_id, path) in &self.paths {
            for &(e0, e1) in &path.edges {
                assert!(!map.contains_key(&(e0, e1)) && !map.contains_key(&(e1, e0)));
                map.insert((e0, e1), path_id);
                map.insert((e1, e0), path_id);
            }
        }
        map
    }

    pub fn intersections(&mut self) -> Option<()> {
        // Create the following "map"
        // List of all intersections
        // At each intersection, list all path segments that intersect there
        // We define both: the "next" intersection, and the local "next" edge
        // Using this information, we should be able to create an embedded graph (HEDS)

        if self.paths.len() < 3 {
            return Some(());
        }

        let edge_to_paths = self.get_edge_to_paths_map();

        // For each path, find all its intersections (in order of the path edges)
        let path_to_intersections: HashMap<PathID, Vec<([EdgeID; 2], [PathID; 2])>> = self
            .paths
            .iter()
            .map(|(path_id, path)| {
                let path_intersections = path
                    .edges
                    .clone()
                    .into_iter()
                    .filter_map(|(edge, _)| {
                        if let Some(set) = edge_to_paths.get(&edge) {
                            if set.len() == 2 {
                                let twin_edge = self.mesh_ref.twin(edge);
                                if edge < twin_edge {
                                    return Some(([edge, twin_edge], [set[1], set[0]]));
                                } else {
                                    return Some(([twin_edge, edge], [set[1], set[0]]));
                                }
                            }
                        }
                        None
                    })
                    .collect_vec();
                (path_id, path_intersections)
            })
            .collect();

        // TODO: if two path segments with exact same intersections ( a face of degree 2 ), we cannot do it.

        let mut intersections = HashMap::new();
        for path_intersections in path_to_intersections.values() {
            for &(edges, [path1, path2]) in path_intersections {
                let intersection_id = edges[0];

                let next_edges = [
                    (edges[0], self.mesh_ref.next(edges[0])),
                    (edges[0], self.mesh_ref.next(self.mesh_ref.next(edges[0]))),
                    (edges[1], self.mesh_ref.next(edges[1])),
                    (edges[1], self.mesh_ref.next(self.mesh_ref.next(edges[1]))),
                ];

                // Intersection TODO: when is it invalid?, filter out (early return) on invalid intersections. (or faces)
                let nexts = next_edges.map(|(intersection_edge, next_edge)| {
                    let (next_path, direction) = if self.paths[path1]
                        .edges
                        .contains(&(intersection_edge, next_edge))
                    {
                        (path1, 1)
                    } else if self.paths[path1]
                        .edges
                        .contains(&(next_edge, intersection_edge))
                    {
                        (path1, -1)
                    } else if self.paths[path2]
                        .edges
                        .contains(&(intersection_edge, next_edge))
                    {
                        (path2, 1)
                    } else if self.paths[path2]
                        .edges
                        .contains(&(next_edge, intersection_edge))
                    {
                        (path2, -1)
                    } else {
                        panic!("Invalid intersection!");
                    };
                    let next_path_intersections = path_to_intersections.get(&next_path).unwrap();
                    let nr_of_intersections = next_path_intersections.len();
                    let this_intersection = next_path_intersections
                        .iter()
                        .position(|(edges, _)| edges[0] == intersection_id)
                        .unwrap()
                        + nr_of_intersections;

                    (
                        next_edge,
                        next_path,
                        next_path_intersections
                            [(this_intersection as i32 + direction) as usize % nr_of_intersections]
                            .0[0],
                    )
                });

                intersections.insert(
                    intersection_id,
                    Intersection {
                        this: edges,
                        next: nexts,
                    },
                );
            }
        }

        let intersection_ids = intersections.keys().cloned().collect_vec();

        // Construct all edges
        let edges = intersections
            .iter()
            .flat_map(|(&this_id, intersection)| {
                intersection
                    .next
                    .iter()
                    .map(|&(_, _, next_id)| (this_id, next_id))
                    .collect_vec()
            })
            .collect_vec();

        // Given an edge (this_x, next_x), find the next edge (next_x, next_next_x)
        let mut path_to_next = HashMap::new();
        for &(this_x, next_x) in &edges {
            let candidates = intersections.get(&next_x).unwrap().next.into_iter();

            // Find the local edge that connects this_x and next_x
            let edge_to_this = candidates.clone().find(|&(_, _, x)| x == this_x).unwrap().0;

            // Find the local edge that connects next_x and next_next_x (the one with the smallest clockwise angle)
            let next_next_x = candidates
                .filter(|&(candidate_edge, _, _)| candidate_edge != edge_to_this)
                .map(|(candidate_edge, _, candidate_x)| {
                    let clockwise_angle = hutspot::geom::calculate_clockwise_angle(
                        self.mesh_ref.midpoint(next_x),
                        self.mesh_ref.midpoint(edge_to_this),
                        self.mesh_ref.midpoint(candidate_edge),
                        self.mesh_ref.normal(self.mesh_ref.face(next_x)),
                    );
                    (candidate_x, clockwise_angle)
                })
                .sorted_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .next()
                .unwrap()
                .0;

            path_to_next.insert((this_x, next_x), (next_x, next_next_x));
        }

        // Construct all faces
        let mut faces = vec![];
        let mut edges_copy = edges.clone();
        while let Some(start) = edges_copy.pop() {
            let mut face = vec![];
            let mut this = start;
            loop {
                face.push(intersection_ids.iter().position(|&x| x == this.0).unwrap());
                this = path_to_next.get(&this).unwrap().clone();
                if this == start {
                    break;
                }
                edges_copy.retain(|&e| e != this);
            }
            faces.push(face);
        }

        // // map vertices to positions
        // let vertex_positions = intersections
        //     .keys()
        //     .map(|&e| self.mesh_ref.midpoint(e))
        //     .collect_vec();

        // Create douconel based on these faces
        let (mut new_mesh, vmap, _) = Douconel::<Intersection, (), ()>::from_faces(&faces).unwrap();

        for (vertex_id, vertex_obj) in &mut new_mesh.verts {
            let intersection_id =
                intersection_ids[vmap.get_by_right(&vertex_id).unwrap().to_owned()];
            *vertex_obj = intersections.get(&intersection_id).unwrap().to_owned();
        }

        self.new_mesh = new_mesh;

        return Some(());
    }

    pub fn get_intersection(&self, intersection_id: VertID) -> &Intersection {
        self.new_mesh.verts.get(intersection_id).unwrap()
    }

    pub fn get_paths_at_intersection(&self, intersection_id: VertID) -> Vec<PathID> {
        self.get_intersection(intersection_id)
            .next
            .iter()
            .map(|&(_, path, _)| path)
            .collect()
    }

    pub fn get_path_between_intersections(&self, start: VertID, end: VertID) -> Option<PathID> {
        let start_intersection = self.get_intersection(start);
        let end_intersection = self.get_intersection(end);
        start_intersection.next.iter().find_map(|&(_, path, next)| {
            if end_intersection.this.contains(&next) {
                Some(path)
            } else {
                None
            }
        })
    }
}

slotmap::new_key_type! {
    pub struct PathID;
    pub struct IntersectionID;
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Intersection {
    pub this: [EdgeID; 2],
    // The next local edge,, the path that follows, and the next intersection
    pub next: [(EdgeID, PathID, EdgeID); 4],
}
