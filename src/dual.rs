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

use crate::EmbeddedMesh;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash, Default, Serialize, Deserialize)]
pub enum PrincipalDirection {
    #[default]
    X,
    Y,
    Z,
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

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Loop {
    // A loop is defined by a sequence of half-edges.
    pub edges: Vec<(EdgeID, EdgeID)>,
    // the direction or labeling associated with the loop
    pub direction: PrincipalDirection,
}

impl Loop {
    pub fn find_edge(&self, edge: EdgeID) -> (usize, usize) {
        (
            self.edges
                .iter()
                .position(|&(_, right)| right == edge)
                .unwrap(),
            self.edges
                .iter()
                .position(|&(left, _)| left == edge)
                .unwrap(),
        )
    }

    pub fn left(&self, edge: EdgeID) -> EdgeID {
        self.edges[self.find_edge(edge).0].0
    }

    pub fn right(&self, edge: EdgeID) -> EdgeID {
        self.edges[self.find_edge(edge).1].1
    }

    pub fn between(&self, start: EdgeID, end: EdgeID) -> Vec<(EdgeID, EdgeID)> {
        let start_pos = self.find_edge(start).1;
        let end_pos = self.find_edge(end).0;

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
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LoopSegment {
    // A loop segment is defined by a reference to a loop (id)
    pub loop_id: LoopID,
    // And a reference to the two half-edges that limit the segment
    pub edges: [EdgeID; 2],
}

type LoopStructure = Douconel<Intersection, LoopSegment, ()>;

slotmap::new_key_type! {
    pub struct LoopID;
    pub struct IntersectionID;
}

// Dual structure is an "ABSTRACT PATH REPRESENTATION"
#[derive(Clone, Debug, Default)]
pub struct Dual {
    mesh_ref: Arc<EmbeddedMesh>,
    loops: SlotMap<LoopID, Loop>,
    loop_structure: LoopStructure,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Intersection {
    // An intersection is defined by the midpoint of two half-edges. At this point, two loops intersect.
    pub this: [EdgeID; 2],
    // The two loops come from the four half-edges adjacent to `this`.
    // The next local edge (adjacent to `this`), the loop, and the intersection (defined by an EdgeID) that is reached by following the loop into the direction of the local edge. The direction is either 1 or -1.
    pub next: [(EdgeID, LoopID, EdgeID, i32); 4],
}

impl Dual {
    pub fn new(mesh_ref: Arc<EmbeddedMesh>) -> Self {
        Self {
            mesh_ref,
            loops: SlotMap::with_key(),
            loop_structure: Douconel::default(),
        }
    }

    pub fn get_mesh_ref(&self) -> &Arc<EmbeddedMesh> {
        &self.mesh_ref
    }

    pub fn get_loop_structure(&self) -> &LoopStructure {
        &self.loop_structure
    }

    pub fn get_loop(&self, loop_id: LoopID) -> Option<&Loop> {
        self.loops.get(loop_id)
    }

    pub fn get_loops(&self) -> impl Iterator<Item = (LoopID, &Loop)> {
        self.loops.iter()
    }

    pub fn del_loop(&mut self, loop_id: LoopID) -> Option<Loop> {
        self.loops.remove(loop_id)
    }

    pub fn add_loop(&mut self, l: Loop) -> Option<LoopID> {
        let loop_id = self.loops.insert(l);

        if !self.verify_nonoverlap() || !self.build_loop_structure() {
            self.del_loop(loop_id);
            return None;
        }

        Some(loop_id)
    }

    pub fn verify_nonoverlap(&self) -> bool {
        let edge_to_loops = self.get_edge_to_loops_map();
        for l in edge_to_loops.values() {
            if l.len() > 2 {
                return false;
            }
        }
        true
    }

    pub fn get_edge_to_loops_map(&self) -> HashMap<EdgeID, Vec<LoopID>> {
        let mut map = HashMap::new();
        for (loop_id, l) in &self.loops {
            for &(e0, e1) in &l.edges {
                map.entry(e0).or_insert(Vec::new()).push(loop_id);
                map.entry(e1).or_insert(Vec::new()).push(loop_id);
            }
        }
        map
    }

    pub fn get_edgepair_to_loop_map(&self) -> HashMap<(EdgeID, EdgeID), LoopID> {
        let mut map = HashMap::new();
        for (loop_id, l) in &self.loops {
            for &(e0, e1) in &l.edges {
                assert!(!map.contains_key(&(e0, e1)) && !map.contains_key(&(e1, e0)));
                map.insert((e0, e1), loop_id);
                map.insert((e1, e0), loop_id);
            }
        }
        map
    }

    pub fn count_loops_in_direction(&self, direction: PrincipalDirection) -> usize {
        self.loops
            .iter()
            .filter(|(_, l)| l.direction == direction)
            .count()
    }

    pub fn build_loop_structure(&mut self) -> bool {
        // Create the following "map"
        // List of all intersections
        // At each intersection, list all path segments that intersect there
        // We define both: the "next" intersection, and the local "next" edge
        // Using this information, we should be able to create an embedded graph (HEDS)

        if self.count_loops_in_direction(PrincipalDirection::X) == 0
            || self.count_loops_in_direction(PrincipalDirection::Y) == 0
            || self.count_loops_in_direction(PrincipalDirection::Z) == 0
        {
            return true;
        }

        let edge_to_paths = self.get_edge_to_loops_map();

        println!("Finding loop to intersections...");

        // For each loop, find all its intersections (in order of the loop edges)
        let loop_to_intersections: HashMap<LoopID, Vec<([EdgeID; 2], [LoopID; 2])>> = self
            .loops
            .iter()
            .map(|(loop_id, l)| {
                let path_intersections = l
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
                (loop_id, path_intersections)
            })
            .collect();

        // TODO: if two path segments with exact same intersections ( a face of degree 2 ), we cannot do it.
        // TODO: return false if graph is not connected
        // TODO: return false if intersections are not transversal.

        let mut intersections = HashMap::new();
        for loop_intersections in loop_to_intersections.values() {
            for &(edges, [l1, l2]) in loop_intersections {
                let intersection_id = edges[0];

                let next_edges = [
                    (edges[0], self.mesh_ref.next(edges[0])),
                    (edges[0], self.mesh_ref.next(self.mesh_ref.next(edges[0]))),
                    (edges[1], self.mesh_ref.next(edges[1])),
                    (edges[1], self.mesh_ref.next(self.mesh_ref.next(edges[1]))),
                ];

                // Intersection TODO: when is it invalid?, filter out (early return) on invalid intersections. (or faces)
                let nexts = next_edges
                    .into_iter()
                    .map(|(intersection_edge, next_edge)| {
                        if let Some((next_loop, direction)) = if self.loops[l1]
                            .edges
                            .contains(&(intersection_edge, next_edge))
                        {
                            Some((l1, 1))
                        } else if self.loops[l1]
                            .edges
                            .contains(&(next_edge, intersection_edge))
                        {
                            Some((l1, -1))
                        } else if self.loops[l2]
                            .edges
                            .contains(&(intersection_edge, next_edge))
                        {
                            Some((l2, 1))
                        } else if self.loops[l2]
                            .edges
                            .contains(&(next_edge, intersection_edge))
                        {
                            Some((l2, -1))
                        } else {
                            None
                        } {
                            let next_loop_intersections =
                                loop_to_intersections.get(&next_loop).unwrap();
                            let nr_of_intersections = next_loop_intersections.len();
                            let this_intersection = next_loop_intersections
                                .iter()
                                .position(|(edges, _)| edges[0] == intersection_id)
                                .unwrap()
                                + nr_of_intersections;
                            let next_intersection =
                                next_loop_intersections[(this_intersection as i32 + direction)
                                    as usize
                                    % nr_of_intersections]
                                    .0[0];

                            Some((next_edge, next_loop, next_intersection, direction))
                        } else {
                            None
                        }
                    })
                    .filter(|x| x.is_some())
                    .map(|x| x.unwrap())
                    .collect_vec();

                if nexts.len() != 4 {
                    return false;
                } else {
                    intersections.insert(
                        intersection_id,
                        Intersection {
                            this: edges,
                            next: [nexts[0], nexts[1], nexts[2], nexts[3]],
                        },
                    );
                }
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
                    .map(|&(_, _, next_id, _)| (this_id, next_id))
                    .collect_vec()
            })
            .collect_vec();

        // Given an edge (this_x, next_x), find the next edge (next_x, next_next_x)
        let mut loop_to_next = HashMap::new();
        for &(this_x, next_x) in &edges {
            let candidates = intersections.get(&next_x).unwrap().next.into_iter();

            // Find the local edge that connects this_x and next_x
            let edge_to_this = candidates
                .clone()
                .find(|&(_, _, x, _)| x == this_x)
                .unwrap()
                .0;

            // Find the local edge that connects next_x and next_next_x (the one with the smallest clockwise angle)
            let next_next_x = candidates
                .filter(|&(candidate_edge, _, _, _)| candidate_edge != edge_to_this)
                .map(|(candidate_edge, _, candidate_x, _)| {
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

            loop_to_next.insert((this_x, next_x), (next_x, next_next_x));
        }

        // Construct all faces
        let mut faces = vec![];
        let mut edges_copy = edges.clone();
        while let Some(start) = edges_copy.pop() {
            let mut counter = 0;
            let mut face = vec![];
            let mut this = start;
            loop {
                face.push(intersection_ids.iter().position(|&x| x == this.0).unwrap());
                this = loop_to_next.get(&this).unwrap().clone();
                edges_copy.retain(|&e| e != this);
                if this == start {
                    break;
                }
                counter += 1;
                if counter > 100 {
                    // Malformed face
                    return false;
                }
            }
            faces.push(face);
        }

        // // map vertices to positions
        // let vertex_positions = intersections
        //     .keys()
        //     .map(|&e| self.mesh_ref.midpoint(e))
        //     .collect_vec();

        // Create douconel based on these faces
        if let Ok((loop_structure, vmap, _)) = LoopStructure::from_faces(&faces) {
            self.loop_structure = loop_structure;
            for (vertex_id, vertex_obj) in &mut self.loop_structure.verts {
                let intersection_id =
                    intersection_ids[vmap.get_by_right(&vertex_id).unwrap().to_owned()];
                *vertex_obj = intersections.get(&intersection_id).unwrap().to_owned();
            }

            let mut edge_to_loop_segment = HashMap::new();

            for (edge_id, _) in &self.loop_structure.edges {
                let root = self.loop_structure.root(edge_id);
                let toor = self.loop_structure.toor(edge_id);

                let root_next = self.loop_structure.verts[root]
                    .next
                    .into_iter()
                    .find(|&(_, _, next_intersection_id, _)| {
                        next_intersection_id == self.loop_structure.verts[toor].this[0]
                    })
                    .unwrap();

                let toor_next = self.loop_structure.verts[toor]
                    .next
                    .into_iter()
                    .find(|&(_, _, next_intersection_id, _)| {
                        next_intersection_id == self.loop_structure.verts[root].this[0]
                    })
                    .unwrap();

                assert!(root_next.1 == toor_next.1);

                let loop_id = root_next.1;
                let local_edges = (root_next.0, toor_next.0);

                edge_to_loop_segment.insert(
                    edge_id,
                    LoopSegment {
                        loop_id,
                        edges: [local_edges.0, local_edges.1],
                    },
                );
            }

            for (edge_id, loop_segment) in edge_to_loop_segment {
                self.loop_structure.edges[edge_id] = loop_segment;
            }

            return true;
        } else {
            return false;
        }
    }

    pub fn get_intersection(&self, intersection_id: VertID) -> &Intersection {
        self.loop_structure.verts.get(intersection_id).unwrap()
    }

    pub fn get_paths_at_intersection(&self, intersection_id: VertID) -> Vec<LoopID> {
        self.get_intersection(intersection_id)
            .next
            .iter()
            .map(|&(_, path, _, _)| path)
            .collect()
    }

    pub fn get_path_between_intersections(&self, start: VertID, end: VertID) -> Option<LoopID> {
        let start_intersection = self.get_intersection(start);
        let end_intersection = self.get_intersection(end);
        start_intersection
            .next
            .iter()
            .find_map(|&(_, path, next, _)| {
                if end_intersection.this.contains(&next) {
                    Some(path)
                } else {
                    None
                }
            })
    }
}
