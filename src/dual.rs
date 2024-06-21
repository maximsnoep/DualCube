#![warn(clippy::all, clippy::pedantic, clippy::nursery, clippy::cargo)]

use bevy::prelude::*;
use douconel::douconel::Douconel;
use douconel::douconel::EdgeID;
use douconel::douconel::VertID;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use serde::Deserialize;
use serde::Serialize;
use slotmap::SlotMap;
use std::collections::HashMap;
use std::collections::HashSet;
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
    pub edges: Vec<EdgeID>,
    // the direction or labeling associated with the loop
    pub direction: PrincipalDirection,
}

impl Loop {
    pub fn cyclic_iterator(&self) -> impl Iterator<Item = EdgeID> + '_ {
        self.edges
            .iter()
            .copied()
            .chain(self.edges.iter().copied().take(1))
    }

    pub fn contains_pair(&self, needle: (EdgeID, EdgeID)) -> bool {
        self.cyclic_iterator()
            .tuple_windows::<(EdgeID, EdgeID)>()
            .any(|es| es == needle)
    }

    pub fn find_edge(&self, needle: EdgeID) -> usize {
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
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub struct LoopSegment {
    // A loop segment is defined by a reference to a loop (id)
    pub loop_id: LoopID,
    // The start and end of the segment (intersections)
    pub start: EdgeID,
    pub end: EdgeID,
    // And a sequence of half-edges that define the segment of the loop (inbetween start and end)
    pub between: Vec<EdgeID>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Subsurface {
    // A subsurface is defined by a set of vertices
    pub verts: Vec<VertID>,
    pub color: Color,
}

type LoopStructure = Douconel<Intersection, LoopSegment, Subsurface>;

slotmap::new_key_type! {
    pub struct LoopID;
    pub struct IntersectionID;
}

// Dual structure is an "ABSTRACT PATH REPRESENTATION"
#[derive(Clone, Debug, Default)]
pub struct Dual {
    mesh_ref: Arc<EmbeddedMesh>,
    loops: SlotMap<LoopID, Loop>,
    occupied: HashMap<EdgeID, Vec<LoopID>>,
    loop_structure: LoopStructure,
    pub intersections: HashMap<EdgeID, Intersection>,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Intersection {
    // An intersection is defined by the midpoint of two half-edges. At this point, two loops intersect. The intersection is defined by the lower half-edge.
    pub this: EdgeID,
    // The two loops come from the four half-edges adjacent to `this`.
    // The next local edge (adjacent to `this`), the loop, and the intersection (defined by an EdgeID) that is reached by following the loop into the direction of the local edge. The direction is either 1 or -1.
    pub next: [(EdgeID, LoopID, EdgeID); 4],
}

impl Dual {
    pub fn new(mesh_ref: Arc<EmbeddedMesh>) -> Self {
        Self {
            mesh_ref,
            loops: SlotMap::with_key(),
            loop_structure: Douconel::default(),
            occupied: HashMap::new(),
            intersections: HashMap::new(),
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

    pub fn get_pairs_of_loop(&self, loop_id: LoopID) -> Vec<[EdgeID; 2]> {
        self.get_loop(loop_id)
            .unwrap()
            .edges
            .windows(2)
            .filter_map(|w| {
                if self.mesh_ref.twin(w[0]) != w[1] {
                    Some([w[0], w[1]])
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn get_loops(&self) -> impl Iterator<Item = (LoopID, &Loop)> {
        self.loops.iter()
    }

    pub fn del_loop(&mut self, loop_id: LoopID) {
        for &e in &self.get_loop(loop_id).unwrap().edges.clone() {
            self.occupied.entry(e).and_modify(|v| {
                v.retain(|&l| l != loop_id);
            });
            if let Some(r) = self.occupied.get(&e) {
                if r.is_empty() {
                    self.occupied.remove(&e);
                }
            }
        }

        self.loops.remove(loop_id);
        self.build_loop_structure();
    }

    pub fn add_loop(&mut self, l: Loop) -> Option<LoopID> {
        let loop_id = self.loops.insert(l.clone());

        for e in self.get_loop(loop_id).unwrap().edges.clone() {
            self.occupied.entry(e).or_insert(vec![]).push(loop_id);
        }

        for [e0, e1] in self.get_pairs_of_loop(loop_id) {
            if let Some(l) = self.is_occupied([e0, e1]) {
                if l != loop_id {
                    self.del_loop(loop_id);
                    return None;
                }
            }
        }

        if !self.build_loop_structure() {
            self.del_loop(loop_id);
            return None;
        }

        Some(loop_id)
    }

    pub fn get_paths_in_edge(&self, edge: EdgeID) -> Option<Vec<LoopID>> {
        self.occupied.get(&edge).cloned()
    }

    pub fn is_occupied(&self, [e1, e2]: [EdgeID; 2]) -> Option<LoopID> {
        if let Some(l1) = self.occupied.get(&e1) {
            if let Some(l2) = self.occupied.get(&e2) {
                for &l in l1 {
                    if l2.contains(&l) {
                        if self.get_loop(l).unwrap().contains_pair((e1, e2))
                            || self.get_loop(l).unwrap().contains_pair((e2, e1))
                        {
                            return Some(l);
                        }
                    }
                }
            }
        }
        return None;
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

        self.loop_structure = LoopStructure::default();

        if self.count_loops_in_direction(PrincipalDirection::X) == 0
            || self.count_loops_in_direction(PrincipalDirection::Y) == 0
            || self.count_loops_in_direction(PrincipalDirection::Z) == 0
        {
            return true;
        }

        #[derive(Debug)]
        struct IntersectionMarker {
            edge: EdgeID,
            loops: [LoopID; 2],
        }

        // For each loop:
        //   We find all its intersections, by following the edges that the loop passes
        //   Therefore, the intersections are in the order of the loop traversal
        //   An intersection is defined per two half-edges, we only store the intersection at the first (lower) half-edge
        let loop_to_intersections: HashMap<LoopID, Vec<IntersectionMarker>> = self
            .loops
            .iter()
            .map(|(loop_id, l)| {
                let path_intersections = l
                    .edges
                    .clone()
                    .into_iter()
                    .map(|edge| (edge, self.get_paths_in_edge(edge).unwrap()))
                    .filter(|(_, set)| set.len() == 2)
                    .filter(|&(edge, _)| edge > self.mesh_ref.twin(edge))
                    .map(|(edge, set)| IntersectionMarker {
                        edge,
                        loops: [set[0], set[1]],
                    })
                    .collect_vec();
                (loop_id, path_intersections)
            })
            .collect();

        // TODO: if two path segments with exact same intersections ( a face of degree 2 ), we cannot do it.
        // TODO: return false if graph is not connected
        // TODO: return false if intersections are not transversal.
        // Intersection TODO: when is it invalid?, filter out (early return) on invalid intersections. (or faces)

        // For each intersection:
        //   We find its (4) next intersections by following its associated loops
        self.intersections = HashMap::new();
        for loop_intersections in loop_to_intersections.values() {
            // for &(edges, [l1, l2]) in loop_intersections {
            for intersection_marker in loop_intersections {
                let this_edge = intersection_marker.edge;
                let twin_edge = self.mesh_ref.twin(this_edge);

                // The four directions that we can go from this intersection
                let next_edges = [
                    (this_edge, self.mesh_ref.next(this_edge)),
                    (this_edge, self.mesh_ref.next(self.mesh_ref.next(this_edge))),
                    (twin_edge, self.mesh_ref.next(twin_edge)),
                    (twin_edge, self.mesh_ref.next(self.mesh_ref.next(twin_edge))),
                ];

                // Map each direction into the next intersection and the loop that is followed
                let nexts = next_edges
                    .into_iter()
                    .filter_map(|(edge, next_edge)| {
                        assert!(edge != next_edge);

                        if self.loops[intersection_marker.loops[0]].contains_pair((edge, next_edge))
                        {
                            Some((next_edge, intersection_marker.loops[0], 1))
                        } else if self.loops[intersection_marker.loops[0]]
                            .contains_pair((next_edge, edge))
                        {
                            Some((next_edge, intersection_marker.loops[0], -1))
                        } else if self.loops[intersection_marker.loops[1]]
                            .contains_pair((edge, next_edge))
                        {
                            Some((next_edge, intersection_marker.loops[1], 1))
                        } else if self.loops[intersection_marker.loops[1]]
                            .contains_pair((next_edge, edge))
                        {
                            Some((next_edge, intersection_marker.loops[1], -1))
                        } else {
                            None
                        }
                    })
                    .map(|(next_edge, follow_loop, direction)| {
                        let follow_loop_intersections =
                            loop_to_intersections.get(&follow_loop).unwrap();

                        // Find current intersection in `follow_loop`
                        let this_intersection = follow_loop_intersections
                            .iter()
                            .position(|x| x.edge == this_edge)
                            .unwrap();

                        // Find the next intersection in `follow_loop`
                        let next_intersection = follow_loop_intersections[((this_intersection
                            + follow_loop_intersections.len())
                            as i32
                            + direction)
                            as usize
                            % follow_loop_intersections.len()]
                        .edge;

                        (next_edge, follow_loop, next_intersection)
                    })
                    .collect_vec();

                assert!(nexts.len() == 4);

                for next in &nexts {
                    if next.0 == next.2 {
                        return false;
                    }
                }

                self.intersections.insert(
                    this_edge,
                    Intersection {
                        this: this_edge,
                        next: [nexts[0], nexts[1], nexts[2], nexts[3]],
                    },
                );
            }
        }

        // Construct all loop segments
        // For each intersection:
        //   For each next intersection:
        //     We create a loop segment, that connects the two intersections
        let edges = self
            .intersections
            .iter()
            .flat_map(|(this_id, intersection)| {
                intersection.next.iter().map(|(_, loop_id, next_id)| {
                    (this_id.clone(), next_id.clone(), loop_id.clone())
                })
            })
            .map(|(this_id, next_id, loop_id)| {
                let this_intersection = self.intersections.get(&this_id).unwrap();
                let next_intersection = self.intersections.get(&next_id).unwrap();

                let this_intersection_pointer_to_next = this_intersection
                    .next
                    .into_iter()
                    .find(|&(_, next_loop_id, next_intersection_id)| {
                        next_intersection_id == next_id && next_loop_id == loop_id
                    })
                    .unwrap();

                let next_intersection_pointer_to_this = next_intersection
                    .next
                    .into_iter()
                    .find(|&(_, next_loop_id, next_intersection_id)| {
                        next_intersection_id == this_id && next_loop_id == loop_id
                    })
                    .unwrap();

                assert!(this_intersection_pointer_to_next.1 == next_intersection_pointer_to_this.1);

                let between_a = self.get_loop(loop_id).unwrap().between(
                    this_intersection_pointer_to_next.0,
                    next_intersection_pointer_to_this.0,
                );
                let between_b = self
                    .get_loop(loop_id)
                    .unwrap()
                    .between(
                        next_intersection_pointer_to_this.0,
                        this_intersection_pointer_to_next.0,
                    )
                    .into_iter()
                    .rev()
                    .collect_vec();

                let assert_a = between_a
                    .iter()
                    .filter(|&&e| e == this_id || e == next_id)
                    .count()
                    == 0;
                let assert_b = between_b
                    .iter()
                    .filter(|&&e| e == this_id || e == next_id)
                    .count()
                    == 0;

                assert!(assert_a ^ assert_b);

                let between = match (assert_a, assert_b) {
                    (true, false) => between_a,
                    (false, true) => between_b,
                    _ => unreachable!(),
                };

                LoopSegment {
                    loop_id,
                    start: this_id,
                    end: next_id,
                    between,
                }
            })
            .collect_vec();

        // Given a loop segment (start: this_x, end: next_x)
        //   We find the next loop segment (start: next_x, end: next_next_x)
        let mut loop_segment_to_next = HashMap::new();
        for loop_segment in &edges {
            let this_x = loop_segment.start;
            let next_x = loop_segment.end;

            let candidates = self.intersections.get(&next_x).unwrap().next.into_iter();

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

            // Find the loop segment that connects next_x and next_next_x
            for next_loop_segment in &edges {
                if next_loop_segment.start == next_x && next_loop_segment.end == next_next_x {
                    loop_segment_to_next.insert(loop_segment, next_loop_segment);
                    break;
                }
            }
        }

        let intersection_ids = self.intersections.keys().copied().collect_vec();

        // Construct all faces
        let mut faces = vec![];
        let mut edges_copy = edges.clone();
        while let Some(start) = edges_copy.pop() {
            let mut counter = 0;
            let mut face = vec![];
            let mut this = start.clone();
            loop {
                face.push(
                    intersection_ids
                        .iter()
                        .position(|&x| x == this.start)
                        .unwrap(),
                );
                this = loop_segment_to_next.get(&this).unwrap().to_owned().clone();
                edges_copy.retain(|e| !(e.start == this.start && e.end == this.end));
                if this.start == start.start {
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

        // Create douconel based on these faces
        if let Ok((loop_structure, vmap, _)) = LoopStructure::from_faces(&faces) {
            self.loop_structure = loop_structure.clone();
            for (vertex_id, vertex_obj) in &mut self.loop_structure.verts {
                let intersection_id =
                    intersection_ids[vmap.get_by_right(&vertex_id).unwrap().to_owned()];
                *vertex_obj = self.intersections.get(&intersection_id).unwrap().to_owned();
            }

            for (edge_id, edge_obj) in &mut self.loop_structure.edges {
                let start = self.loop_structure.verts[loop_structure.root(edge_id)].this;
                let end = self.loop_structure.verts[loop_structure.toor(edge_id)].this;
                *edge_obj = edges
                    .iter()
                    .find(|e| e.start == start && e.end == end)
                    .unwrap()
                    .clone();
            }

            // Map from some verts to adjacent loops(segments)
            let mut vert_to_loop_segments: HashMap<VertID, HashSet<_>> = HashMap::new();
            for (ls_id, ls) in &self.loop_structure.edges {
                // remove first and last element (intersection points)
                let between = &ls.between[1..ls.between.len() - 1];

                for &edge_id in between {
                    let endpoints = self.mesh_ref.endpoints(edge_id);
                    vert_to_loop_segments
                        .entry(endpoints.0)
                        .or_default()
                        .insert(ls_id);

                    vert_to_loop_segments
                        .entry(endpoints.1)
                        .or_default()
                        .insert(ls_id);
                }
            }

            let mut face_to_subsurface = HashMap::new();

            for (face_id, _) in &self.loop_structure.faces {
                let blocked: HashSet<EdgeID> = self
                    .loop_structure
                    .edges(face_id)
                    .into_iter()
                    .map(|edge_id| self.loop_structure.edges[edge_id].clone())
                    .flat_map(|ls| [ls.between, vec![ls.start, ls.end]].concat())
                    .collect();

                // Our loopsegments:
                let loop_segments: HashSet<_> = self
                    .loop_structure
                    .edges(face_id)
                    .iter()
                    .flat_map(|&e| [e, self.loop_structure.twin(e)])
                    .collect();

                // Make a neighborhood function that blocks all edges that are part of the loop segments of this face
                let nfunction = |vertex: VertID| {
                    self.mesh_ref.neighbor_function_primal()(vertex)
                        .into_iter()
                        .filter(|&neighbor| {
                            let edge = self.mesh_ref.edge_between_verts(vertex, neighbor).unwrap();
                            !blocked.contains(&edge.0) && !blocked.contains(&edge.1)
                        })
                        .collect()
                };

                // Find the connected components (should be 2!)
                let ccs =
                    hutspot::graph::find_ccs(&self.mesh_ref.verts.keys().collect_vec(), nfunction);

                assert!(ccs.len() == 2);

                let verify_0: HashSet<_> = ccs[0]
                    .iter()
                    .flat_map(|&v| vert_to_loop_segments.get(&v).cloned().unwrap_or_default())
                    .collect();

                let verify_1: HashSet<_> = ccs[1]
                    .iter()
                    .flat_map(|&v| vert_to_loop_segments.get(&v).cloned().unwrap_or_default())
                    .collect();

                assert!(verify_0.is_subset(&loop_segments) ^ verify_1.is_subset(&loop_segments));

                if verify_0.is_subset(&loop_segments) {
                    let subsurface = Subsurface {
                        verts: ccs[0].iter().copied().collect(),
                        color: hutspot::color::hsl_to_rgb(rand::random::<f32>() * 360., 0.5, 0.5)
                            .into(),
                    };
                    face_to_subsurface.insert(face_id, subsurface);
                } else {
                    let subsurface = Subsurface {
                        verts: ccs[1].iter().copied().collect(),
                        color: hutspot::color::hsl_to_rgb(rand::random::<f32>() * 360., 0.5, 0.5)
                            .into(),
                    };
                    face_to_subsurface.insert(face_id, subsurface);
                }
            }

            for (face_id, subsurface) in face_to_subsurface {
                self.loop_structure.faces[face_id] = subsurface;
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
            .map(|&(_, path, _)| path)
            .collect()
    }

    pub fn get_path_between_intersections(&self, start: VertID, end: VertID) -> Option<LoopID> {
        let start_intersection = self.get_intersection(start);
        let end_intersection = self.get_intersection(end);
        start_intersection.next.iter().find_map(|&(_, path, next)| {
            if end_intersection.this == next || self.mesh_ref.twin(end_intersection.this) == next {
                Some(path)
            } else {
                None
            }
        })
    }
}
