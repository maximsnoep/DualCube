use crate::{
    solutions::{Loop, LoopID},
    EdgeID, EmbeddedMesh, PrincipalDirection, VertID,
};
use douconel::douconel::{Douconel, MeshError};
use itertools::Itertools;
use serde::{Deserialize, Serialize};
use slotmap::SlotMap;
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

// A collection of loops forms a loop structure; a graph. Vertices are loop intersections. Edges are loop segments. And faces are loop regions.
slotmap::new_key_type! {
    pub struct IntersectionID;
    pub struct SegmentID;
    pub struct RegionID;
}

pub type LoopStructure = Douconel<IntersectionID, EdgeID, SegmentID, Segment, RegionID, Region>;

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub struct Segment {
    // A loop segment is defined by a reference to a loop (id)
    pub loop_id: LoopID,
    // And the direction of the loop segment (either following the loop, or opposite direction of the loop)
    pub orientation: Orientation,
}

#[derive(Clone, Debug, Copy, Default, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum Orientation {
    #[default]
    Forwards,
    Backwards,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Region {
    // A region is defined by a set of vertices
    pub verts: HashSet<VertID>,
}

slotmap::new_key_type! {
    pub struct ZoneID;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Zone {
    // A zone is defined by a direction
    pub direction: PrincipalDirection,
    // All regions that are part of the zone
    pub regions: HashSet<RegionID>,
}

// Dual structure (of a polycube)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Dual {
    pub mesh_ref: Arc<EmbeddedMesh>,
    pub loops_ref: SlotMap<LoopID, Loop>,

    pub loop_structure: LoopStructure,
    pub zones: SlotMap<ZoneID, Zone>,

    // topological sort on the zones
    pub adjacency: (HashMap<ZoneID, HashSet<ZoneID>>, HashMap<ZoneID, HashSet<ZoneID>>),

    pub level_graphs: [HashMap<ZoneID, Vec<ZoneID>>; 3],
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub enum PropertyViolationError<IntersectionID> {
    #[default]
    UnknownError,
    MissingDirection,
    FaceWithDegreeLessThanThree,
    FaceWithDegreeMoreThanSix,
    InvalidFaceBoundary,
    SelfIntersection,
    NonTwoColorable,
    NonConnectedComponents,
    NonSeperatedDirection,
    CyclicDependency,
    PatchesMissing,
    LoopStructureError(MeshError<IntersectionID>),
    PathTooLong,
    PathEmpty,
    LoopHasTooFewIntersections,
}

impl Dual {
    pub fn from(mesh_ref: Arc<EmbeddedMesh>, loops_ref: &SlotMap<LoopID, Loop>) -> Result<Self, PropertyViolationError<IntersectionID>> {
        let mut dual = Self {
            mesh_ref,
            loops_ref: loops_ref.clone(),
            loop_structure: Douconel::default(),
            zones: SlotMap::with_key(),
            adjacency: (HashMap::new(), HashMap::new()),
            level_graphs: [HashMap::new(), HashMap::new(), HashMap::new()],
        };

        // Find all intersections and loop regions induced by the loops, and compute the loop structure
        dual.assign_loop_structure()?;

        // For each loop region, find its actual subsurface (on the mesh)
        dual.assign_subsurfaces();

        // Find the zones
        dual.assign_zones();

        // Construct the level graphs
        dual.assign_level_graphs();

        // Verify properties
        dual.verify_properties()?;

        Ok(dual)
    }

    pub fn intersection_to_edge(&self, intersection: IntersectionID) -> EdgeID {
        self.loop_structure.verts[intersection]
    }

    pub fn segment_to_loop(&self, segment: SegmentID) -> LoopID {
        self.loop_structure.edges[segment].loop_id
    }

    pub fn segment_to_orientation(&self, segment: SegmentID) -> Orientation {
        self.loop_structure.edges[segment].orientation
    }

    pub fn segment_to_endpoints(&self, segment: SegmentID) -> (EdgeID, EdgeID) {
        let endpoints = &self.loop_structure.endpoints(segment);
        (self.intersection_to_edge(endpoints.0), self.intersection_to_edge(endpoints.1))
    }

    pub fn region_to_zone(&self, region: RegionID, direction: PrincipalDirection) -> ZoneID {
        self.zones
            .iter()
            .filter(|(_, zone)| zone.direction == direction)
            .find(|(_, zone)| zone.regions.contains(&region))
            .unwrap()
            .0
    }

    pub fn segment_to_edges_between(&self, segment: SegmentID) -> Vec<EdgeID> {
        let endpoints = self.segment_to_endpoints(segment);
        let twins_of_endpoints = (self.mesh_ref.twin(endpoints.0), self.mesh_ref.twin(endpoints.1));
        let loop_id = self.loop_structure.edges[segment].loop_id;
        let mut between = if self.segment_to_orientation(segment) == Orientation::Forwards {
            self.loops_ref[loop_id].between(endpoints.0, endpoints.1)
        } else {
            self.loops_ref[loop_id].between(endpoints.1, endpoints.0)
        };

        between.retain(|&edge| edge != endpoints.0 && edge != endpoints.1 && edge != twins_of_endpoints.0 && edge != twins_of_endpoints.1);
        between
    }

    pub fn segment_to_edges(&self, segment: SegmentID) -> Vec<EdgeID> {
        let edges = self.segment_to_edges_between(segment);

        let mut fixed_edges = vec![];

        for edge_pair in edges.windows(2) {
            let (from, to) = (edge_pair[0], edge_pair[1]);

            // they are either twins, or they share a face
            let they_are_twins = self.mesh_ref.twin(from) == to;
            let they_share_face = self.mesh_ref.face(from) == self.mesh_ref.face(to);

            if they_are_twins || they_share_face {
                fixed_edges.push(from);
            } else {
                // there is an edge missing between them.
                // this missing edge is either the twin of from or the twin of to.
                let candidate_missing1 = self.mesh_ref.twin(from);
                let candidate_missing2 = self.mesh_ref.twin(to);

                // the true missing edge is the one that is twin to one and shares face with the other.
                let candidate_missing1_is_true = self.mesh_ref.face(candidate_missing1) == self.mesh_ref.face(to);
                let candidate_missing2_is_true = self.mesh_ref.face(candidate_missing2) == self.mesh_ref.face(from);
                assert!(candidate_missing1_is_true ^ candidate_missing2_is_true);

                let missing = if candidate_missing1_is_true { candidate_missing1 } else { candidate_missing2 };

                fixed_edges.push(from);
                fixed_edges.push(missing);
            }
        }
        fixed_edges.push(edges[edges.len() - 1]);

        fixed_edges
    }

    pub fn segment_to_direction(&self, segment: SegmentID) -> PrincipalDirection {
        let loop_id = self.loop_structure.edges[segment].loop_id;
        self.loops_ref[loop_id].direction
    }

    fn pos<T: PartialEq>(list: &[T], needle: &T) -> usize {
        list.iter().position(|x| x == needle).unwrap()
    }

    fn next<T: PartialEq + Copy>(list: &[T], needle: T) -> T {
        let pos = Self::pos(list, &needle);
        list[(pos + 1) % list.len()]
    }

    fn prev<T: PartialEq + Copy>(list: &[T], needle: T) -> T {
        let pos = Self::pos(list, &needle);
        list[(pos + list.len() - 1) % list.len()]
    }

    // Returns error if:
    //    1. A loop has less than 4 intersections.
    //    2. A face has more than 6 edges (we know the face degree is at most 6, so we can early stop, and we also want to prevent infinite loops / malformed faces)
    fn assign_loop_structure(&mut self) -> Result<(), PropertyViolationError<IntersectionID>> {
        // For each edge, we store the loops that pass it
        let mut occupied: HashMap<EdgeID, Vec<LoopID>> = HashMap::new();
        for loop_id in self.loops_ref.keys() {
            for &edge in &self.loops_ref[loop_id].edges {
                occupied.entry(edge).or_default().push(loop_id);
            }
        }

        // Intersections are edges that are occupied exactly twice. It is not possible for an edge to be occupied more than twice.
        // NOTE: An intersection exists on two half-edges, we only store the intersection at the lower ID half-edge
        let intersection_markers: HashMap<EdgeID, [LoopID; 2]> = occupied
            .into_iter()
            .filter_map(|(edge, loops)| {
                assert!(loops.len() == 1 || loops.len() == 2);
                if loops.len() == 2 && edge > self.mesh_ref.twin(edge) {
                    Some((edge, [loops[0], loops[1]]))
                } else {
                    None
                }
            })
            .collect();

        // For each loop we find its intersections
        let loop_to_intersection_markers: HashMap<LoopID, Vec<EdgeID>> = self
            .loops_ref
            .iter()
            .map(|(loop_id, lewp)| {
                (
                    loop_id,
                    lewp.edges.iter().filter(|edge| intersection_markers.contains_key(edge)).copied().collect_vec(),
                )
            })
            .collect();

        // If any loop has too few intersections (less than 4), we return an error
        if loop_to_intersection_markers.values().any(|x| x.len() < 4) {
            return Err(PropertyViolationError::LoopHasTooFewIntersections);
        }

        #[derive(Debug, Copy, Clone)]
        pub struct Intersection {
            // An intersection is defined by the midpoint of two half-edges. At this point, two loops intersect. The intersection is defined by the lower half-edge.
            pub this: EdgeID,
            // The two loops come from the four half-edges adjacent to `this`.
            // The next local edge (adjacent to `this`), the loop, and the intersection (defined by an EdgeID) that is reached by following the loop into the direction of the local edge. The direction is either 1 or -1.
            pub next: [(LoopID, EdgeID, Orientation); 4],
        }

        // For each intersection we find its adjacent intersections (should be 4, by following its associated (two) loops in all (two) directions.
        let mut intersections = HashMap::new();
        for (intersection_id, [l1, l2]) in intersection_markers {
            let this_edge = intersection_id;
            let twin_edge = self.mesh_ref.twin(this_edge);
            let quad = self.mesh_ref.quad(this_edge);

            // Find the adjacent intersections in l1
            let l1_next_intersection = Self::next(&loop_to_intersection_markers[&l1], this_edge);
            let l1_prev_intersection = Self::prev(&loop_to_intersection_markers[&l1], this_edge);

            // Find the adjacent intersections in l2
            let l2_next_intersection = Self::next(&loop_to_intersection_markers[&l2], this_edge);
            let l2_prev_intersection = Self::prev(&loop_to_intersection_markers[&l2], this_edge);

            let mut l1_edges_prev = Self::prev(&self.loops_ref[l1].edges, this_edge);
            let mut l1_edges_next = Self::next(&self.loops_ref[l1].edges, this_edge);
            // Either the prev or next is the twin edge, go one step further.
            if l1_edges_prev == twin_edge {
                l1_edges_prev = Self::prev(&self.loops_ref[l1].edges, l1_edges_prev);
            } else if l1_edges_next == twin_edge {
                l1_edges_next = Self::next(&self.loops_ref[l1].edges, l1_edges_next);
            }

            let mut l2_edges_prev = Self::prev(&self.loops_ref[l2].edges, this_edge);
            let mut l2_edges_next = Self::next(&self.loops_ref[l2].edges, this_edge);
            // Either the prev or next is the twin edge, go one step further.
            if l2_edges_prev == twin_edge {
                l2_edges_prev = Self::prev(&self.loops_ref[l2].edges, l2_edges_prev);
            } else if l2_edges_next == twin_edge {
                l2_edges_next = Self::next(&self.loops_ref[l2].edges, l2_edges_next);
            }

            // We can order the intersections based on the local ordering of the edges in the loops
            let ordered_adjacent_intersections = quad
                .iter()
                .filter_map(|&x| match x {
                    x if x == l1_edges_next => Some((l1, l1_next_intersection, Orientation::Forwards)),
                    x if x == l1_edges_prev => Some((l1, l1_prev_intersection, Orientation::Backwards)),
                    x if x == l2_edges_next => Some((l2, l2_next_intersection, Orientation::Forwards)),
                    x if x == l2_edges_prev => Some((l2, l2_prev_intersection, Orientation::Backwards)),
                    _ => None,
                })
                .collect_vec();
            assert!(ordered_adjacent_intersections.len() == 4);
            assert!(ordered_adjacent_intersections.iter().map(|x| x.1).collect::<HashSet<_>>().len() == 4);
            assert!(ordered_adjacent_intersections[0].0 != ordered_adjacent_intersections[1].0);
            assert!(ordered_adjacent_intersections[1].0 != ordered_adjacent_intersections[2].0);
            assert!(ordered_adjacent_intersections[2].0 != ordered_adjacent_intersections[3].0);
            assert!(ordered_adjacent_intersections[3].0 != ordered_adjacent_intersections[0].0);

            // Add the four adjacent intersections
            intersections.insert(
                this_edge,
                Intersection {
                    this: this_edge,
                    next: [
                        ordered_adjacent_intersections[0],
                        ordered_adjacent_intersections[1],
                        ordered_adjacent_intersections[2],
                        ordered_adjacent_intersections[3],
                    ],
                },
            );
        }

        // Create DCEL based on the intersections and loop regions
        // Construct all faces
        let edge_id_to_index: HashMap<EdgeID, usize> = intersections.keys().enumerate().map(|(i, &e)| (e, i)).collect();

        let mut edges = intersections
            .values()
            .flat_map(|this| this.next.iter().map(|next| (this.this, next.1)))
            .collect_vec();

        let mut faces = vec![];
        while let Some(start) = edges.pop() {
            let mut counter = 0;
            let mut face = vec![start.0, start.1];
            loop {
                let u = face[face.len() - 2];
                let v = face[face.len() - 1];
                // get all intersections that are adjacent to v
                let adj = intersections[&v].next;
                let u_index = adj.iter().position(|&(_, x, _)| x == u).unwrap();
                let w = adj[(u_index + 4 - 1) % 4].1;
                edges.retain(|e| !(e.0 == v && e.1 == w));
                if w == face[0] {
                    break;
                }
                counter += 1;
                if counter > 6 {
                    return Err(PropertyViolationError::FaceWithDegreeMoreThanSix);
                }
                face.push(w);
            }
            faces.push(face.iter().map(|&x| edge_id_to_index[&x]).collect_vec());
        }

        let (mut douconel, vmap, _) = LoopStructure::from_faces(&faces).unwrap();
        assert!(4 * douconel.vert_ids().len() == douconel.edge_ids().len());
        let intersection_ids = intersections.keys().copied().collect_vec();
        for vertex_id in douconel.vert_ids() {
            douconel.verts[vertex_id] = intersection_ids[vmap.get_by_right(&vertex_id).unwrap().to_owned()];
        }
        for edge_id in douconel.edge_ids() {
            let this = douconel.verts[douconel.root(edge_id)];
            let next = douconel.verts[douconel.toor(edge_id)];
            let (loop_id, _, orientation) = intersections[&this].next.iter().find(|&(_, x, _)| *x == next).unwrap().to_owned();
            douconel.edges[edge_id] = Segment { loop_id, orientation }
        }

        self.loop_structure = douconel;

        Ok(())
    }

    // TODO: function that re-orients loops such that acyclic level graphs may be possible (if it currently is not possible).
    // // First we assign to each loop segment a direction
    //     // We do this by first: building a bipartite graph of loop segment adjacency.
    //     // Then we find a labeling that is consistent and acyclic, using loop adjacency.

    //     // Create a bipartite graph of loop segment adjacency (for X/Y/Z)
    //     // Create a graph with a vertex for each loop segment
    //     // If the loop segments share a face (are adjacent), we add an edge between the two vertices
    //     // Also add edge for: twin, loop segments of other side of this loop
    //     for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
    //         let mut sides_graph = HashMap::new();

    //         for segment in self.segments_of_direction(direction) {
    //             let next_of_same_color = |id: SegmentID| {
    //                 let cand = self
    //                     .loop_structure
    //                     .outgoing(self.loop_structure.toor(id))
    //                     .iter()
    //                     .flat_map(|&x| {
    //                         vec![
    //                             (x, self.loops_ref[self.loop_structure.edges[x].loop_id].direction),
    //                             (
    //                                 self.loop_structure.twin(x),
    //                                 self.loops_ref[self.loop_structure.edges[self.loop_structure.twin(x)].loop_id].direction,
    //                             ),
    //                         ]
    //                     })
    //                     .collect_vec();
    //                 let cur_pos = cand.iter().position(|&(x, _)| x == id).unwrap();

    //                 cand.iter().cycle().skip(cur_pos + 1).find(|&&(x, dir)| dir == direction).unwrap().to_owned().0
    //             };

    //             let mut this_side = vec![segment];
    //             let mut next_seg = next_of_same_color(segment);
    //             while next_seg != segment {
    //                 this_side.push(next_seg);
    //                 next_seg = next_of_same_color(next_seg);
    //             }

    //             let other_side = this_side.iter().map(|&x| self.loop_structure.twin(x)).collect_vec();
    //             let neighbors_of_direction = self.filter_direction(&self.loop_structure.nexts(segment), direction);
    //             sides_graph.insert(segment, [other_side, neighbors_of_direction].concat());
    //         }

    //         for segment in self.segments_of_direction(direction) {
    //             for neighbor in sides_graph.get(&segment).unwrap().clone() {
    //                 let entry = sides_graph.entry(neighbor).or_insert(vec![]);
    //                 entry.push(segment);
    //             }
    //         }

    //         self.side_ccs[direction as usize] = hutspot::graph::find_ccs(&sides_graph.keys().copied().collect_vec(), |e| sides_graph[&e].clone());

    //         for component in &self.side_ccs[direction as usize] {
    //             // Find a labeling that is consistent and acyclic
    //             // We do this by finding a twocoloring of the bipartite graph
    //             let two_coloring = hutspot::graph::two_color::<_>(&component.iter().copied().collect_vec(), |e| sides_graph[&e].clone());
    //             if two_coloring.is_none() {
    //                 return Err(PropertyViolationError::NonTwoColorable);
    //             }
    //             let (upper_segments, lower_segments) = two_coloring.unwrap();

    //             let mut cur_score = 0.0;
    //             let mut flip_score = 0.0;
    //             for (&ls_id, s) in upper_segments
    //                 .iter()
    //                 .map(|ls_id| (ls_id, Vector3D::from(direction)))
    //                 .chain(lower_segments.iter().map(|ls_id| (ls_id, -Vector3D::from(direction))))
    //             {
    //                 let ls = &self.loop_structure.edges[ls_id];
    //                 let edges = vec![ls.between.clone()].into_iter().flatten();

    //                 for (edge1, edge2) in edges.tuple_windows() {
    //                     if self.mesh_ref.twin(edge1) == edge2 {
    //                         continue;
    //                     }
    //                     let u = self.mesh_ref.midpoint(edge1);
    //                     let v = self.mesh_ref.midpoint(edge2);
    //                     let face = self.mesh_ref.face(edge1);
    //                     assert!(face == self.mesh_ref.face(edge2));
    //                     let edge_normal = self.mesh_ref.normal(face);
    //                     let edge_direction = (v - u).normalize();
    //                     let edge_length = (v - u).norm();
    //                     let cross = edge_normal.cross(&edge_direction).normalize();
    //                     cur_score += cross.dot(&s) * edge_length;
    //                     flip_score += cross.dot(&-s) * edge_length;
    //                 }
    //             }

    //             if cur_score > flip_score {
    //                 for edge_id in upper_segments {
    //                     self.loop_structure.edges[edge_id].side = Some(Side::Upper);
    //                 }

    //                 for edge_id in lower_segments {
    //                     self.loop_structure.edges[edge_id].side = Some(Side::Lower);
    //                 }
    //             } else {
    //                 for edge_id in upper_segments {
    //                     self.loop_structure.edges[edge_id].side = Some(Side::Lower);
    //                 }

    //                 for edge_id in lower_segments {
    //                     self.loop_structure.edges[edge_id].side = Some(Side::Upper);
    //                 }
    //             }
    //         }
    //     }

    fn assign_subsurfaces(&mut self) {
        // Get all blocked edges (ALL LOOPS)
        let blocked: HashSet<_> = self
            .loop_structure
            .edge_ids()
            .into_iter()
            .flat_map(|segment_id| {
                let endpoints = self.segment_to_endpoints(segment_id);
                [
                    vec![endpoints.0],
                    vec![endpoints.1],
                    vec![self.mesh_ref.twin(endpoints.0)],
                    vec![self.mesh_ref.twin(endpoints.1)],
                    self.segment_to_edges_between(segment_id),
                ]
                .concat()
            })
            .flat_map(|edge_id| [edge_id, self.mesh_ref.twin(edge_id)])
            .map(|edge_id| self.mesh_ref.endpoints(edge_id))
            .collect();

        let vertex_neighbors: HashMap<VertID, Vec<VertID>> = self
            .mesh_ref
            .verts
            .keys()
            .map(|vertex| {
                (
                    vertex,
                    self.mesh_ref.neighbor_function_primal()(vertex)
                        .into_iter()
                        .filter(|&neighbor| !blocked.contains(&(vertex, neighbor)))
                        .collect_vec(),
                )
            })
            .collect();

        // Make a neighborhood function that blocks all edges that are part of the loop segments of this face
        let nfunction = |vertex| vertex_neighbors[&vertex].clone();
        // Find all connected components (should be equal to the number of faces)
        let ccs = hutspot::graph::find_ccs(&self.mesh_ref.verts.keys().collect_vec(), nfunction);
        assert!(ccs.len() == self.loop_structure.face_ids().len());

        // Every loop segment should be part of exactly TWO connected components (on both sides)
        let mut segment_to_ccs: HashMap<SegmentID, HashSet<usize>> = HashMap::new();
        for &segment_id in &self.loop_structure.edge_ids() {
            let edges_between = self.segment_to_edges_between(segment_id);
            for edge_between in edges_between {
                let endpoints = self.mesh_ref.endpoints(edge_between);
                let mut cc1 = ccs.iter().enumerate().filter(|(_, cc)| cc.contains(&endpoints.0)).map(|(id, _)| id);
                assert!(cc1.clone().count() == 1);
                let mut cc2 = ccs.iter().enumerate().filter(|(_, cc)| cc.contains(&endpoints.1)).map(|(id, _)| id);
                assert!(cc2.clone().count() == 1);

                let cc1_single = cc1.next().unwrap().to_owned();
                let cc2_single = cc2.next().unwrap().to_owned();
                assert!(cc1_single != cc2_single);

                segment_to_ccs.entry(segment_id).or_default().insert(cc1_single);
                segment_to_ccs.entry(segment_id).or_default().insert(cc2_single);
            }
        }

        // Assert that each loop segment is part of exactly two connected components
        assert!(segment_to_ccs.values().all(|ccs| ccs.len() == 2));

        let segment_to_ccs_tuplified = segment_to_ccs
            .into_iter()
            .map(|(k, v)| {
                let v_vec = v.into_iter().collect_vec();
                assert!(v_vec.len() == 2);
                (k, (v_vec[0], v_vec[1]))
            })
            .collect::<HashMap<_, _>>();

        // For every loop region, get the connected component that is shared among its loop segments
        for &face_id in &self.loop_structure.face_ids() {
            let loop_segments = self.loop_structure.edges(face_id);
            assert!(loop_segments.len() >= 3 && loop_segments.len() <= 6);

            // From an arbitrary loop segment, get its two ccs
            // It has to be one of these two, check with the other segments whether its cc1 or cc2
            let its_cc1 = loop_segments.iter().all(|&segment| {
                segment_to_ccs_tuplified[&segment].0 == segment_to_ccs_tuplified[&loop_segments[0]].0
                    || segment_to_ccs_tuplified[&segment].1 == segment_to_ccs_tuplified[&loop_segments[0]].0
            });

            let its_cc2 = loop_segments.iter().all(|&segment| {
                segment_to_ccs_tuplified[&segment].0 == segment_to_ccs_tuplified[&loop_segments[0]].1
                    || segment_to_ccs_tuplified[&segment].1 == segment_to_ccs_tuplified[&loop_segments[0]].1
            });
            assert!(its_cc1 ^ its_cc2);

            let cc = if its_cc1 {
                segment_to_ccs_tuplified[&loop_segments[0]].0
            } else {
                segment_to_ccs_tuplified[&loop_segments[0]].1
            };

            self.loop_structure.faces[face_id] = Region { verts: ccs[cc].clone() };
        }
    }

    fn assign_zones(&mut self) {
        // A zone is a collection of loop regions that are connected, and are bounded by only one type of loop segment (either X, Y, or Z).
        self.zones = SlotMap::with_key();

        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            // Get all loop regions
            let mut regions_queue = self.loop_structure.face_ids();

            // While there are loop regions left, grab a loop region, and find its corresponding zone
            while let Some(region) = regions_queue.pop() {
                // Traverse the adjacent regions until none are left
                let mut zone_regions = HashSet::new();
                let mut zone_queue = vec![region];
                let mut visited = HashSet::new();
                while let Some(region) = zone_queue.pop() {
                    if visited.contains(&region) {
                        continue;
                    }
                    visited.insert(region);

                    // Get all adjacent regions, that are connected by a loop segment that is NOT equal to `direction`
                    let adjacent_regions = self
                        .loop_structure
                        .edges(region)
                        .into_iter()
                        .filter(|&segment| self.segment_to_direction(segment) != direction)
                        .map(|segment| self.loop_structure.twin(segment))
                        .map(|segment| self.loop_structure.face(segment))
                        .collect_vec();

                    // Add the adjacent regions to the queue
                    zone_queue.extend(adjacent_regions.clone());
                    // Add the adjacent regions to the zone
                    zone_regions.extend(adjacent_regions.clone());
                }

                // Create a zone from the adjacent regions
                regions_queue.retain(|&region| !zone_regions.contains(&region));
                self.zones.insert(Zone {
                    direction,
                    regions: zone_regions,
                });
            }
        }
    }

    fn assign_level_graphs(&mut self) {
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let mut adjacency = HashMap::new();
            let mut adjacency_backwards = HashMap::new();

            let zones = self.zones.iter().filter(|(_, zone)| zone.direction == direction);

            let region_to_zone = zones
                .clone()
                .filter(|(_, zone)| zone.direction == direction)
                .flat_map(|(zone_id, zone)| zone.regions.iter().map(move |&region_id| (region_id, zone_id)))
                .collect::<HashMap<_, _>>();

            for (zone_id, zone) in zones {
                // Get all the loop regions of this zone
                let regions = &zone.regions;

                // Get all the loop segments (in direction) of this zone
                let segments = regions
                    .iter()
                    .flat_map(|&region_id| self.loop_structure.edges(region_id))
                    .filter(|&segment_id| {
                        self.segment_to_direction(segment_id) == direction && self.segment_to_orientation(segment_id) == Orientation::Forwards
                    });

                // Grab all adjacent zones (through these segments)
                let adjacent_zones = segments
                    .map(|segment_id| {
                        let twin_segment = self.loop_structure.twin(segment_id);
                        let adjacent_region_id = self.loop_structure.face(twin_segment);
                        region_to_zone[&adjacent_region_id]
                    })
                    .collect::<HashSet<_>>();

                for &adjacent_zone in &adjacent_zones {
                    let entry = adjacency_backwards.entry(adjacent_zone).or_insert_with(HashSet::new);
                    entry.insert(zone_id);
                }
                adjacency.insert(zone_id, adjacent_zones.into_iter().collect_vec());
            }

            self.level_graphs[direction as usize] = adjacency;
        }
    }

    fn verify_properties(&self) -> Result<(), PropertyViolationError<IntersectionID>> {
        // Definition 3.2. An oriented loop structure L is a polycube loop structure if:
        // 1. No three loops intersect at a single point.
        // 2. Each loop region is bounded by at least three loop segments.
        // 3. Within each loop region boundary, no two loop segments have the same axis label and side label.
        // 4. Each loop region has the topology of a disk.
        // 5. The level graphs are acyclic.

        // 1. is verified by construction, simply by the way we construct the loop structure.

        for face_id in self.loop_structure.face_ids() {
            let edges = self.loop_structure.edges(face_id);

            // Verify 2.
            if edges.len() < 3 {
                return Err(PropertyViolationError::FaceWithDegreeLessThanThree);
            }

            // Verify 3.
            for edge in edges {
                let mut label_count = [0; 6];
                let loop_id = self.loop_structure.edges[edge].loop_id;
                let direction = self.loops_ref[loop_id].direction;
                let orientation = self.loop_structure.edges[edge].orientation;
                match (direction, orientation) {
                    (PrincipalDirection::X, Orientation::Forwards) => label_count[0] += 1,
                    (PrincipalDirection::X, Orientation::Backwards) => label_count[1] += 1,
                    (PrincipalDirection::Y, Orientation::Forwards) => label_count[2] += 1,
                    (PrincipalDirection::Y, Orientation::Backwards) => label_count[3] += 1,
                    (PrincipalDirection::Z, Orientation::Forwards) => label_count[4] += 1,
                    (PrincipalDirection::Z, Orientation::Backwards) => label_count[5] += 1,
                }
                if label_count.iter().any(|&x| x > 1) {
                    return Err(PropertyViolationError::InvalidFaceBoundary);
                }
            }
        }

        // 4. must be verified: TODO

        // Verify 5.
        for level_graph in &self.level_graphs {
            let topological_sort = hutspot::graph::topological_sort::<ZoneID>(&level_graph.keys().copied().collect_vec(), |z| level_graph[&z].clone());
            if topological_sort.is_none() {
                return Err(PropertyViolationError::CyclicDependency);
            }
        }

        Ok(())
    }
}
