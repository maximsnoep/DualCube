use crate::elements::{Loop, LoopSegment, PrincipalDirection, Side, Subsurface, Zone};
use crate::EmbeddedMesh;
use douconel::douconel::{Douconel, EdgeID, Empty, FaceID, MeshError, VertID};
use douconel::douconel_embedded::{EmbeddedVertex, HasPosition};
use hutspot::geom::Vector3D;
use itertools::Itertools;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use serde::{Deserialize, Serialize};
use slotmap::{SecondaryMap, SlotMap};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

pub type Polycube = Douconel<PolycubeVertex, Vec<VertID>, VertID>;

// Embedded vertices (have a position)
#[derive(Default, Copy, Clone, Debug, Serialize, Deserialize)]
pub struct PolycubeVertex {
    // real space
    pub position: Vector3D,
    // pointer to the face of the loop structure
    pub pointer_loop_region: FaceID,
    // pointer to the corresponding vertex on the mesh
    pub pointer_primal_vertex: VertID,
}

impl HasPosition for PolycubeVertex {
    fn position(&self) -> Vector3D {
        self.position
    }
    fn set_position(&mut self, position: Vector3D) {
        self.position = position;
    }
}

type LoopStructure = Douconel<Intersection, LoopSegment, Subsurface>;

slotmap::new_key_type! {
    pub struct LoopID;
    pub struct ZoneID;
    pub struct IntersectionID;
}

// Dual structure is an "ABSTRACT PATH REPRESENTATION"
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Dual {
    mesh_ref: Arc<EmbeddedMesh>,
    loops: SlotMap<LoopID, Loop>,
    pub occupied: SecondaryMap<EdgeID, Vec<LoopID>>,
    loop_structure: LoopStructure,
    pub properties: Properties,
    pub side_ccs: [Vec<HashSet<EdgeID>>; 3],
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub enum PropertyViolationError {
    #[default]
    UnknownError,
    MissingDirection,
    FaceWithDegreeLessThanThree,
    FaceWithDegreeMoreThanSix,
    NonSimpleIntersection,
    NonTransversalIntersection,
    EqualColoredIntersection,
    SelfIntersection,
    NonTwoColorable,
    NonSeperatedDirection,
    CyclicDependency,
    LoopStructureError(MeshError),
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Properties {
    pub has_face_lower_degree: bool,
    pub has_transversal_intersection: bool,
    pub has_same_colored_intersection: bool,
    pub has_no_two_coloring: bool,
    pub has_incomplete_direction: bool,
    pub has_malformed_face: bool,
}

#[derive(Debug)]
pub struct IntersectionMarker {
    edge: EdgeID,
    loops: [LoopID; 2],
}

#[derive(Debug, Default, Copy, Clone, Serialize, Deserialize)]
pub struct Intersection {
    // An intersection is defined by the midpoint of two half-edges. At this point, two loops intersect. The intersection is defined by the lower half-edge.
    pub this: EdgeID,
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
            occupied: SecondaryMap::new(),
            properties: Properties::default(),
            side_ccs: [vec![], vec![], vec![]],
        }
    }

    pub const fn get_loop_structure(&self) -> &LoopStructure {
        &self.loop_structure
    }

    pub fn get_loop(&self, loop_id: LoopID) -> Option<&Loop> {
        self.loops.get(loop_id)
    }

    pub fn get_pairs_of_loop(&self, loop_id: LoopID) -> Vec<[EdgeID; 2]> {
        self.get_pairs_of_sequence(&self.get_loop(loop_id).unwrap().edges)
    }

    pub fn get_pairs_of_sequence(&self, sequence: &[EdgeID]) -> Vec<[EdgeID; 2]> {
        sequence
            .windows(2)
            .filter_map(|w| if self.mesh_ref.twin(w[0]) == w[1] { None } else { Some([w[0], w[1]]) })
            .collect()
    }

    pub fn del_loop(&mut self, loop_id: LoopID) {
        for &e in &self.get_loop(loop_id).unwrap().edges.clone() {
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

        for e in self.get_loop(loop_id).unwrap().edges.clone() {
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

    pub fn get_paths_in_edge(&self, edge: EdgeID) -> Option<Vec<LoopID>> {
        self.occupied.get(edge).cloned()
    }

    pub fn get_loop_ids(&self) -> Vec<LoopID> {
        self.loops.keys().collect()
    }

    pub fn get_loops(&self) -> Vec<&Loop> {
        self.loops.values().collect()
    }

    pub fn is_occupied(&self, [e1, e2]: [EdgeID; 2]) -> Option<LoopID> {
        if let Some(loops_e1) = self.occupied.get(e1) {
            if let Some(loops_e2) = self.occupied.get(e2) {
                for &loop_e1 in loops_e1 {
                    for &loop_e2 in loops_e2 {
                        if loop_e1 == loop_e2
                            && (self.get_loop(loop_e1).unwrap().contains_pair((e1, e2)) || self.get_loop(loop_e1).unwrap().contains_pair((e2, e1)))
                        {
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

    pub fn count_loops_in_direction(&self, direction: PrincipalDirection) -> usize {
        self.loops.iter().filter(|(_, l)| l.direction == direction).count()
    }

    pub fn loop_to_direction(&self, loop_id: LoopID) -> PrincipalDirection {
        self.get_loop(loop_id).unwrap().direction
    }

    pub fn segment_to_loop(&self, segment: EdgeID) -> LoopID {
        self.loop_structure.edges[segment].loop_id
    }

    pub fn segment_to_edges(&self, segment: EdgeID) -> Vec<EdgeID> {
        [
            vec![self.loop_structure.edges[segment].start],
            self.loop_structure.edges[segment].between.clone(),
            vec![self.loop_structure.edges[segment].end],
        ]
        .concat()
    }

    pub fn segment_to_direction(&self, segment: EdgeID) -> PrincipalDirection {
        self.loop_to_direction(self.segment_to_loop(segment))
    }

    pub fn segment_to_side(&self, segment: EdgeID, mask: [u32; 3]) -> Side {
        let cc = self.side_ccs[self.segment_to_direction(segment) as usize]
            .iter()
            .position(|cc| cc.contains(&segment))
            .unwrap() as u8;

        let side = self.loop_structure.edges[segment].side.clone().unwrap();

        if mask[self.segment_to_direction(segment) as usize] & (1 << cc) == 1 {
            side.flip()
        } else {
            side
        }
    }

    pub fn filter_direction(&self, selection: &[EdgeID], direction: PrincipalDirection) -> Vec<EdgeID> {
        selection
            .iter()
            .filter(|&segment| self.segment_to_direction(*segment) == direction)
            .copied()
            .collect()
    }

    pub fn segments_of_direction(&self, direction: PrincipalDirection) -> Vec<EdgeID> {
        self.filter_direction(&self.loop_structure.edge_ids(), direction)
    }

    pub fn primal(&self, side_mask: [u32; 3]) -> Result<Polycube, PropertyViolationError> {
        // Create zones
        let mut zones: SlotMap<ZoneID, Zone> = SlotMap::with_key();

        // Do this for each direction
        for this_direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let edges_of_this_direction: HashSet<EdgeID> = self
                .segments_of_direction(this_direction)
                .into_iter()
                .flat_map(|segment| self.segment_to_edges(segment))
                .collect();

            // Make a neighborhood function that blocks all edges that are part of the loop segments of this face
            let blocked_function = |vertex: VertID| {
                self.mesh_ref.neighbor_function_primal()(vertex)
                    .into_iter()
                    .filter(|&neighbor| {
                        let (edge1, edge2) = self.mesh_ref.edge_between_verts(vertex, neighbor).unwrap();
                        !edges_of_this_direction.contains(&edge1) && !edges_of_this_direction.contains(&edge2)
                    })
                    .collect()
            };

            // Find the connected components (should more than 2(?)!)
            let ccs = hutspot::graph::find_ccs(&self.mesh_ref.vert_ids(), blocked_function);
            if ccs.len() < 2 {
                println!("Connected components less than 2 for direction {:?}", this_direction);
                return Err(PropertyViolationError::NonSeperatedDirection);
            }

            for cc in ccs {
                zones.insert(Zone {
                    direction: this_direction,
                    subsurfaces: cc
                        .iter()
                        .map(|&v| self.loop_structure.faces.iter().find(|(_, f)| f.verts.contains(&v)).unwrap().0)
                        .collect(),
                    coordinate: None,
                });
            }
        }

        let segment_to_zone_map = self
            .loop_structure
            .edge_ids()
            .into_iter()
            .map(|segment| {
                let zone = zones
                    .iter()
                    .find(|(_, z)| z.subsurfaces.contains(&self.loop_structure.face(segment)) && z.direction == self.segment_to_direction(segment))
                    .unwrap()
                    .0;
                (segment, zone)
            })
            .collect::<HashMap<_, _>>();

        // Create a directed graph on the zones
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let zones_of_dir = zones.iter().filter(|(_, z)| z.direction == direction).collect_vec();

            let mut adjacency = HashMap::new();
            let mut adjacency_backwards = HashMap::new();

            for &(zone_id, zone) in &zones_of_dir {
                // Get all Upper loop segments of the zone in this direction
                let adjacent_zones: HashSet<ZoneID> = segment_to_zone_map
                    .iter()
                    .filter(|&(&segment, &this_zone_id)| {
                        this_zone_id == zone_id && self.segment_to_side(segment, side_mask) == Side::Upper && self.segment_to_direction(segment) == direction
                    })
                    // Map to the other zone (by following the twin segment)
                    .map(|(&segment, _)| {
                        let twin_segment = self.loop_structure.twin(segment);
                        segment_to_zone_map.get(&twin_segment).cloned().unwrap()
                    })
                    .collect();

                for &adjacent_zone in &adjacent_zones {
                    let entry = adjacency_backwards.entry(adjacent_zone).or_insert_with(HashSet::new);
                    entry.insert(zone_id);
                }

                adjacency.insert(zone_id, adjacent_zones);
            }

            let topological_sort = hutspot::graph::topological_sort::<ZoneID>(&zones_of_dir.into_iter().map(|(id, _)| id).collect_vec(), |z| {
                adjacency.get(&z).cloned().unwrap_or_default().into_iter().collect_vec()
            });

            if topological_sort.is_none() {
                println!("Topological sort failed for direction {:?}", direction);
                return Err(PropertyViolationError::CyclicDependency);
            }

            for zone_id in topological_sort.unwrap() {
                let dependencies = adjacency_backwards
                    .get(&zone_id)
                    .cloned()
                    .unwrap_or_default()
                    .iter()
                    .filter_map(|&z| zones[z].coordinate)
                    .collect_vec();

                zones[zone_id].coordinate = if dependencies.is_empty() {
                    Some(0.0)
                } else {
                    Some(dependencies.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() + 0.1)
                };
            }
        }

        // Each face to an int
        let face_to_int: HashMap<FaceID, usize> = self.loop_structure.faces.keys().enumerate().map(|(i, f)| (f, i)).collect();

        // Create the dual (primal)
        // By creating the primal faces
        let primal_faces = self
            .loop_structure
            .verts
            .iter()
            .map(|(id, _)| {
                self.loop_structure
                    .star(id)
                    .iter()
                    .map(|f| face_to_int.get(f).unwrap().to_owned())
                    .rev()
                    .collect_vec()
            })
            .collect_vec();

        let vertex_positions = self
            .loop_structure
            .faces
            .iter()
            .map(|(subsurface_id, _)| {
                // Find the three zones that this subsurface is part of (X/Y/Z)
                let this_zones = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|d| {
                    zones
                        .iter()
                        .find(|(_, z)| z.subsurfaces.contains(&subsurface_id) && z.direction == d)
                        .unwrap()
                        .0
                });

                Vector3D::new(
                    zones[this_zones[0]].coordinate.unwrap(),
                    zones[this_zones[1]].coordinate.unwrap(),
                    zones[this_zones[2]].coordinate.unwrap(),
                )
            })
            .collect_vec();

        let (mut primal, vmap, fmap) = Polycube::from_embedded_faces(&primal_faces, &vertex_positions).unwrap();

        for (vert_id, vert_obj) in &mut primal.verts {
            vert_obj.pointer_loop_region = self.loop_structure.faces.keys().collect_vec()[vmap.get_by_right(&vert_id).unwrap().to_owned()];
        }

        for (vert_id, vert_obj) in &mut primal.verts {
            vert_obj.pointer_primal_vertex = self.loop_structure.faces[vert_obj.pointer_loop_region].verts.iter().next().unwrap().to_owned();
        }

        let mut edge_to_obj = HashMap::new();
        for (edge_id, edge_obj) in &primal.edges {
            let (u_new, v_new) = primal.endpoints(edge_id);
            let (u, v) = (primal.verts[u_new].pointer_primal_vertex, primal.verts[v_new].pointer_primal_vertex);
            // shortest path from u to v
            let path = hutspot::graph::find_shortest_path(
                u,
                v,
                self.mesh_ref.neighbor_function_primal(),
                self.mesh_ref.weight_function_euclidean(),
                &mut HashMap::new(),
            );
            edge_to_obj.insert(edge_id, path.unwrap().0);
        }

        for (edge_id, edge_obj) in &mut primal.edges {
            *edge_obj = edge_to_obj.get(&edge_id).unwrap().clone();
        }

        for (face_id, face_obj) in &mut primal.faces {
            *face_obj = self.loop_structure.verts.keys().collect_vec()[fmap.get_by_right(&face_id).unwrap().to_owned()];
        }

        Ok(primal)
    }

    // Returns error if:
    //    1. Self-intersections or transversal intersections
    //    2. More than two loops intersect at each intersection (resulting in vertices of degree 4)
    pub fn intersections(&self) -> Result<HashMap<EdgeID, Intersection>, PropertyViolationError> {
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
                    .map(|(edge, set)| IntersectionMarker { edge, loops: [set[0], set[1]] })
                    .collect_vec();
                (loop_id, path_intersections)
            })
            .collect();

        // For each intersection:
        //   We find its (4) next intersections by following its associated loops
        let mut intersections = HashMap::new();
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

                        if self.loops[intersection_marker.loops[0]].contains_pair((edge, next_edge)) {
                            Some((next_edge, intersection_marker.loops[0], 1))
                        } else if self.loops[intersection_marker.loops[0]].contains_pair((next_edge, edge)) {
                            Some((next_edge, intersection_marker.loops[0], -1))
                        } else if self.loops[intersection_marker.loops[1]].contains_pair((edge, next_edge)) {
                            Some((next_edge, intersection_marker.loops[1], 1))
                        } else if self.loops[intersection_marker.loops[1]].contains_pair((next_edge, edge)) {
                            Some((next_edge, intersection_marker.loops[1], -1))
                        } else {
                            None
                        }
                    })
                    .map(|(next_edge, follow_loop, direction)| {
                        let follow_loop_intersections = loop_to_intersections.get(&follow_loop).unwrap();

                        // Find current intersection in `follow_loop`
                        let this_intersection = follow_loop_intersections.iter().position(|x| x.edge == this_edge).unwrap();

                        // Find the next intersection in `follow_loop`
                        let next_intersection = follow_loop_intersections
                            [((this_intersection + follow_loop_intersections.len()) as i32 + direction) as usize % follow_loop_intersections.len()]
                        .edge;

                        (next_edge, follow_loop, next_intersection, direction)
                    })
                    .collect_vec();

                assert!(!(nexts.len() < 4), "Should simply be impossible {nexts:?} (len: {})", nexts.len());
                if nexts.len() > 4 {
                    return Err(PropertyViolationError::NonSimpleIntersection);
                }
                for next in &nexts {
                    println!("next.0 == next.2  ????? {next:?}");
                    if next.0 == next.2 {
                        return Err(PropertyViolationError::UnknownError);
                    }
                }

                intersections.insert(
                    this_edge,
                    Intersection {
                        this: this_edge,
                        next: [nexts[0], nexts[1], nexts[2], nexts[3]],
                    },
                );
            }
        }

        Ok(intersections)
    }

    pub fn loop_segments(&self, intersections: &HashMap<EdgeID, Intersection>) -> Vec<LoopSegment> {
        // Construct all loop segments
        // For each intersection:
        //   For each next intersection:
        //     We create a loop segment, that connects the two intersections
        intersections
            .iter()
            .flat_map(|(this_id, intersection)| {
                intersection
                    .next
                    .iter()
                    .map(|(_, loop_id, next_id, direction)| (*this_id, *next_id, *loop_id, *direction))
            })
            .map(|(this_id, next_id, loop_id, direction)| {
                let this_intersection = intersections.get(&this_id).unwrap();
                let next_intersection = intersections.get(&next_id).unwrap();

                let this_intersection_pointer_to_next = this_intersection
                    .next
                    .into_iter()
                    .find(|&(_, next_loop_id, next_intersection_id, _)| next_intersection_id == next_id && next_loop_id == loop_id)
                    .unwrap();

                let next_intersection_pointer_to_this = next_intersection
                    .next
                    .into_iter()
                    .find(|&(_, next_loop_id, next_intersection_id, _)| next_intersection_id == this_id && next_loop_id == loop_id)
                    .unwrap();

                assert!(this_intersection_pointer_to_next.1 == next_intersection_pointer_to_this.1);

                let between = if direction == 1 {
                    self.get_loop(loop_id)
                        .unwrap()
                        .between(this_intersection_pointer_to_next.0, next_intersection_pointer_to_this.0)
                } else {
                    self.get_loop(loop_id)
                        .unwrap()
                        .between(next_intersection_pointer_to_this.0, this_intersection_pointer_to_next.0)
                        .into_iter()
                        .rev()
                        .collect_vec()
                };

                LoopSegment {
                    loop_id,
                    start: this_id,
                    end: next_id,
                    between,
                    side: None,
                }
            })
            .collect()
    }

    fn next_intersection(&self, (this, next): (EdgeID, EdgeID), intersections: &HashMap<EdgeID, Intersection>) -> EdgeID {
        // Given two adjacent intersections (this, next), we find the third intersection in clockwise order.

        // Find the local edge that connects this and next
        let edge_to_this = intersections.get(&next).unwrap().next.iter().find(|&&(_, _, x, _)| x == this).unwrap().0;

        // Find the local edge that connects next and third (the one with the smallest clockwise angle)
        intersections
            .get(&next)
            .unwrap()
            .next
            .into_iter()
            .filter(|&(candidate_edge, _, _, _)| candidate_edge != edge_to_this)
            .map(|(candidate_edge, _, candidate_x, _)| {
                let clockwise_angle = hutspot::geom::calculate_clockwise_angle(
                    self.mesh_ref.midpoint(next),
                    self.mesh_ref.midpoint(edge_to_this),
                    self.mesh_ref.midpoint(candidate_edge),
                    self.mesh_ref.normal(self.mesh_ref.face(next)),
                );
                (candidate_x, clockwise_angle)
            })
            .sorted_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .next()
            .unwrap()
            .0
    }

    // Returns error if faces have more than 6 edges (we know the face degree is at most 6, so we can early stop, and we also want to prevent infinite loops / malformed faces)
    pub fn loop_faces(&self, intersections: &HashMap<EdgeID, Intersection>) -> Result<Vec<Vec<usize>>, PropertyViolationError> {
        // Construct all faces
        let fn_to_id = |x: EdgeID| intersections.keys().position(|&y| y == x).unwrap();

        let mut edges = intersections
            .values()
            .flat_map(|this| this.next.iter().map(|next| (this.this, next.2)))
            .collect_vec();

        let mut faces = vec![];
        while let Some(start) = edges.pop() {
            let mut counter = 0;
            let mut face = vec![start.0, start.1];
            loop {
                let u = face[face.len() - 2];
                let v = face[face.len() - 1];
                let w = self.next_intersection((u, v), intersections);

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
            faces.push(face.iter().map(|&x| fn_to_id(x)).collect_vec());
        }

        Ok(faces)
    }

    pub fn assign_loop_structure(
        &self,
        faces: &[Vec<usize>],
        intersections: &HashMap<EdgeID, Intersection>,
        segments: &[LoopSegment],
    ) -> Result<LoopStructure, PropertyViolationError> {
        // Create douconel based on these faces
        let loop_structure_maybe = LoopStructure::from_faces(faces);
        if let Err(err) = loop_structure_maybe {
            return Err(PropertyViolationError::LoopStructureError(err));
        }
        let (mut loop_structure, vmap, _) = loop_structure_maybe.unwrap();

        println!("found loop struct., doing other stuff now");

        let intersection_ids = intersections.keys().copied().collect_vec();
        for (vertex_id, vertex_obj) in &mut loop_structure.verts {
            let intersection_id = intersection_ids[vmap.get_by_right(&vertex_id).unwrap().to_owned()];
            intersections.get(&intersection_id).unwrap().clone_into(vertex_obj);
        }

        let mut endpoints_to_segment = HashMap::new();
        for segment in segments {
            endpoints_to_segment.insert((segment.start, segment.end), segment);
        }

        let loop_structure_c = loop_structure.clone();
        for (edge_id, edge_obj) in &mut loop_structure.edges {
            let start = loop_structure.verts[loop_structure_c.root(edge_id)].this;
            let end = loop_structure.verts[loop_structure_c.toor(edge_id)].this;
            *edge_obj = endpoints_to_segment.get(&(start, end)).copied().unwrap().clone();
        }

        // Map from some verts to adjacent loops(segments)
        let mut vert_to_loop_segments: HashMap<VertID, HashSet<_>> = HashMap::new();
        for (ls_id, ls) in &loop_structure.edges {
            // remove first and last element (intersection points)
            let between = &ls.between[1..ls.between.len() - 1];

            for &edge_id in between {
                let endpoints = self.mesh_ref.endpoints(edge_id);
                vert_to_loop_segments.entry(endpoints.0).or_default().insert(ls_id);
                vert_to_loop_segments.entry(endpoints.1).or_default().insert(ls_id);
            }
        }

        let face_to_subsurface = loop_structure
            .faces
            .keys()
            .collect_vec()
            .par_iter()
            .map(|&face_id| {
                let blocked: HashSet<EdgeID> = loop_structure
                    .edges(face_id)
                    .into_iter()
                    .map(|edge_id| loop_structure.edges[edge_id].clone())
                    .flat_map(|ls| [ls.between, vec![ls.start, ls.end]].concat())
                    .collect();

                // Our loopsegments:
                let loop_segments: HashSet<_> = loop_structure.edges(face_id).iter().flat_map(|&e| [e, loop_structure.twin(e)]).collect();

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
                let ccs = hutspot::graph::find_ccs(&self.mesh_ref.verts.keys().collect_vec(), nfunction);

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
                        color: hutspot::color::hsl_to_rgb(rand::random::<f32>() * 360., 0.5, 0.5).into(),
                    };
                    (face_id, subsurface)
                } else {
                    let subsurface = Subsurface {
                        verts: ccs[1].iter().copied().collect(),
                        color: hutspot::color::hsl_to_rgb(rand::random::<f32>() * 360., 0.5, 0.5).into(),
                    };
                    (face_id, subsurface)
                }
            })
            .collect::<Vec<_>>();

        for (face_id, subsurface) in face_to_subsurface {
            loop_structure.faces[face_id] = subsurface;
        }

        Ok(loop_structure)
    }

    pub fn build_loop_structure(&mut self) -> Result<Polycube, PropertyViolationError> {
        // Create the following "map"
        // List of all intersections
        // At each intersection, list all path segments that intersect there
        // We define both: the "next" intersection, and the local "next" edge
        // Using this information, we should be able to create an embedded graph (HEDS)

        self.properties = Properties::default();

        if self.count_loops_in_direction(PrincipalDirection::X) == 0
            || self.count_loops_in_direction(PrincipalDirection::Y) == 0
            || self.count_loops_in_direction(PrincipalDirection::Z) == 0
        {
            return Err(PropertyViolationError::MissingDirection);
        }

        self.loop_structure = LoopStructure::default();

        println!("Computing intersections...");
        let intersections = self.intersections()?;

        println!("Constructing faces...");
        let faces = self.loop_faces(&intersections)?;

        println!("Constructing segments...");
        let loop_segments = self.loop_segments(&intersections);

        println!("Constructing loop structure...");
        self.loop_structure = self.assign_loop_structure(&faces, &intersections, &loop_segments)?;

        // Characterization
        println!("Basic properties...");
        for face_id in self.loop_structure.face_ids() {
            let edges = self.loop_structure.edges(face_id);

            // Must be at least degree 3
            if edges.len() < 3 {
                return Err(PropertyViolationError::FaceWithDegreeLessThanThree);
            }

            // Non-transversal intersection not allowed
            for (this, next) in hutspot::math::wrap_pairs(&edges) {
                if self.segment_to_loop(this) == self.segment_to_loop(next) {
                    return Err(PropertyViolationError::NonTransversalIntersection);
                }
            }

            // An intersection with its own direction is not allowed
            for (this, next) in hutspot::math::wrap_pairs(&edges) {
                if self.segment_to_direction(this) == self.segment_to_direction(next) {
                    println!("Face {face_id} has an intersection with its own direction");
                    return Err(PropertyViolationError::EqualColoredIntersection);
                }
            }
        }

        // First we assign to each loop segment a direction
        // We do this by first: building a bipartite graph of loop segment adjacency.
        // Then we find a labeling that is consistent and acyclic, using loop adjacency.

        // Create a bipartite graph of loop segment adjacency (for X/Y/Z)
        // Create a graph with a vertex for each loop segment
        // If the loop segments share a face (are adjacent), we add an edge between the two vertices
        // Also add edge for: twin, loop segments of other side of this loop
        println!("Assigning sides...");
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let mut sides_graph = HashMap::new();

            for segment in self.segments_of_direction(direction) {
                let neighbors = self.loop_structure.nexts(segment);
                let neighbors_of_direction = self.filter_direction(&neighbors, direction);

                // for genus=0 ?
                // assert!(neighbors_of_direction.len() <= 1);

                let next_of_same_color = |id: EdgeID| {
                    let cand = self
                        .loop_structure
                        .outgoing(self.loop_structure.toor(id))
                        .iter()
                        .flat_map(|&x| {
                            vec![
                                (x, self.loops[self.loop_structure.edges[x].loop_id].direction),
                                (
                                    self.loop_structure.twin(x),
                                    self.loops[self.loop_structure.edges[self.loop_structure.twin(x)].loop_id].direction,
                                ),
                            ]
                        })
                        .collect_vec();
                    let cur_pos = cand.iter().position(|&(x, _)| x == id).unwrap();

                    cand.iter().cycle().skip(cur_pos + 1).find(|&&(x, dir)| dir == direction).unwrap().to_owned().0
                };

                let mut this_side = vec![segment];
                let mut next_seg = next_of_same_color(segment);
                while next_seg != segment {
                    this_side.push(next_seg);
                    next_seg = next_of_same_color(next_seg);
                }

                let other_side = this_side.iter().map(|&x| self.loop_structure.twin(x)).collect_vec();

                let neighbors = [neighbors_of_direction, other_side].concat();

                sides_graph.insert(segment, neighbors.clone());

                // insert or update
                for neighbor in neighbors {
                    let entry = sides_graph.entry(neighbor).or_insert(vec![]);
                    entry.push(segment);
                }
            }

            self.side_ccs[direction as usize] = hutspot::graph::find_ccs(&sides_graph.keys().copied().collect_vec(), |e: EdgeID| sides_graph[&e].clone());

            println!("DIR: {}: {:?}", direction, self.side_ccs[direction as usize]);

            for component in &self.side_ccs[direction as usize] {
                // Find a labeling that is consistent and acyclic
                // We do this by finding a twocoloring of the bipartite graph
                println!("cc: {direction}: {component:?}");

                if let Some(two_coloring) = hutspot::graph::two_color::<EdgeID>(&component.iter().copied().collect_vec(), |e: EdgeID| sides_graph[&e].clone()) {
                    for edge_id in two_coloring.0 {
                        self.loop_structure.edges[edge_id].side = Some(Side::Upper);
                    }

                    for edge_id in two_coloring.1 {
                        self.loop_structure.edges[edge_id].side = Some(Side::Lower);
                    }
                } else {
                    return Err(PropertyViolationError::NonTwoColorable);
                }
            }
        }

        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        println!("Computing primal...");
        let polycube = self.primal([0, 0, 0])?;

        Ok(polycube)
    }
}
