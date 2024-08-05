use crate::elements::{Loop, LoopSegment, PrincipalDirection, Side, Subsurface, Zone};
use crate::EmbeddedMesh;
use douconel::douconel::{Douconel, EdgeID, FaceID, MeshError, VertID};
use douconel::douconel_embedded::HasPosition;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};
use slotmap::{SecondaryMap, SlotMap};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

pub type Polycube = Douconel<PolycubeVertex, Vec<VertID>, VertID>;

#[derive(Default, Clone, Debug, Serialize, Deserialize, PartialEq, Eq, Hash, Copy)]
pub enum NodeType {
    Vertex(VertID),
    Face(FaceID),
    #[default]
    Phantom,
}

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
    zones: SlotMap<ZoneID, Zone>,
    pub side_ccs: [Vec<HashSet<EdgeID>>; 3],
    pub granulated_mesh: Option<EmbeddedMesh>,
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
    NonConnectedComponents,
    NonSeperatedDirection,
    CyclicDependency,
    LoopStructureError(MeshError),
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
            zones: SlotMap::with_key(),
            occupied: SecondaryMap::new(),
            side_ccs: [vec![], vec![], vec![]],
            granulated_mesh: None,
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

    pub fn assign_zones(&mut self) {
        // A zone is a collection of loop segments that are connected, and have the same direction (and side)

        // Create zones
        self.zones = SlotMap::with_key();

        // Grab all loop segments
        let mut loop_segments_queue = self.loop_structure.edge_ids();

        // While there are loop segments left, grab a loop segment, and find its corresponding zone
        while let Some(segment) = loop_segments_queue.pop() {
            // Find the direction of the segment
            let this_direction = self.segment_to_direction(segment);

            // Get the whole subsurface of the segment
            let subsurface = self.loop_structure.face(segment);

            let mut edges_of_this_zone = vec![];
            let mut subsurface_queue = vec![subsurface];
            let mut neighboring_subsurfaces = HashSet::new();
            // Do traversal of neighboring subsurfaces.
            while let Some(subsurface) = subsurface_queue.pop() {
                neighboring_subsurfaces.insert(subsurface);

                // Grab the segments of this subsurface that are in the same direction
                let edges_of_this_subsurface = self
                    .loop_structure
                    .edges(subsurface)
                    .into_iter()
                    .filter(|&segment| self.segment_to_direction(segment) == this_direction && !edges_of_this_zone.contains(&segment))
                    .collect_vec();
                edges_of_this_zone.extend(edges_of_this_subsurface.clone());

                // Get neighboring subsurfaces of this subsurface, that are connected only by a loop segment != this_direction
                let new_neighbors = self
                    .loop_structure
                    .edges(subsurface)
                    .into_iter()
                    .filter(|&segment| self.segment_to_direction(segment) != this_direction)
                    .map(|segment| self.loop_structure.twin(segment))
                    .map(|segment| self.loop_structure.face(segment))
                    .filter(|&subsurface| !neighboring_subsurfaces.contains(&subsurface))
                    .collect_vec();

                neighboring_subsurfaces.extend(new_neighbors.clone());
                subsurface_queue.extend(new_neighbors.clone());
            }

            // Remove the segments of this zone from the queue
            loop_segments_queue.retain(|&segment| !edges_of_this_zone.contains(&segment));

            // Create a zone from the neighboring subsurfaces
            self.zones.insert(Zone {
                direction: this_direction,
                subsurfaces: neighboring_subsurfaces,
                coordinate: None,
            });
        }
    }

    // Get zones and topological sort on the zones.
    pub fn zone_graph(&mut self, side_mask: [u32; 3]) -> Result<(), PropertyViolationError> {
        let mut adjacency = HashMap::new();
        let mut adjacency_backwards = HashMap::new();

        // For each segment, find the zone that it belongs to
        let segment_to_zone_map = self
            .loop_structure
            .edge_ids()
            .into_iter()
            .map(|segment| {
                let zone = self
                    .zones
                    .iter()
                    .find(|(_, z)| z.subsurfaces.contains(&self.loop_structure.face(segment)) && z.direction == self.segment_to_direction(segment))
                    .unwrap()
                    .0;
                (segment, zone)
            })
            .collect::<HashMap<_, _>>();

        // Create a directed graph on the zones
        for this_direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let zones_of_this_direction = self.zones.iter().filter(|(_, z)| z.direction == this_direction);

            for (zone_id, zone) in zones_of_this_direction {
                // Get all Upper loop segments of the zone in this direction
                let adjacent_zones: HashSet<ZoneID> = segment_to_zone_map
                    .iter()
                    .filter(|&(&segment, &this_zone_id)| {
                        this_zone_id == zone_id
                            && self.segment_to_side(segment, side_mask) == Side::Lower
                            && self.segment_to_direction(segment) == this_direction
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
        }

        let topological_sort = hutspot::graph::topological_sort::<ZoneID>(&self.zones.clone().into_iter().map(|(id, _)| id).collect_vec(), |z| {
            adjacency.get(&z).cloned().unwrap_or_default().into_iter().collect_vec()
        });

        if topological_sort.is_none() {
            return Err(PropertyViolationError::CyclicDependency);
        }

        for &zone_id in &topological_sort.unwrap() {
            let dependencies = adjacency_backwards
                .get(&zone_id)
                .cloned()
                .unwrap_or_default()
                .iter()
                .filter_map(|&z| self.zones[z].coordinate)
                .collect_vec();

            self.zones[zone_id].coordinate = if dependencies.is_empty() {
                Some(0.0)
            } else {
                Some(dependencies.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() + 0.1)
            };
        }

        Ok(())
    }

    pub fn primal(&mut self) -> Polycube {
        let primal_vertices = self.loop_structure.face_ids();
        let primal_faces = self.loop_structure.vert_ids();

        // Each face to an int
        let vert_to_int: HashMap<FaceID, usize> = primal_vertices.clone().into_iter().enumerate().map(|(i, f)| (f, i)).collect();

        // Create the dual (primal)
        // By creating the primal faces
        let faces = primal_faces
            .into_iter()
            .map(|dual_vert_id| self.loop_structure.star(dual_vert_id).into_iter().rev().collect_vec())
            .collect_vec();
        let int_faces = faces.iter().map(|face| face.iter().map(|vert| vert_to_int[vert]).collect_vec()).collect_vec();

        let vertex_positions = primal_vertices
            .clone()
            .into_iter()
            .map(|subsurface_id| {
                // Find the three zones that this subsurface is part of (X/Y/Z)
                let coordinates = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|d| {
                    self.zones
                        .iter()
                        .find(|(_, z)| z.subsurfaces.contains(&subsurface_id) && z.direction == d)
                        .unwrap()
                        .1
                        .coordinate
                        .unwrap()
                });
                Vector3D::new(coordinates[0], coordinates[1], coordinates[2])
            })
            .collect_vec();

        let (mut primal, vmap, fmap) = Polycube::from_embedded_faces(&int_faces, &vertex_positions).unwrap();

        for (vert_id, vert_obj) in &mut primal.verts {
            vert_obj.pointer_loop_region = primal_vertices[vmap.get_by_right(&vert_id).unwrap().to_owned()];
        }

        let mut zone_to_coordinate = HashMap::new();

        // for each zone, find a coordinate that minimizes Hausdorf distance to each subsurface in the zone
        for (zone_id, zone_obj) in &self.zones {
            let direction_of_zone = zone_obj.direction;
            let subsurfaces_in_this_zone = zone_obj.subsurfaces.clone();

            // get all DIRECTION coordinates of the vertices of the subsurfaces in this zone
            let coordinates = subsurfaces_in_this_zone
                .iter()
                .map(|&subsurface_id| {
                    self.loop_structure.faces[subsurface_id]
                        .verts
                        .iter()
                        .map(|&vert_id| self.mesh_ref.position(vert_id)[direction_of_zone as usize])
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

        for (subsurface_id) in primal_vertices {
            let coordinates = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|d| {
                zone_to_coordinate[&self
                    .zones
                    .iter()
                    .find(|(_, z)| z.subsurfaces.contains(&subsurface_id) && z.direction == d)
                    .unwrap()
                    .0]
            });
            let target = Vector3D::new(coordinates[0], coordinates[1], coordinates[2]);

            let vertices_in_subsurface = self.loop_structure.faces[subsurface_id].verts.clone();
            let best_vertex_in_subsurface = vertices_in_subsurface
                .iter()
                .map(|&v| (v, self.mesh_ref.position(v)))
                .min_by(|(_, a), (_, b)| (a - target).norm().partial_cmp(&(b - target).norm()).unwrap())
                .unwrap()
                .0;

            let primal_vert_id = primal.verts.iter().find(|(_, v)| v.pointer_loop_region == subsurface_id).unwrap().0;
            primal.verts[primal_vert_id].pointer_primal_vertex = best_vertex_in_subsurface;
        }

        for (face_id, face_obj) in &mut primal.faces {
            *face_obj = self.loop_structure.verts.keys().collect_vec()[fmap.get_by_right(&face_id).unwrap().to_owned()];
        }

        primal
    }

    fn place_paths(&mut self, primal: &mut Polycube) -> EmbeddedMesh {
        let mut granulated_mesh = self.mesh_ref.as_ref().clone();

        let mut edge_to_obj = HashMap::new();
        for (edge_id, edge_obj) in &primal.edges {
            let (u_new, v_new) = primal.endpoints(edge_id);
            let (u, v) = (primal.verts[u_new].pointer_primal_vertex, primal.verts[v_new].pointer_primal_vertex);

            // TODO:
            // block previously used vertices...
            // block previously used edges...
            // block all primals (cannot go over primal vertex)
            // shortest path from u to v
            let n_function = |node: NodeType| match node {
                NodeType::Face(f_id) => {
                    let f_neighbors: Vec<NodeType> = granulated_mesh.fneighbors(f_id).into_iter().map(NodeType::Face).collect_vec();
                    let v_neighbors = granulated_mesh.corners(f_id).into_iter().map(NodeType::Vertex).collect_vec();
                    [v_neighbors, f_neighbors].concat()
                }
                NodeType::Vertex(v_id) => {
                    let v_neighbors = granulated_mesh.vneighbors(v_id).into_iter().map(NodeType::Vertex).collect_vec();
                    let f_neighbors = granulated_mesh.star(v_id).into_iter().map(NodeType::Face).collect_vec();
                    [v_neighbors, f_neighbors].concat()
                }
                NodeType::Phantom => vec![],
            };

            let nodetype_to_pos = |node: NodeType| match node {
                NodeType::Face(f_id) => granulated_mesh.centroid(f_id),
                NodeType::Vertex(v_id) => granulated_mesh.position(v_id),
                NodeType::Phantom => unreachable!(),
            };

            let w_function = |a: NodeType, b: NodeType| {
                let a_pos = nodetype_to_pos(a);
                let b_pos = nodetype_to_pos(b);
                OrderedFloat(a_pos.metric_distance(&b_pos))
            };

            let mut granulated_path = vec![];
            if let Some((path, _)) = hutspot::graph::find_shortest_path(NodeType::Vertex(u), NodeType::Vertex(v), n_function, w_function, &mut HashMap::new()) {
                let mut last_f_ids_maybe: Option<[FaceID; 3]> = None;
                for node in path {
                    match node {
                        NodeType::Vertex(v_id) => {
                            granulated_path.push(v_id);
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
                                            granulated_path.push(mid_v_id);
                                        }
                                    }
                                }
                            }

                            last_f_ids_maybe = Some(new_f_ids);
                            granulated_path.push(new_v_id);
                        }
                        NodeType::Phantom => unreachable!(),
                    }
                }
            };

            println!("granulated path: {granulated_path:?}");

            edge_to_obj.insert(edge_id, granulated_path);
        }

        for (edge_id, edge_obj) in &mut primal.edges {
            edge_obj.clone_from(edge_to_obj.get(&edge_id).unwrap());
        }

        granulated_mesh
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

    pub fn verify_properties_and_assign_sides(&mut self) -> Result<(), PropertyViolationError> {
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
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let mut sides_graph = HashMap::new();

            for segment in self.segments_of_direction(direction) {
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
                let neighbors_of_direction = self.filter_direction(&self.loop_structure.nexts(segment), direction);
                sides_graph.insert(segment, [other_side, neighbors_of_direction].concat());
            }

            for segment in self.segments_of_direction(direction) {
                for neighbor in sides_graph.get(&segment).unwrap().clone() {
                    let entry = sides_graph.entry(neighbor).or_insert(vec![]);
                    entry.push(segment);
                }
            }

            self.side_ccs[direction as usize] = hutspot::graph::find_ccs(&sides_graph.keys().copied().collect_vec(), |e: EdgeID| sides_graph[&e].clone());

            for component in &self.side_ccs[direction as usize] {
                // Find a labeling that is consistent and acyclic
                // We do this by finding a twocoloring of the bipartite graph
                let two_coloring = hutspot::graph::two_color::<EdgeID>(&component.iter().copied().collect_vec(), |e: EdgeID| sides_graph[&e].clone());
                if two_coloring.is_none() {
                    return Err(PropertyViolationError::NonTwoColorable);
                }
                let (upper_segments, lower_segments) = two_coloring.unwrap();

                let mut cur_score = 0.0;
                let mut flip_score = 0.0;
                for (&ls_id, s) in upper_segments
                    .iter()
                    .map(|ls_id| (ls_id, Vector3D::from(direction)))
                    .chain(lower_segments.iter().map(|ls_id| (ls_id, -Vector3D::from(direction))))
                {
                    let ls = &self.loop_structure.edges[ls_id];
                    let edges = vec![ls.between.clone()].into_iter().flatten();

                    for (edge1, edge2) in edges.tuple_windows() {
                        if self.mesh_ref.twin(edge1) == edge2 {
                            continue;
                        }
                        let u = self.mesh_ref.midpoint(edge1);
                        let v = self.mesh_ref.midpoint(edge2);
                        let face = self.mesh_ref.face(edge1);
                        assert!(face == self.mesh_ref.face(edge2));
                        let edge_normal = self.mesh_ref.normal(face);
                        let edge_direction = (v - u).normalize();
                        let edge_length = (v - u).norm();
                        let cross = edge_normal.cross(&edge_direction).normalize();
                        cur_score += cross.dot(&s) * edge_length;
                        flip_score += cross.dot(&-s) * edge_length;
                    }
                }

                if cur_score > flip_score {
                    for edge_id in upper_segments {
                        self.loop_structure.edges[edge_id].side = Some(Side::Upper);
                    }

                    for edge_id in lower_segments {
                        self.loop_structure.edges[edge_id].side = Some(Side::Lower);
                    }
                } else {
                    for edge_id in upper_segments {
                        self.loop_structure.edges[edge_id].side = Some(Side::Lower);
                    }

                    for edge_id in lower_segments {
                        self.loop_structure.edges[edge_id].side = Some(Side::Upper);
                    }
                }
            }
        }

        Ok(())
    }

    pub fn assign_loop_structure(
        &mut self,
        faces: &[Vec<usize>],
        intersections: &HashMap<EdgeID, Intersection>,
        segments: &[LoopSegment],
    ) -> Result<(), PropertyViolationError> {
        // Create douconel based on these faces
        let loop_structure_maybe = LoopStructure::from_faces(faces);
        if let Err(err) = loop_structure_maybe {
            return Err(PropertyViolationError::LoopStructureError(err));
        }
        let vmap;
        (self.loop_structure, vmap, _) = loop_structure_maybe.unwrap();

        let intersection_ids = intersections.keys().copied().collect_vec();
        for (vertex_id, vertex_obj) in &mut self.loop_structure.verts {
            let intersection_id = intersection_ids[vmap.get_by_right(&vertex_id).unwrap().to_owned()];
            intersections.get(&intersection_id).unwrap().clone_into(vertex_obj);
        }

        let mut endpoints_to_segment = HashMap::new();
        for segment in segments {
            endpoints_to_segment.insert((segment.start, segment.end), segment);
        }

        let loop_structure_c = self.loop_structure.clone();
        for (edge_id, edge_obj) in &mut self.loop_structure.edges {
            let start = self.loop_structure.verts[loop_structure_c.root(edge_id)].this;
            let end = self.loop_structure.verts[loop_structure_c.toor(edge_id)].this;
            *edge_obj = endpoints_to_segment.get(&(start, end)).copied().unwrap().clone();
        }

        Ok(())
    }

    pub fn assign_subsurfaces(&mut self) -> Result<(), PropertyViolationError> {
        // Get all blocked edges (ALL LOOPS)
        let blocked: HashSet<_> = self
            .loop_structure
            .edge_ids()
            .into_iter()
            .map(|edge_id| self.loop_structure.edges[edge_id].clone())
            .flat_map(|ls| [ls.between, vec![ls.start, ls.end]].concat())
            .flat_map(|edge_id| [edge_id, self.mesh_ref.twin(edge_id)])
            .map(|edge_id| self.mesh_ref.endpoints(edge_id))
            .collect();

        let vertex_neighbors: HashMap<VertID, Vec<VertID>> = self
            .mesh_ref
            .verts
            .keys()
            .map(|vertex: VertID| {
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
        let nfunction = |vertex: VertID| vertex_neighbors[&vertex].clone();

        // Find all connected components (should be equal to the number of faces)
        let ccs = hutspot::graph::find_ccs(&self.mesh_ref.verts.keys().collect_vec(), nfunction);
        if ccs.len() != self.loop_structure.face_ids().len() {
            return Err(PropertyViolationError::NonConnectedComponents);
        }

        // Map from some verts to adjacent loops(segments)
        let mut vert_to_loop_segments: HashMap<VertID, HashSet<_>> = HashMap::new();
        for (ls_id, ls) in &self.loop_structure.edges {
            for &edge_id in &ls.between {
                let endpoints = self.mesh_ref.endpoints(edge_id);
                vert_to_loop_segments.entry(endpoints.0).or_default().insert(ls_id);
                vert_to_loop_segments.entry(endpoints.1).or_default().insert(ls_id);
            }
        }

        // For each connected component, assign a subsurface
        // We can find the correct subsurface, by checking which loop segments are part of the connected component
        // So first map each connected component, to the adjacent loop segments
        let face_and_cc = ccs
            .into_iter()
            .map(|cc| {
                (
                    cc.clone(),
                    cc.into_iter()
                        .flat_map(|v| vert_to_loop_segments.get(&v).cloned().unwrap_or_default())
                        .collect::<HashSet<_>>(),
                )
            })
            .filter_map(|(cc, loop_segments)| {
                self.loop_structure
                    .face_ids()
                    .into_iter()
                    .find(|&face_id| {
                        // check that all loop segments of this face are part of the connected component
                        self.loop_structure.edges(face_id).iter().all(|&e| loop_segments.contains(&e))
                    })
                    .map(|face_id| (face_id, cc))
            })
            .map(|(face_id, subsurface)| {
                (
                    face_id,
                    Subsurface {
                        verts: subsurface.into_iter().collect(),
                        color: hutspot::color::hsl_to_rgb(rand::random::<f32>() * 360., 0.5, 0.5).into(),
                    },
                )
            })
            .collect_vec();

        if face_and_cc.len() != self.loop_structure.face_ids().len() {
            return Err(PropertyViolationError::NonConnectedComponents);
        }

        for (face_id, subsurface) in face_and_cc {
            self.loop_structure.faces[face_id] = subsurface;
        }

        Ok(())
    }

    pub fn build_loop_structure(&mut self) -> Result<Polycube, PropertyViolationError> {
        // Create the following "map"
        // List of all intersections
        // At each intersection, list all path segments that intersect there
        // We define both: the "next" intersection, and the local "next" edge
        // Using this information, we should be able to create an embedded graph (HEDS)

        if self.count_loops_in_direction(PrincipalDirection::X) == 0
            || self.count_loops_in_direction(PrincipalDirection::Y) == 0
            || self.count_loops_in_direction(PrincipalDirection::Z) == 0
        {
            return Err(PropertyViolationError::MissingDirection);
        }

        self.loop_structure = LoopStructure::default();

        let mut timer = hutspot::timer::Timer::new();

        println!("Computing intersections...");
        let intersections = self.intersections()?;
        timer.report("Intersections computation");
        timer.reset();

        println!("Constructing faces...");
        let faces = self.loop_faces(&intersections)?;
        timer.report("Faces computation");
        timer.reset();

        println!("Constructing segments...");
        let loop_segments = self.loop_segments(&intersections);
        timer.report("Segments computation");
        timer.reset();

        println!("Constructing loop structure...");
        self.assign_loop_structure(&faces, &intersections, &loop_segments)?;
        timer.report("Loop structure computation");
        timer.reset();

        println!("Assigning subsurfaces...");
        self.assign_subsurfaces()?;
        timer.report("Subsurfaces computation");
        timer.reset();

        println!("Basic properties and assigning sides...");
        self.verify_properties_and_assign_sides()?;
        timer.report("Properties and sides computation");
        timer.reset();

        println!("Computing zones...");
        self.assign_zones();
        timer.report("Zones computation");
        timer.reset();

        println!("Computing zone graph...");
        self.zone_graph([0, 0, 0])?;
        timer.report("Zone graph computation");
        timer.reset();

        println!("Computing Polycube...");
        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        let mut polycube = self.primal();
        timer.report("Polycube computation");
        timer.reset();

        println!("Computing paths");
        self.granulated_mesh = Some(self.place_paths(&mut polycube));
        timer.report("Place paths computation");
        timer.reset();

        Ok(polycube)
    }
}
