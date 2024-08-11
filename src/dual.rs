use crate::{
    solutions::{Loop, LoopID},
    EdgeID, EmbeddedMesh, VertID,
};
use bevy::render::color::Color;
use douconel::douconel::{Douconel, MeshError};
use hutspot::geom::Vector3D;
use itertools::Itertools;
use serde::{Deserialize, Serialize};
use slotmap::SlotMap;
use std::{
    collections::{HashMap, HashSet},
    fmt::Display,
    sync::Arc,
};

// Principal directions, used to characterize a polycube (each edge and face is associated with a principal direction)
#[derive(Copy, Clone, Default, PartialEq, Eq, Debug, Hash, Serialize, Deserialize)]
pub enum PrincipalDirection {
    #[default]
    X,
    Y,
    Z,
}

impl Display for PrincipalDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::X => write!(f, "X-axis"),
            Self::Y => write!(f, "Y-axis"),
            Self::Z => write!(f, "Z-axis"),
        }
    }
}

impl PrincipalDirection {
    pub fn to_primal_color(self) -> Color {
        match self {
            Self::X => hutspot::color::ROODT.into(),
            Self::Y => hutspot::color::BLAUW.into(),
            Self::Z => hutspot::color::YELLO.into(),
        }
    }

    pub fn to_primal_color_sided(self, s: Side) -> Color {
        match (self, s) {
            (Self::X, Side::Upper) => hutspot::color::ROODT.into(),
            (Self::X, Side::Lower) => hutspot::color::ROODT_L.into(),
            (Self::Y, Side::Upper) => hutspot::color::BLAUW.into(),
            (Self::Y, Side::Lower) => hutspot::color::BLAUW_L.into(),
            (Self::Z, Side::Upper) => hutspot::color::YELLO.into(),
            (Self::Z, Side::Lower) => hutspot::color::YELLO_L.into(),
        }
    }

    pub fn to_dual_color(self) -> Color {
        match self {
            Self::X => hutspot::color::GREEN.into(),
            Self::Y => hutspot::color::ORANG.into(),
            Self::Z => hutspot::color::PURPL.into(),
        }
    }

    pub fn to_dual_color_sided(self, s: Side) -> Color {
        match (self, s) {
            (Self::X, Side::Upper) => hutspot::color::GREEN.into(),
            (Self::X, Side::Lower) => hutspot::color::GREEN_L.into(),
            (Self::Y, Side::Upper) => hutspot::color::ORANG.into(),
            (Self::Y, Side::Lower) => hutspot::color::ORANG_L.into(),
            (Self::Z, Side::Upper) => hutspot::color::PURPL.into(),
            (Self::Z, Side::Lower) => hutspot::color::PURPL_L.into(),
        }
    }
}

impl From<PrincipalDirection> for Vector3D {
    fn from(dir: PrincipalDirection) -> Self {
        match dir {
            PrincipalDirection::X => Self::new(1., 0., 0.),
            PrincipalDirection::Y => Self::new(0., 1., 0.),
            PrincipalDirection::Z => Self::new(0., 0., 1.),
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum Side {
    Upper,
    Lower,
}

impl Side {
    pub const fn flip(self) -> Self {
        match self {
            Self::Upper => Self::Lower,
            Self::Lower => Self::Upper,
        }
    }
}

pub fn to_principal_direction(v: Vector3D) -> (PrincipalDirection, Side) {
    let x_is_max = v.x.abs() > v.y.abs() && v.x.abs() > v.z.abs();
    let y_is_max = v.y.abs() > v.x.abs() && v.y.abs() > v.z.abs();
    let z_is_max = v.z.abs() > v.x.abs() && v.z.abs() > v.y.abs();
    assert!(x_is_max ^ y_is_max ^ z_is_max);

    if x_is_max {
        if v.x > 0. {
            (PrincipalDirection::X, Side::Upper)
        } else {
            (PrincipalDirection::X, Side::Lower)
        }
    } else if y_is_max {
        if v.y > 0. {
            (PrincipalDirection::Y, Side::Upper)
        } else {
            (PrincipalDirection::Y, Side::Lower)
        }
    } else if z_is_max {
        if v.z > 0. {
            (PrincipalDirection::Z, Side::Upper)
        } else {
            (PrincipalDirection::Z, Side::Lower)
        }
    } else {
        unreachable!()
    }
}

// A collection of loops forms a loop structure; a graph. Vertices are loop intersections. Edges are loop segments. And faces are loop regions.
slotmap::new_key_type! {
    pub struct IntersectionID;
    pub struct SegmentID;
    pub struct RegionID;
}

type LoopStructure = Douconel<IntersectionID, Intersection, SegmentID, Segment, RegionID, Region>;

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub struct Segment {
    // A loop segment is defined by a reference to a loop (id)
    pub loop_id: LoopID,
    // direction
    pub direction: PrincipalDirection,
    // The start and end of the segment (intersections)
    pub start: EdgeID,
    pub end: EdgeID,
    // And a sequence of half-edges that define the segment of the loop (inbetween start and end)
    pub between: Vec<EdgeID>,
    // Side
    pub side: Option<Side>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Region {
    // A region is defined by a set of vertices
    pub verts: HashSet<VertID>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Zone {
    // A zone is defined by a direction
    pub direction: PrincipalDirection,
    // All regions that are part of the zone
    pub regions: HashSet<RegionID>,
    // Coordinate, TODO: probably dont want this here
    pub coordinate: Option<f64>,
}

slotmap::new_key_type! {
    pub struct ZoneID;
}

// Dual structure is an "ABSTRACT PATH REPRESENTATION"
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct Dual {
    pub mesh_ref: Arc<EmbeddedMesh>,
    pub loop_structure: LoopStructure,
    pub zones: SlotMap<ZoneID, Zone>,
    pub side_ccs: [Vec<HashSet<SegmentID>>; 3],
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub enum PropertyViolationError<IntersectionID> {
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
    PatchesMissing,
    LoopStructureError(MeshError<IntersectionID>),
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
            loop_structure: Douconel::default(),
            zones: SlotMap::with_key(),
            side_ccs: [vec![], vec![], vec![]],
        }
    }

    pub fn from(mesh_ref: Arc<EmbeddedMesh>, loops: &SlotMap<LoopID, Loop>) -> Result<Self, PropertyViolationError<IntersectionID>> {
        let mut timer = hutspot::timer::Timer::new();

        let mut dual = Self::new(mesh_ref);

        let intersections = dual.loop_intersections(loops)?;
        timer.report("Computed all loop intersections");
        timer.reset();

        let faces = dual.loop_faces(&intersections)?;
        timer.report("Computed all loop faces");
        timer.reset();

        let loop_segments = dual.loop_segments(loops, &intersections);
        timer.report("Computed all loop segments");
        timer.reset();

        dual.assign_loop_structure(&faces, &intersections, &loop_segments)?;
        timer.report("Computed loop structure");
        timer.reset();

        dual.assign_subsurfaces()?;
        timer.report("Computed subsurfaces");
        timer.reset();

        dual.verify_properties_and_assign_sides(loops)?;
        timer.report("Verified properties and assigned sides");
        timer.reset();

        dual.assign_zones();
        timer.report("Computed zones");
        timer.reset();

        dual.zone_graph([0, 0, 0])?;
        timer.report("Computed and verified zone graph");
        timer.reset();

        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>
        // UNKNOWN HIGHER GENUS REQUIREMENTS HERE>>>

        Ok(dual)
    }

    pub fn segment_to_loop(&self, segment: SegmentID) -> LoopID {
        self.loop_structure.edges[segment].loop_id
    }

    pub fn segment_to_edges(&self, segment: SegmentID) -> Vec<EdgeID> {
        [
            vec![self.loop_structure.edges[segment].start],
            self.loop_structure.edges[segment].between.clone(),
            vec![self.loop_structure.edges[segment].end],
        ]
        .concat()
    }

    // pub fn segment_to_direction(&self, segment: SegmentID) -> PrincipalDirection {
    //     let loop_id = self.segment_to_loop(segment);
    //     self.loops[loop_id].direction
    // }

    pub fn segment_to_side(&self, segment: SegmentID, mask: [u32; 3]) -> Side {
        let cc = self.side_ccs[self.loop_structure.edges[segment].direction as usize]
            .iter()
            .position(|cc| cc.contains(&segment))
            .unwrap() as u8;

        let side = self.loop_structure.edges[segment].side.clone().unwrap();

        if mask[self.loop_structure.edges[segment].direction as usize] & (1 << cc) == 1 {
            side.flip()
        } else {
            side
        }
    }

    pub fn filter_direction(&self, selection: &[SegmentID], direction: PrincipalDirection) -> Vec<SegmentID> {
        selection
            .iter()
            .filter(|&&segment| self.loop_structure.edges[segment].direction == direction)
            .copied()
            .collect()
    }

    pub fn segments_of_direction(&self, direction: PrincipalDirection) -> Vec<SegmentID> {
        self.filter_direction(&self.loop_structure.edge_ids(), direction)
    }

    fn assign_zones(&mut self) {
        // A zone is a collection of loop segments that are connected, and have the same direction (and side)

        // Create zones
        self.zones = SlotMap::with_key();

        // Grab all loop segments
        let mut loop_segments_queue = self.loop_structure.edge_ids();

        // While there are loop segments left, grab a loop segment, and find its corresponding zone
        while let Some(segment) = loop_segments_queue.pop() {
            // Find the direction of the segment
            let this_direction = self.loop_structure.edges[segment].direction;

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
                    .filter(|&segment| self.loop_structure.edges[segment].direction == this_direction && !edges_of_this_zone.contains(&segment))
                    .collect_vec();
                edges_of_this_zone.extend(edges_of_this_subsurface.clone());

                // Get neighboring subsurfaces of this subsurface, that are connected only by a loop segment != this_direction
                let new_neighbors = self
                    .loop_structure
                    .edges(subsurface)
                    .into_iter()
                    .filter(|&segment| self.loop_structure.edges[segment].direction != this_direction)
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
                regions: neighboring_subsurfaces,
                coordinate: None,
            });
        }
    }

    // Get zones and topological sort on the zones.
    fn zone_graph(&mut self, side_mask: [u32; 3]) -> Result<(), PropertyViolationError<IntersectionID>> {
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
                    .find(|(_, z)| z.regions.contains(&self.loop_structure.face(segment)) && z.direction == self.loop_structure.edges[segment].direction)
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
                            && self.loop_structure.edges[segment].direction == this_direction
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

    // Returns error if:
    //    1. Self-intersections or transversal intersections
    //    2. More than two loops intersect at each intersection (resulting in vertices of degree 4)
    fn loop_intersections(&self, loops: &SlotMap<LoopID, Loop>) -> Result<HashMap<EdgeID, Intersection>, PropertyViolationError<IntersectionID>> {
        // For each loop:
        //   We find all its intersections, by following the edges that the loop passes
        //   Therefore, the intersections are in the order of the loop traversal
        //   An intersection is defined per two half-edges, we only store the intersection at the first (lower) half-edge

        let occupied = Loop::occupied(loops);

        let loop_to_intersections: HashMap<LoopID, Vec<IntersectionMarker>> = loops
            .iter()
            .map(|(loop_id, l)| {
                let path_intersections = l
                    .edges
                    .clone()
                    .into_iter()
                    .map(|edge| (edge, occupied.get(edge).cloned().unwrap()))
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

                        if loops[intersection_marker.loops[0]].contains_pair((edge, next_edge)) {
                            Some((next_edge, intersection_marker.loops[0], 1))
                        } else if loops[intersection_marker.loops[0]].contains_pair((next_edge, edge)) {
                            Some((next_edge, intersection_marker.loops[0], -1))
                        } else if loops[intersection_marker.loops[1]].contains_pair((edge, next_edge)) {
                            Some((next_edge, intersection_marker.loops[1], 1))
                        } else if loops[intersection_marker.loops[1]].contains_pair((next_edge, edge)) {
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

    fn loop_segments(&self, loops: &SlotMap<LoopID, Loop>, intersections: &HashMap<EdgeID, Intersection>) -> Vec<Segment> {
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
                    loops[loop_id].between(this_intersection_pointer_to_next.0, next_intersection_pointer_to_this.0)
                } else {
                    loops[loop_id]
                        .between(next_intersection_pointer_to_this.0, this_intersection_pointer_to_next.0)
                        .into_iter()
                        .rev()
                        .collect_vec()
                };

                Segment {
                    loop_id,
                    direction: loops[loop_id].direction,
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
    fn loop_faces(&self, intersections: &HashMap<EdgeID, Intersection>) -> Result<Vec<Vec<usize>>, PropertyViolationError<IntersectionID>> {
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

    fn verify_properties_and_assign_sides(&mut self, loops: &SlotMap<LoopID, Loop>) -> Result<(), PropertyViolationError<IntersectionID>> {
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
                if self.loop_structure.edges[this].direction == self.loop_structure.edges[next].direction {
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
                let next_of_same_color = |id: SegmentID| {
                    let cand = self
                        .loop_structure
                        .outgoing(self.loop_structure.toor(id))
                        .iter()
                        .flat_map(|&x| {
                            vec![
                                (x, loops[self.loop_structure.edges[x].loop_id].direction),
                                (
                                    self.loop_structure.twin(x),
                                    loops[self.loop_structure.edges[self.loop_structure.twin(x)].loop_id].direction,
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

            self.side_ccs[direction as usize] = hutspot::graph::find_ccs(&sides_graph.keys().copied().collect_vec(), |e| sides_graph[&e].clone());

            for component in &self.side_ccs[direction as usize] {
                // Find a labeling that is consistent and acyclic
                // We do this by finding a twocoloring of the bipartite graph
                let two_coloring = hutspot::graph::two_color::<_>(&component.iter().copied().collect_vec(), |e| sides_graph[&e].clone());
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

    fn assign_loop_structure(
        &mut self,
        faces: &[Vec<usize>],
        intersections: &HashMap<EdgeID, Intersection>,
        segments: &[Segment],
    ) -> Result<(), PropertyViolationError<IntersectionID>> {
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

    fn assign_subsurfaces(&mut self) -> Result<(), PropertyViolationError<IntersectionID>> {
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
            .map(|(face_id, subsurface)| (face_id, Region { verts: subsurface }))
            .collect_vec();

        if face_and_cc.len() != self.loop_structure.face_ids().len() {
            return Err(PropertyViolationError::NonConnectedComponents);
        }

        for (face_id, subsurface) in face_and_cc {
            self.loop_structure.faces[face_id] = subsurface;
        }

        Ok(())
    }
}
