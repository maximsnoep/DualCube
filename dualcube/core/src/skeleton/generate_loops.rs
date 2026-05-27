use std::{
    collections::{BinaryHeap, HashMap, HashSet},
    f64::consts::PI,
};

use bimap::BiHashMap;
use ordered_float::OrderedFloat;
use log::{error, warn};
use mehsh::prelude::{HasEdges, HasFaces, HasPosition, HasVertices, Mesh, Vector3D};
use petgraph::{
    graph::{EdgeIndex, NodeIndex},
    visit::{EdgeRef, IntoEdgeReferences, IntoNodeReferences},
};
use slotmap::SlotMap;

use crate::{
    prelude::{EdgeID, FaceID, PrincipalDirection, VertID, INPUT},
    skeleton::{
        boundary_loop::BoundaryLoop,
        orthogonalize::{AxisSign, LabeledCurveSkeleton, LabeledSkeletonSignExt},
        SkeletonData,
    },
    solutions::{Loop, LoopID},
};

const ALL_DIRS: [PrincipalDirection; 3] = [
    PrincipalDirection::X,
    PrincipalDirection::Y,
    PrincipalDirection::Z,
];
const ALL_SIGNS: [AxisSign; 2] = [AxisSign::Positive, AxisSign::Negative];

/// Per boundary loop, the crossing points for each orthogonal (direction, sign).
pub type CrossingMap = HashMap<LoopID, HashMap<(PrincipalDirection, AxisSign), EdgeID>>;

/// Per node, a face point for each (direction, sign) slot that has no neighboring patch.
/// Each face point is an interior mesh edge midpoint on the patch surface.
pub type FacePointMap = HashMap<NodeIndex, HashMap<(PrincipalDirection, AxisSign), EdgeID>>;

pub enum LoopGenerationError {
    MissingLabeledSkeleton,
    // TODO other error variants
}

/// Diagnostics about loops that could not be routed, for GUI visualization. Lets the user
/// see WHICH loops were dropped, how far they got, and WHERE a segment failed to connect.
/// Recomputed each `generate_loops` call; not persisted.
#[derive(Debug, Clone, Default)]
pub struct RoutingDiagnostics {
    /// Loops dropped because at least one segment failed to route. Each entry is the loop's
    /// axis and the raw single-half-edge path it had built before being abandoned (segments
    /// that did route are present; failed segments appear as gaps / straight chords).
    pub dropped_loops: Vec<(PrincipalDirection, Vec<EdgeID>)>,
    /// Control-point pairs `(src, tgt)` that a segment's Dijkstra could not connect — the
    /// gaps where routing got stuck.
    pub failed_segments: Vec<(EdgeID, EdgeID)>,
}

/// Generates surface-embedded loops from a polycube and polycube map.
pub fn generate_loops(
    skeleton_data: &SkeletonData,
    mesh: &Mesh<INPUT>,
) -> Result<(SlotMap<LoopID, Loop>, CrossingMap, FacePointMap, RoutingDiagnostics), LoopGenerationError> {
    let mut map: SlotMap<LoopID, Loop> = SlotMap::with_key();
    let mut diagnostics = RoutingDiagnostics::default();

    let skeleton: &LabeledCurveSkeleton = skeleton_data
        .labeled_skeleton
        .as_ref()
        .ok_or_else(|| LoopGenerationError::MissingLabeledSkeleton)?;
    let (boundary_map, crossings) = get_boundaries_and_crossing_points(skeleton, mesh, &mut map);
    let face_points = compute_face_points(skeleton, mesh);

    // Trace paths between boundary points and face points to create the loops
    pathing_for_loops(
        boundary_map,
        crossings.clone(), // TODO: later we can simply consume as we no longer need to return it
        face_points.clone(), // TODO: same here
        skeleton,
        mesh,
        &mut map,
        &mut diagnostics,
    );

    // Pathing (and the boundary face-walk) produce loops with one half-edge per crossed
    // geometric edge. The downstream `Dual` builder requires both halves of every crossed
    // edge: it detects crossings by canonicalizing to the higher half-edge (`edge > twin(edge)`)
    // and its quad-walk assumes each edge's twin sits adjacent to it in the sequence
    // (see `solutions::check_loop` for the canonical alternating twin/same-face form).
    // We expand AFTER pathing so the routing's blocking still operates on the raw single-half form.
    for (_, l) in map.iter_mut() {
        let raw = std::mem::take(&mut l.edges);
        l.edges = expand_to_double_halfedges(raw, mesh);
    }

    Ok((map, crossings, face_points, diagnostics))
}

/// Calculates for each patch-patch boundary the appropriate loop and crossing points for the other two loop types.
///
/// For each boundary loop, places 4 crossings by:
/// 1. Projecting the boundary points onto the loop's plane (perpendicular to the skeleton edge).
/// 2. Computing angles from the centroid in that plane.
/// 3. For each orthogonal (direction, sign), projecting the axis direction onto the plane to
///    get a target angle, then picking the boundary point closest to that target angle.
///
/// This ensures crossings are naturally spread around the loop (~90° apart for axis-aligned geometry)
/// without needing direction propagation between nodes.
fn get_boundaries_and_crossing_points(
    skeleton: &LabeledCurveSkeleton,
    mesh: &Mesh<INPUT>,
    map: &mut SlotMap<LoopID, Loop>,
) -> (BiHashMap<EdgeIndex, LoopID>, CrossingMap) {
    let mut crossings: CrossingMap = HashMap::new();
    let mut boundary_map = BiHashMap::new();

    for edge in skeleton.edge_references() {
        let weight = edge.weight();
        let direction = weight.direction;
        let boundary = weight.boundary_loop.clone();

        // Create loop and save its ID
        let loop_id = map.insert(get_loop(boundary.clone(), direction));
        boundary_map.insert(edge.id(), loop_id);

        // Compute centroid of boundary loop
        let n = boundary.edge_midpoints.len() as f64;
        let centroid: Vector3D = boundary
            .edge_midpoints
            .iter()
            .fold(Vector3D::zeros(), |acc, &e| acc + mesh.position(e))
            / n;

        // Loop plane normal from skeleton edge geometry
        let source_pos = skeleton[edge.source()].skeleton_node.position;
        let target_pos = skeleton[edge.target()].skeleton_node.position;
        let normal = (target_pos - source_pos).normalize();

        // Build orthonormal basis (u, v) on the plane perpendicular to the normal
        let arbitrary = if normal.x.abs() < 0.9 {
            Vector3D::new(1.0, 0.0, 0.0)
        } else {
            Vector3D::new(0.0, 1.0, 0.0)
        };
        let u = normal.cross(&arbitrary).normalize();
        let v = normal.cross(&u); // already unit length

        // Compute angle of each boundary point relative to centroid in the loop plane
        let point_angles: Vec<(usize, f64)> = boundary
            .edge_midpoints
            .iter()
            .enumerate()
            .map(|(i, &e)| {
                let offset = mesh.position(e) - centroid;
                let proj_u = offset.dot(&u);
                let proj_v = offset.dot(&v);
                (i, proj_v.atan2(proj_u))
            })
            .collect();

        // For each orthogonal (direction, sign), project the axis direction onto the loop plane
        // to get a target angle, then pick the boundary point closest in angle.
        let ortho: Vec<_> = ALL_DIRS
            .iter()
            .copied()
            .filter(|&d| d != direction)
            .collect();
        let mut loop_crossings = HashMap::new();

        for &dir in &ortho {
            // A `dir`-type loop crosses this boundary at positions spread along the
            // THIRD direction T = third(dir, boundary_dir).  Keying by (T, sign)
            // means crossing (T,+) is at the +T side and (T,-) at the -T side,
            // matching the face-point sign convention on adjacent nodes.
            let t_dir = third(dir, direction);
            for sign in ALL_SIGNS {
                let axis_vec = match sign {
                    AxisSign::Positive => Vector3D::from(t_dir),
                    AxisSign::Negative => -Vector3D::from(t_dir),
                };
                let target_angle = axis_vec.dot(&v).atan2(axis_vec.dot(&u));

                let &(best_idx, _) = point_angles
                    .iter()
                    .min_by(|(_, a), (_, b)| {
                        let diff_a = angle_distance(*a, target_angle);
                        let diff_b = angle_distance(*b, target_angle);
                        diff_a
                            .partial_cmp(&diff_b)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .expect("boundary loop should not be empty");

                loop_crossings.insert((t_dir, sign), boundary.edge_midpoints[best_idx]);
            }
        }

        crossings.insert(loop_id, loop_crossings);
    }

    (boundary_map, crossings)
}

/// For each skeleton node, finds an interior mesh edge midpoint for every (direction, sign)
/// slot that does not already have a neighboring patch (skeleton edge).
///
/// Boundary centroids from existing edges are used internally as direction constraints:
/// if the opposite sign of the same direction has a boundary, the face point is placed
/// directly opposite it. Otherwise falls back to the global axis direction.
pub fn compute_face_points(skeleton: &LabeledCurveSkeleton, mesh: &Mesh<INPUT>) -> FacePointMap {
    let mut result: FacePointMap = HashMap::new();

    for (node_idx, node_weight) in skeleton.node_references() {
        let node_pos = node_weight.skeleton_node.position;
        let patch_set: HashSet<VertID> = node_weight
            .skeleton_node
            .patch_vertices
            .iter()
            .copied()
            .collect();

        // Record boundary centroids and their directions.
        // These are NOT stored in the output, only used to guide placement of missing slots.
        let mut occupied: HashSet<(PrincipalDirection, AxisSign)> = HashSet::new();
        let mut boundary_centroids: HashMap<(PrincipalDirection, AxisSign), Vector3D> =
            HashMap::new();
        let mut known_dirs: Vec<Vector3D> = Vec::new();

        let degree = skeleton.edges(node_idx).count();

        for edge_ref in skeleton.edges(node_idx) {
            let ew = edge_ref.weight();
            let dir = ew.direction;
            // edge_ref.source() on StableUnGraph returns the stored first endpoint,
            // NOT necessarily the querying node. Use edge_endpoints to check correctly.
            let (stored_a, _) = skeleton
                .edge_endpoints(edge_ref.id())
                .expect("edge must exist");
            let sign = if stored_a == node_idx {
                ew.sign
            } else {
                ew.sign.flipped()
            };

            if occupied.contains(&(dir, sign)) {
                warn!(
                    "Node {:?} (degree {}): duplicate slot ({:?}, {:?}) — edge {:?} \
                     (stored sign {:?}, stored_source={:?}, this node={:?})",
                    node_idx,
                    degree,
                    dir,
                    sign,
                    edge_ref.id(),
                    ew.sign,
                    stored_a,
                    node_idx
                );
            }

            let boundary = &ew.boundary_loop;
            let n = boundary.edge_midpoints.len() as f64;
            let centroid: Vector3D = boundary
                .edge_midpoints
                .iter()
                .fold(Vector3D::zeros(), |acc, &e| acc + mesh.position(e))
                / n;

            let dir_vec = (centroid - node_pos).normalize();
            known_dirs.push(dir_vec);
            occupied.insert((dir, sign));
            boundary_centroids.insert((dir, sign), centroid);
        }

        // Collect candidate interior edge midpoints
        // Exclude edges that lie on any boundary loop (those are near patch borders).
        let mut boundary_edges: HashSet<(VertID, VertID)> = HashSet::new();
        for edge_ref in skeleton.edges(node_idx) {
            for &e in &edge_ref.weight().boundary_loop.edge_midpoints {
                let a = mesh.root(e);
                let b = mesh.toor(e);
                let key = if a < b { (a, b) } else { (b, a) };
                boundary_edges.insert(key);
            }
        }

        let mut seen: HashSet<(VertID, VertID)> = HashSet::new();
        let mut candidates: Vec<EdgeID> = Vec::new();

        for &v in &node_weight.skeleton_node.patch_vertices {
            for face in mesh.faces(v) {
                let verts: Vec<VertID> = mesh.vertices(face).collect();
                for i in 0..verts.len() {
                    let a = verts[i];
                    let b = verts[(i + 1) % verts.len()];
                    if !patch_set.contains(&a) || !patch_set.contains(&b) {
                        continue;
                    }
                    let key = if a < b { (a, b) } else { (b, a) };
                    if boundary_edges.contains(&key) {
                        continue;
                    }
                    if seen.insert(key) {
                        if let Some((e, _)) = mesh.edge_between_verts(a, b) {
                            candidates.push(e);
                        }
                    }
                }
            }
        }

        if candidates.is_empty() {
            warn!(
                "Node {:?} has no interior edge candidates for missing face points.",
                node_idx
            );
        }

        // Fill missing slots (only directions without a skeleton edge)
        let mut interior_points: HashMap<(PrincipalDirection, AxisSign), EdgeID> = HashMap::new();

        for dir in ALL_DIRS {
            for sign in ALL_SIGNS {
                if occupied.contains(&(dir, sign)) {
                    continue;
                }
                if candidates.is_empty() {
                    continue;
                }

                let opposite_sign = sign.flipped();
                let target_dir =
                    if let Some(&centroid) = boundary_centroids.get(&(dir, opposite_sign)) {
                        // Opposite side has a boundary: place directly opposite its centroid
                        -(centroid - node_pos).normalize()
                    } else if let Some(&edge) = interior_points.get(&(dir, opposite_sign)) {
                        // Opposite side was already placed as an interior point: go opposite
                        -(edge_midpoint_pos(edge, mesh) - node_pos).normalize()
                    } else {
                        // No opposite exists yet: use global axis direction
                        match sign {
                            AxisSign::Positive => Vector3D::from(dir),
                            AxisSign::Negative => -Vector3D::from(dir),
                        }
                    };

                let best = *candidates
                    .iter()
                    .max_by(|&&e1, &&e2| {
                        let v1 = (edge_midpoint_pos(e1, mesh) - node_pos).normalize();
                        let v2 = (edge_midpoint_pos(e2, mesh) - node_pos).normalize();
                        let dot1 = v1.dot(&target_dir);
                        let dot2 = v2.dot(&target_dir);
                        dot1.partial_cmp(&dot2).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .expect("candidates is non-empty");

                let best_pos = edge_midpoint_pos(best, mesh);
                known_dirs.push((best_pos - node_pos).normalize());
                interior_points.insert((dir, sign), best);
            }
        }

        assert_eq!(
            occupied.len(),
            degree,
            "Node {:?} (degree {}): only {} unique (dir, sign) slots — \
             upstream orthogonalization assigned duplicate slots. Occupied: {:?}",
            node_idx,
            degree,
            occupied.len(),
            occupied
        );
        debug_assert_eq!(
            degree + interior_points.len(),
            6,
            "Node {:?}: degree ({}) + face_points ({}) != 6",
            node_idx,
            degree,
            interior_points.len()
        );

        result.insert(node_idx, interior_points);
    }

    result
}

/// Position of an edge's midpoint.
fn edge_midpoint_pos(e: EdgeID, mesh: &Mesh<INPUT>) -> Vector3D {
    let a = mesh.position(mesh.root(e));
    let b = mesh.position(mesh.toor(e));
    (a + b) * 0.5
}

/// Shortest angular distance between two angles in radians.
fn angle_distance(a: f64, b: f64) -> f64 {
    let mut d = (a - b) % (2.0 * PI);
    if d > PI {
        d -= 2.0 * PI;
    } else if d < -PI {
        d += 2.0 * PI;
    }
    d.abs()
}

fn get_loop(boundary: BoundaryLoop, direction: PrincipalDirection) -> Loop {
    Loop {
        edges: boundary.edge_midpoints,
        direction,
    }
}

/// Expands a loop recorded as one half-edge per crossed geometric edge into the
/// canonical form the downstream `Dual` builder expects: a sequence in which
/// consecutive entries are always either twins of one another or share a face
/// (so every crossed edge appears as both of its half-edges).
///
/// Inserts the bridging half-edges produced by `bridge_edges` between each pair of
/// consecutive originals, skipping any insert already present to avoid duplicates.
fn expand_to_double_halfedges(edges: Vec<EdgeID>, mesh: &Mesh<INPUT>) -> Vec<EdgeID> {
    let n = edges.len();
    if n < 2 {
        return edges;
    }
    let mut present: HashSet<EdgeID> = edges.iter().copied().collect();
    let mut expanded = Vec::with_capacity(2 * n);
    for i in 0..n {
        let from = edges[i];
        let to = edges[(i + 1) % n];

        expanded.push(from);
        for inserted in bridge_edges(from, to, mesh) {
            if present.contains(&inserted) {
                continue;
            }
            present.insert(inserted);
            expanded.push(inserted);
        }
    }
    expanded
}

/// Returns the half-edges to insert between `from` and `to` so that consecutive entries
/// in the resulting sequence are always either twins of one another or share a face.
fn bridge_edges(from: EdgeID, to: EdgeID, mesh: &Mesh<INPUT>) -> Vec<EdgeID> {
    if mesh.twin(from) == to || mesh.face(from) == mesh.face(to) {
        return vec![];
    }
    let twin_from = mesh.twin(from);
    let twin_to = mesh.twin(to);
    // Single-edge bridge: twin(from) shares face with `to`.
    if mesh.face(twin_from) == mesh.face(to) {
        return vec![twin_from];
    }
    // Single-edge bridge: twin(to) shares face with `from`.
    if mesh.face(twin_to) == mesh.face(from) {
        return vec![twin_to];
    }
    // Two-edge bridge: [twin(from), twin(to)] — the inserted twins themselves share
    // a face. Concretely: from in F0, to in F2, twin(from) and twin(to) both in F1.
    // The expanded sub-sequence is then [from, twin(from), twin(to), to] with the
    // pairs being (twins, share-face, twins).
    if mesh.face(twin_from) == mesh.face(twin_to) {
        return vec![twin_from, twin_to];
    }
    panic!(
        "bridge_edges: cannot bridge half-edges {:?} (face {:?}, twin_face {:?}) and {:?} (face {:?}, twin_face {:?})",
        from, mesh.face(from), mesh.face(twin_from),
        to, mesh.face(to), mesh.face(twin_to)
    );
}

/// Cost multiplier applied when entering or leaving a face that has at least one blocked edge.
const SHARED_EDGE_MULTIPLIER: f64 = 8.0;

/// Dijkstra's on the mesh dual graph to find the shortest intermediate path between two
/// control-point edges, using geodesic (face-centroid -> edge-midpoint -> face-centroid) cost.
///
/// Returns the mesh edges crossed between `source` and `target`, **exclusive** of both.
/// The caller is responsible for pushing `source` before and `target` after this list.
///
/// `blocked` contains the reverse half-edges of existing loops (twin of each traversed edge).
/// A path cannot traverse a half-edge that is in `blocked`, which prevents crossing an existing
/// loop in the opposite direction while still allowing parallel (same-direction) traversal.
///
/// `used` contains half-edges already traversed by the current loop being traced. This prevents
/// self-intersection: a loop cannot reuse an edge it already traversed (even in the same direction).
/// Unlike `blocked`, `used` is scoped to a single loop and discarded afterward so that later loops
/// can still share those edges in parallel.
///
/// Faces adjacent to blocked/used edges incur a `SHARED_EDGE_MULTIPLIER` cost penalty,
/// encouraging paths to stay away from existing loops even when parallel traversal is allowed.
///
/// `forced_first`, when `Some(e)`, requires the path's FIRST step (out of a source face) to be
/// exactly `e`. `forced_last`, when `Some(e)`, requires the path's LAST step (into a target
/// face) to be exactly `e`. These enforce the straight-through (quad-diagonal) crossing rule at
/// the control points: a loop must leave a crossing via the diagonal partner of how it arrived,
/// and the closing segment must arrive matching the first segment's departure.
///
/// Returns `None` if no path exists.
fn surface_path_intermediates(
    source: EdgeID,
    target: EdgeID,
    forced_first: Option<EdgeID>,
    forced_last: Option<EdgeID>,
    blocked: &HashSet<EdgeID>,
    used: &HashSet<EdgeID>,
    mesh: &Mesh<INPUT>,
) -> Option<Vec<EdgeID>> {
    let faces_of = |e: EdgeID| -> Vec<FaceID> {
        let a = mesh.root(e);
        let b = mesh.toor(e);
        let set_a: HashSet<FaceID> = mesh.faces(a).collect();
        mesh.faces(b).filter(|f| set_a.contains(f)).collect()
    };

    let face_centroid = |f: FaceID| -> Vector3D {
        let verts: Vec<VertID> = mesh.vertices(f).collect();
        let n = verts.len() as f64;
        verts.iter().fold(Vector3D::zeros(), |acc, &v| acc + mesh.position(v)) / n
    };

    // Returns true if the face has at least one edge in `blocked` or `used` (other than `except`).
    // Checks both half-edges of each geometric edge: with directional blocking only one
    // half-edge per geometric edge is in `blocked`, depending on which direction was traversed.
    // `except` is from the source face's perspective; its twin is this face's view of the same
    // geometric edge, so both are excluded to avoid counting the entry edge as a blocked neighbor.
    let face_touches_blocked = |f: FaceID, except: EdgeID| -> bool {
        let except_twin = mesh.twin(except);
        let verts: Vec<VertID> = mesh.vertices(f).collect();
        let n = verts.len();
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            if let Some((e, _)) = mesh.edge_between_verts(a, b) {
                if e != except && e != except_twin {
                    let twin = mesh.twin(e);
                    if blocked.contains(&e) || blocked.contains(&twin)
                        || used.contains(&e) || used.contains(&twin)
                    {
                        return true;
                    }
                }
            }
        }
        false
    };

    let source_faces = faces_of(source);
    let target_face_set: HashSet<FaceID> = faces_of(target).into_iter().collect();

    // If source and target already share a face, no intermediate edges are needed.
    if source_faces.iter().any(|f| target_face_set.contains(f)) {
        return Some(vec![]);
    }

    // Dijkstra. dist: best known cost to reach a face.
    // prev: face -> (parent_face, edge_used_to_reach_it).
    // Heap entries: (Reverse(cost), face).
    let mut dist: HashMap<FaceID, f64> = HashMap::new();
    let mut prev: HashMap<FaceID, (FaceID, Option<EdgeID>)> = HashMap::new();
    let mut heap: BinaryHeap<(std::cmp::Reverse<OrderedFloat<f64>>, FaceID)> = BinaryHeap::new();

    for &sf in &source_faces {
        dist.insert(sf, 0.0);
        prev.insert(sf, (sf, None));
        heap.push((std::cmp::Reverse(OrderedFloat(0.0)), sf));
    }

    let mut found: Option<FaceID> = None;

    'dijkstra: while let Some((std::cmp::Reverse(OrderedFloat(cost)), face)) = heap.pop() {
        if dist.get(&face).copied().unwrap_or(f64::INFINITY) < cost {
            continue; // stale entry
        }

        let face_verts: Vec<VertID> = mesh.vertices(face).collect();
        let n = face_verts.len();
        let centroid_a = face_centroid(face);
        // A source face is one seeded with no incoming edge. Leaving it, `forced_first`
        // (if set) is the only edge permitted, so the loop departs the crossing via the
        // required diagonal side.
        let is_source = prev.get(&face).map(|p| p.1.is_none()).unwrap_or(false);

        for i in 0..n {
            let a = face_verts[i];
            let b = face_verts[(i + 1) % n];
            let Some((edge, _)) = mesh.edge_between_verts(a, b) else { continue };

            if is_source {
                if let Some(ff) = forced_first {
                    if edge != ff {
                        continue;
                    }
                }
            }

            if blocked.contains(&edge) || used.contains(&edge) {
                continue;
            }

            // The adjacent face across this edge.
            let set_a: HashSet<FaceID> = mesh.faces(a).collect();
            let Some(next_face) = mesh.faces(b).find(|&f| set_a.contains(&f) && f != face)
            else {
                continue;
            };

            // Entering a target face is only allowed via `forced_last` (if set), so the
            // closing segment arrives at the start crossing on the required diagonal side.
            let entering_target = target_face_set.contains(&next_face);
            if entering_target {
                if let Some(fl) = forced_last {
                    if edge != fl {
                        continue;
                    }
                }
            }

            let edge_mid = edge_midpoint_pos(edge, mesh);
            let centroid_b = face_centroid(next_face);
            let mut step_cost = (centroid_a - edge_mid).norm() + (edge_mid - centroid_b).norm();

            // Penalize entering a face adjacent to blocked edges.
            if face_touches_blocked(next_face, edge) {
                step_cost *= SHARED_EDGE_MULTIPLIER;
            }

            let new_cost = cost + step_cost;
            if new_cost < dist.get(&next_face).copied().unwrap_or(f64::INFINITY) {
                dist.insert(next_face, new_cost);
                prev.insert(next_face, (face, Some(edge)));

                if entering_target {
                    found = Some(next_face);
                    break 'dijkstra;
                }

                heap.push((std::cmp::Reverse(OrderedFloat(new_cost)), next_face));
            }
        }
    }

    // Reconstruct intermediate edges (source and target excluded).
    let end_face = found?;
    let mut path: Vec<EdgeID> = Vec::new();
    let mut current = end_face;
    loop {
        let (parent, edge_opt) = prev[&current];
        match edge_opt {
            None => break, // reached a start face
            Some(edge) => path.push(edge),
        }
        current = parent;
    }
    path.reverse();
    Some(path)
}

/// Fully blocks edges adjacent to a control point within a loop's edge sequence.
/// Both `e` and `twin(e)` are inserted, preventing any future loop from using those
/// edges in either direction. This guarantees 4 distinct arms at each intersection.
/// Adjacent edges that are themselves control points are skipped (they stay accessible
/// as Dijkstra start/end points for other loops).
fn block_adjacent_to_control_points(
    loop_edges: &[EdgeID],
    all_control_points: &HashSet<EdgeID>,
    blocked: &mut HashSet<EdgeID>,
    mesh: &Mesh<INPUT>,
) {
    let len = loop_edges.len();
    for i in 0..len {
        if all_control_points.contains(&loop_edges[i]) {
            for &adj_i in &[(i + len - 1) % len, (i + 1) % len] {
                let adj = loop_edges[adj_i];
                if !all_control_points.contains(&adj) {
                    blocked.insert(adj);
                    blocked.insert(mesh.twin(adj));
                }
            }
        }
    }
}

/// Records a committed loop's edge occupancy into `blocked`.
///
/// For every edge `e` the loop uses we always block `twin(e)` (no other loop may cross
/// it in the opposite direction). For NON-control-point edges we additionally block `e`
/// itself, so the geometric edge is fully occupied and no later loop can reuse it in
/// either direction. Control-point edges are exempt on the forward half so the designated
/// crossings can still occur there.
///
/// This is the geometric-edge–occupancy rule. The earlier `map(twin)`-only blocking was
/// inherited from the flow-graph representation (where a loop carries BOTH halves of every
/// crossed edge, so blocking all twins already blocked both halves). The skeleton router
/// carries a single half-edge per crossing, so twin-only blocking left the forward halves
/// open, letting later loops run over committed loops and form illegal non-CP crossings.
fn block_loop_occupancy(
    loop_edges: &[EdgeID],
    all_control_points: &HashSet<EdgeID>,
    blocked: &mut HashSet<EdgeID>,
    mesh: &Mesh<INPUT>,
) {
    for &e in loop_edges {
        blocked.insert(mesh.twin(e));
        let is_cp = all_control_points.contains(&e) || all_control_points.contains(&mesh.twin(e));
        if !is_cp {
            blocked.insert(e);
        }
    }
}

/// Given a quad side `side` of the control-point edge `cp` (an edge in one of `cp`'s two
/// incident triangles), returns the diagonally-opposite side: the edge in the OTHER incident
/// triangle that shares NO vertex with `side`. Returns `None` if `side` is not a side of one
/// of `cp`'s two triangles.
///
/// The crossing rule: a loop crossing at `cp` must enter one triangle and leave the other via
/// diagonal-partner sides (so it cuts diagonally across the quad). The two loops meeting at `cp`
/// then occupy the two distinct diagonals, giving the 4 distinct arms `Dual` requires.
fn quad_diagonal_partner(side: EdgeID, cp: EdgeID, mesh: &Mesh<INPUT>) -> Option<EdgeID> {
    let side_face = mesh.face(side);
    let cp_face = mesh.face(cp);
    let cp_twin_face = mesh.face(mesh.twin(cp));
    let other_face = if side_face == cp_face {
        cp_twin_face
    } else if side_face == cp_twin_face {
        cp_face
    } else {
        return None;
    };
    let a = mesh.root(side);
    let b = mesh.toor(side);
    mesh.edges(other_face).find(|&e| {
        let r = mesh.root(e);
        let t = mesh.toor(e);
        r != a && r != b && t != a && t != b
    })
}

fn pathing_for_loops(
    boundary_map: BiHashMap<EdgeIndex, LoopID>,
    crossings: CrossingMap,
    face_points: FacePointMap,
    skeleton: &LabeledCurveSkeleton,
    mesh: &Mesh<INPUT>,
    map: &mut SlotMap<LoopID, Loop>,
    diagnostics: &mut RoutingDiagnostics,
) {
    // All edges that serve as intersection points between loops of different axis types.
    // Edges adjacent to these in a completed loop's path are fully blocked (both directions)
    // to guarantee 4 distinct arms at every intersection as required by the dual structure.
    let all_control_points: HashSet<EdgeID> = crossings
        .values()
        .flat_map(|m| m.values().copied())
        .chain(face_points.values().flat_map(|m| m.values().copied()))
        .collect();

    for &loop_axis in &ALL_DIRS {
        // Crossings visited by this loop axis: a crossing on a boundary with direction D and
        // dir_sign (A, s) is visited by loops with axis third(D, A), so filter by that.
        let mut unvisited_crossings: HashSet<(LoopID, (PrincipalDirection, AxisSign))> = crossings
            .iter()
            .flat_map(|(&loop_id, dir_sign_map)| {
                let &edge_idx = boundary_map
                    .get_by_right(&loop_id)
                    .expect("every loop has a boundary edge");
                let boundary_dir = skeleton
                    .edge_weight(edge_idx)
                    .expect("edge must exist")
                    .direction;
                dir_sign_map
                    .keys()
                    .filter(|(a, _)| third(boundary_dir, *a) == loop_axis)
                    .map(|&ds| (loop_id, ds))
                    .collect::<Vec<_>>()
            })
            .collect();

        // Face points visited by this loop axis: any face point whose dir_sign direction ≠ loop_axis.
        // (Each face point is visited once per orthogonal loop type, so we rebuild per axis.)
        let mut unvisited_face_points: HashSet<(NodeIndex, (PrincipalDirection, AxisSign))> =
            face_points
                .iter()
                .flat_map(|(&node, dir_sign_map)| {
                    dir_sign_map
                        .keys()
                        .filter(|(dir, _)| *dir != loop_axis)
                        .map(|&ds| (node, ds))
                        .collect::<Vec<_>>()
                })
                .collect();

        // Blocked edges: geometric-edge occupancy of all loops established before this
        // direction. Both halves of every non-control-point edge are blocked (true
        // edge-disjointness), and only the reverse half of control-point edges, so the
        // designated crossings remain shareable. Additionally, edges adjacent to control
        // points are fully blocked to guarantee 4 distinct arms at every intersection.
        let mut blocked: HashSet<EdgeID> = HashSet::new();
        for l in map.values() {
            block_loop_occupancy(&l.edges, &all_control_points, &mut blocked, mesh);
        }
        for l in map.values() {
            block_adjacent_to_control_points(&l.edges, &all_control_points, &mut blocked, mesh);
        }

        // Repeatedly pick any unvisited point and trace the full loop it belongs to.
        while !unvisited_crossings.is_empty() || !unvisited_face_points.is_empty() {
            let start = if let Some(&(loop_id, dir_sign)) = unvisited_crossings.iter().next() {
                NextPoint::Crossing { loop_id, dir_sign }
            } else {
                let &(patch, dir_sign) = unvisited_face_points.iter().next().unwrap();
                NextPoint::FacePoint { patch, dir_sign }
            };

            // First pass: collect control-point edges in order.
            let mut current = start;
            let mut control_points: Vec<EdgeID> = Vec::new();
            loop {
                let edge_id = match current {
                    NextPoint::Crossing { loop_id, dir_sign } => crossings[&loop_id][&dir_sign],
                    NextPoint::FacePoint { patch, dir_sign } => face_points[&patch][&dir_sign],
                };
                match current {
                    NextPoint::Crossing { loop_id, dir_sign } => {
                        unvisited_crossings.remove(&(loop_id, dir_sign));
                    }
                    NextPoint::FacePoint { patch, dir_sign } => {
                        unvisited_face_points.remove(&(patch, dir_sign));
                    }
                }
                control_points.push(edge_id);
                current = next_point(current, loop_axis, skeleton, &boundary_map);
                if current == start {
                    break;
                }
            }

            // Second pass: connect consecutive control points via surface path. Each crossing
            // must be diagonal: a loop leaves a control point via the quad-diagonal partner of
            // the side it arrived on, so it cuts straight across the crossing edge's two
            // triangles. `forced_first` enforces this departure for every segment after the
            // first; `forced_last` closes the loop by making the final segment arrive at the
            // start control point matching the first segment's departure.
            let n = control_points.len();
            let mut loop_edges = Vec::new();
            let mut used_in_loop: HashSet<EdgeID> = HashSet::new();
            let mut path_ok = true;
            let mut seg0_first: Option<EdgeID> = None;
            for i in 0..n {
                let src = control_points[i];
                let tgt = control_points[(i + 1) % n];

                // Depart `src` via the diagonal partner of how the previous segment arrived
                // (its last edge `prev_last`; `twin(prev_last)` is that edge as seen from src's
                // arrival triangle). The first segment has no predecessor, so it starts free.
                let forced_first = if i == 0 {
                    None
                } else {
                    let prev_last = *loop_edges.last().expect("previous segment pushed edges");
                    quad_diagonal_partner(mesh.twin(prev_last), src, mesh)
                };

                // The closing segment must arrive at the start control point (`tgt == cp0`) on
                // the diagonal partner of the first segment's departure side, so that crossing
                // is diagonal too. `forced_last` is the edge crossed INTO the arrival triangle.
                let forced_last = if i == n - 1 {
                    seg0_first
                        .and_then(|f| quad_diagonal_partner(f, tgt, mesh))
                        .map(|partner| mesh.twin(partner))
                } else {
                    None
                };

                loop_edges.push(src);
                used_in_loop.insert(src);
                match surface_path_intermediates(src, tgt, forced_first, forced_last, &blocked, &used_in_loop, mesh) {
                    Some(inter) => {
                        if i == 0 {
                            seg0_first = inter.first().copied();
                        }
                        // Honest self-avoidance: block BOTH halves of each intermediate so the
                        // loop cannot later run back along its own path (anti-parallel via the
                        // twin) and close a degenerate self-overlapping cycle. Forward-only
                        // blocking left the twins open — the intra-loop version of the
                        // inter-loop occupancy bug.
                        for &e in &inter {
                            used_in_loop.insert(e);
                            used_in_loop.insert(mesh.twin(e));
                        }
                        loop_edges.extend(inter);
                    }
                    None => {
                        error!(
                            "No surface path from {:?} to {:?} for {:?}-loop (control point {}/{})",
                            src, tgt, loop_axis, i + 1, n
                        );
                        path_ok = false;
                        diagnostics.failed_segments.push((src, tgt));
                    }
                }
            }

            if path_ok {
                // Record this loop's edge occupancy: both halves of non-CP edges (so future
                // loops cannot reuse them in either direction), reverse half of CP edges.
                block_loop_occupancy(&loop_edges, &all_control_points, &mut blocked, mesh);
                // 4-arm guarantee: fully block edges adjacent to control points.
                block_adjacent_to_control_points(&loop_edges, &all_control_points, &mut blocked, mesh);
                map.insert(Loop { edges: loop_edges, direction: loop_axis });
            } else {
                // Dropped loop: record how far it got so the GUI can show where it broke.
                // `blocked` is untouched — the dropped loop leaves no routing trace.
                diagnostics.dropped_loops.push((loop_axis, loop_edges));
            }
        }
    }
}

/// A point on the surface that lies on a loop.
///
/// - `Crossing`: on a boundary between two patches.
///   `dir_sign = (A, s)` is the CrossingMap key, where `A = third(loop_axis, boundary_dir)`
///   and `s` is the sign of the slot the loop was at *before* crossing this boundary.
///   An L-loop only visits crossings whose `dir_sign` direction ≠ L.
/// - `FacePoint`: on a node patch. `dir_sign = (A, s)` is the key into `FacePointMap[patch]`,
///   representing the face the loop is currently sitting on.
///   An L-loop only visits face points whose `dir_sign` direction ≠ L.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum NextPoint {
    Crossing { loop_id: LoopID, dir_sign: (PrincipalDirection, AxisSign) },
    FacePoint { patch: NodeIndex, dir_sign: (PrincipalDirection, AxisSign) },
}

/// Returns the next `(direction, sign)` slot in CCW order for an `L`-loop currently at `(dir, sign)`.
///
/// CCW is defined as: when viewed from the `+L` direction, rotation goes from one orthogonal
/// axis to the next via the right-hand cross product: `L × (dir * sign)`.
/// This always maps between the two non-L directions, cycling through 4 slots.
fn ccw_next(
    loop_axis: PrincipalDirection,
    dir: PrincipalDirection,
    sign: AxisSign,
) -> (PrincipalDirection, AxisSign) {
    use PrincipalDirection::{X, Y, Z};
    // Cross product table (positive results): X×Y=+Z, Y×Z=+X, Z×X=+Y.
    // Swapping operands negates: X×Z=-Y, Y×X=-Z, Z×Y=-X.
    let cross_positive = matches!((loop_axis, dir), (X, Y) | (Y, Z) | (Z, X));
    let res_dir = third(loop_axis, dir);
    let res_sign = if cross_positive { sign } else { sign.flipped() };
    (res_dir, res_sign)
}

/// Core traversal step: given a node and the slot `(A, s)` the loop is currently at,
/// returns the next `NextPoint` for `loop_axis`.
///
/// - Computes `next_slot = ccw_next(loop_axis, A, s)`.
/// - If the node has an edge in the `next_slot` direction+sign: we cross that boundary ->
///   returns `Crossing` with `dir_sign = (A, s)` (the slot we departed from).
/// - Otherwise: we stay on the same patch -> returns `FacePoint` with `dir_sign = next_slot`.
fn next_from_node_slot(
    node: NodeIndex,
    slot: (PrincipalDirection, AxisSign),
    loop_axis: PrincipalDirection,
    skeleton: &LabeledCurveSkeleton,
    boundary_map: &bimap::BiHashMap<EdgeIndex, LoopID>,
) -> NextPoint {
    let (a, s) = slot;
    let next_slot = ccw_next(loop_axis, a, s);
    let (next_dir, next_sign) = next_slot;

    // Check whether this node has a skeleton edge in the next_slot direction.
    let edge_in_next_dir: Option<EdgeIndex> = skeleton
        .edges(node)
        .find(|e| {
            let sign_from_node = skeleton
                .edge_sign_from(e.id(), node)
                .expect("node must be endpoint");
            e.weight().direction == next_dir && sign_from_node == next_sign
        })
        .map(|e| e.id());

    match edge_in_next_dir {
        Some(edge_idx) => {
            // Cross the boundary: crossing key = slot we departed from.
            let &loop_id = boundary_map.get_by_left(&edge_idx).expect("edge must have a loop");
            NextPoint::Crossing { loop_id, dir_sign: slot }
        }
        None => {
            // Rotate within the same patch.
            NextPoint::FacePoint { patch: node, dir_sign: next_slot }
        }
    }
}

/// Given a `current` surface point and the `loop_axis`, returns the unique next point
/// in CCW traversal order.
///
/// Two cases:
///
/// **FacePoint** — delegate directly to `next_from_node_slot`.
///
/// **Crossing** — determine which endpoint node we enter (via `ccw_next` on the crossing slot,
/// which gives the direction+sign we move through the boundary), compute the incoming slot
/// on that node (boundary direction with flipped sign), then delegate to `next_from_node_slot`.
fn next_point(
    current: NextPoint,
    loop_axis: PrincipalDirection,
    skeleton: &LabeledCurveSkeleton,
    boundary_map: &bimap::BiHashMap<EdgeIndex, LoopID>,
) -> NextPoint {
    match current {
        NextPoint::FacePoint { patch, dir_sign } => {
            next_from_node_slot(patch, dir_sign, loop_axis, skeleton, boundary_map)
        }

        NextPoint::Crossing { loop_id, dir_sign: (a, s) } => {
            // The boundary direction D = third(loop_axis, a).
            // ccw_next tells us which direction+sign we move through the boundary.
            let (move_dir, move_sign) = ccw_next(loop_axis, a, s);
            // move_dir == third(loop_axis, a) == the boundary's direction.

            // Find the boundary edge and determine which endpoint node we enter.
            let &edge_idx = boundary_map.get_by_right(&loop_id).expect("loop must have an edge");
            let (src, tgt) = skeleton.edge_endpoints(edge_idx).expect("edge must exist");

            // The edge is stored with a sign from src->tgt. Compare to move_sign to pick the node.
            let edge_sign_from_src = skeleton
                .edge_sign_from(edge_idx, src)
                .expect("src is an endpoint");
            let entered_node = if move_sign == edge_sign_from_src {
                tgt  // moving in the same direction as src->tgt means we enter tgt
            } else {
                src
            };

            // We entered through the face opposite to move_sign.
            let incoming_slot = (move_dir, move_sign.flipped());
            next_from_node_slot(entered_node, incoming_slot, loop_axis, skeleton, boundary_map)
        }
    }
}

/// Returns the unique direction that is neither `a` nor `b`.
fn third(a: PrincipalDirection, b: PrincipalDirection) -> PrincipalDirection {
    match (a, b) {
        (PrincipalDirection::X, PrincipalDirection::Y)
        | (PrincipalDirection::Y, PrincipalDirection::X) => PrincipalDirection::Z,
        (PrincipalDirection::X, PrincipalDirection::Z)
        | (PrincipalDirection::Z, PrincipalDirection::X) => PrincipalDirection::Y,
        (PrincipalDirection::Y, PrincipalDirection::Z)
        | (PrincipalDirection::Z, PrincipalDirection::Y) => PrincipalDirection::X,
        _ => panic!("directions must be different"),
    }
}
