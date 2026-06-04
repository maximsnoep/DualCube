use std::{
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
    f64::consts::PI,
};

use bimap::BiHashMap;
use ordered_float::OrderedFloat;
use log::{error, warn};
use mehsh::prelude::{HasEdges, HasFaces, HasNormal, HasPosition, HasVertices, Mesh, Vector3D};
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
    let (boundary_map, mut crossings) = get_boundaries_and_crossing_points(skeleton, mesh, &mut map);
    let mut face_points = compute_face_points(skeleton, mesh);

    // A face point can land on an interior edge whose crossing quad already contains a boundary-loop
    // edge (one of the 4 arms the two orthogonal loops need), making the 4-arm crossing unroutable.
    // Relocate such face points to a clear interior edge of the same patch.
    repair_face_points(skeleton, mesh, &map, &mut face_points);

    // Slide each boundary crossing onto a straight, threadable boundary edge so the boundary
    // loop crosses diagonally (leaving the complementary diagonal free for the orthogonal loop)
    // and the orthogonal loop can actually pass through without being boxed in. Topological.
    repair_boundary_crossings(&map, &mut crossings, mesh);

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

    // Normalize loop winding so all loops of the SAME axis wind the same way about that axis.
    // The downstream `Dual` derives each segment's side label (Forwards/Backwards) from the loop's
    // stored edge ORDER; if two parallel same-axis loops are stored with opposite winding, a region
    // between them gets the same (axis, side) label twice → Property 3 ("Invalid face boundary").
    // `ccw_next` is supposed to give a consistent winding but inherits any orthogonalization sign
    // inconsistency, so we enforce consistency directly here via the loop's signed area about its
    // axis (reversing the edge order flips the winding; it preserves the canonical twin/same-face
    // alternation since that relation is symmetric). The absolute choice is irrelevant — a global
    // per-axis flip is just a mirror, which `realize` already canonicalizes — only consistency matters.
    for (_, l) in map.iter_mut() {
        if signed_area_about_axis(&l.edges, l.direction, mesh) < 0.0 {
            l.edges.reverse();
        }
    }

    Ok((map, crossings, face_points, diagnostics))
}

/// Signed area of a loop projected onto the plane perpendicular to `axis` (shoelace over edge
/// midpoints). Its sign encodes the loop's winding direction about `axis`; only the sign, and only
/// its consistency across loops of the same axis, is meaningful here.
fn signed_area_about_axis(edges: &[EdgeID], axis: PrincipalDirection, mesh: &Mesh<INPUT>) -> f64 {
    let project = |p: Vector3D| -> (f64, f64) {
        match axis {
            PrincipalDirection::X => (p.y, p.z),
            PrincipalDirection::Y => (p.z, p.x),
            PrincipalDirection::Z => (p.x, p.y),
        }
    };
    let pts: Vec<(f64, f64)> = edges
        .iter()
        .map(|&e| project(edge_midpoint_pos(e, mesh)))
        .collect();
    let n = pts.len();
    let mut area = 0.0;
    for i in 0..n {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % n];
        area += x0 * y1 - x1 * y0;
    }
    area * 0.5
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

        // Patch border for centrality-based placement (keep face points off the rim).
        let border_positions = patch_border_positions(skeleton, node_idx, mesh);

        // Fill missing slots (only directions without a skeleton edge)
        let mut interior_points: HashMap<(PrincipalDirection, AxisSign), EdgeID> = HashMap::new();
        let mut chosen: HashSet<EdgeID> = HashSet::new();

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

                // Most central (far-from-border) candidate, steered toward `target_dir`.
                let best = best_face_point_candidate(
                    &candidates,
                    &chosen,
                    node_pos,
                    target_dir,
                    &border_positions,
                    mesh,
                )
                .expect("candidates is non-empty");

                chosen.insert(best);
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

/// Repair pass after [`compute_face_points`]: a face point can land on an interior edge whose
/// crossing quad already contains a boundary-loop edge (one of the 4 arms the two orthogonal loops
/// need). That arm is permanently occupied by the boundary, so the 4-arm crossing can't form and
/// the loops through this face point are unroutable. For each such face point, relocate it to the
/// best-direction-aligned interior edge of the same patch whose quad is clear of boundary edges,
/// preserving the original placement direction. Leaves it in place (logged) if the patch offers no
/// clear edge (a genuine mesh-resolution limit). Does not touch the working placement logic.
fn repair_face_points(
    skeleton: &LabeledCurveSkeleton,
    mesh: &Mesh<INPUT>,
    map: &SlotMap<LoopID, Loop>,
    face_points: &mut FacePointMap,
) {
    // All boundary-loop edges (both halves). At this stage `map` holds only the boundary loops.
    let mut boundary_edges: HashSet<EdgeID> = HashSet::new();
    for l in map.values() {
        for &e in &l.edges {
            boundary_edges.insert(e);
            boundary_edges.insert(mesh.twin(e));
        }
    }
    let quad_clear = |cp: EdgeID| -> bool {
        mesh.quad(cp)
            .iter()
            .all(|&arm| !boundary_edges.contains(&arm) && !boundary_edges.contains(&mesh.twin(arm)))
    };

    let (mut invalid, mut repaired, mut unrepairable) = (0usize, 0usize, 0usize);

    for (&node_idx, slots) in face_points.iter_mut() {
        let node = &skeleton[node_idx];
        let node_pos = node.skeleton_node.position;
        let patch_set: HashSet<VertID> = node.skeleton_node.patch_vertices.iter().copied().collect();

        // Same candidate pool as `compute_face_points`: patch-interior mesh edges that are not
        // themselves boundary-loop edges. Additionally require a clear quad here.
        let mut seen: HashSet<(VertID, VertID)> = HashSet::new();
        let mut clear_candidates: Vec<EdgeID> = Vec::new();
        for &vert in &node.skeleton_node.patch_vertices {
            for face in mesh.faces(vert) {
                let verts: Vec<VertID> = mesh.vertices(face).collect();
                for i in 0..verts.len() {
                    let a = verts[i];
                    let b = verts[(i + 1) % verts.len()];
                    if !patch_set.contains(&a) || !patch_set.contains(&b) {
                        continue;
                    }
                    let key = if a < b { (a, b) } else { (b, a) };
                    if !seen.insert(key) {
                        continue;
                    }
                    if let Some((e, _)) = mesh.edge_between_verts(a, b) {
                        if boundary_edges.contains(&e) || boundary_edges.contains(&mesh.twin(e)) {
                            continue;
                        }
                        if quad_clear(e) {
                            clear_candidates.push(e);
                        }
                    }
                }
            }
        }

        let border_positions = patch_border_positions(skeleton, node_idx, mesh);
        // Slots that already sit on a clear quad keep their edge; reserve those so a relocation
        // can't land on top of them.
        let mut chosen: HashSet<EdgeID> = slots
            .values()
            .filter(|&&cp| quad_clear(cp))
            .copied()
            .collect();

        for cp in slots.values_mut() {
            if quad_clear(*cp) {
                continue;
            }
            invalid += 1;
            // Keep the face point as close as possible to its original direction from the node,
            // but relocate using the same centrality-biased scoring as initial placement.
            let target = (edge_midpoint_pos(*cp, mesh) - node_pos).normalize();
            match best_face_point_candidate(
                &clear_candidates,
                &chosen,
                node_pos,
                target,
                &border_positions,
                mesh,
            ) {
                Some(new_cp) => {
                    *cp = new_cp;
                    chosen.insert(new_cp);
                    repaired += 1;
                }
                None => unrepairable += 1,
            }
        }
    }

    if invalid > 0 {
        warn!(
            "face points: {} had a boundary edge in their quad, {} relocated, {} unrepairable",
            invalid, repaired, unrepairable
        );
    }
}

/// Position of an edge's midpoint.
fn edge_midpoint_pos(e: EdgeID, mesh: &Mesh<INPUT>) -> Vector3D {
    let a = mesh.position(mesh.root(e));
    let b = mesh.position(mesh.toor(e));
    (a + b) * 0.5
}

/// Midpoint positions of the patch border around a skeleton node: the edge midpoints of every
/// incident skeleton edge's boundary loop. Used to score face-point placement by distance from the
/// border (see [`face_point_score`]).
fn patch_border_positions(
    skeleton: &LabeledCurveSkeleton,
    node_idx: NodeIndex,
    mesh: &Mesh<INPUT>,
) -> Vec<Vector3D> {
    let mut positions = Vec::new();
    for edge_ref in skeleton.edges(node_idx) {
        for &e in &edge_ref.weight().boundary_loop.edge_midpoints {
            positions.push(edge_midpoint_pos(e, mesh));
        }
    }
    positions
}

/// Pick the best face-point candidate edge for a slot. The score balances two competing forces:
///
/// - **Alignment** (the strong *spreading* force, like the old angle heuristic): the normalized dot
///   of the candidate's direction-from-node against the slot's `target_dir`, in `[-1, 1]`. This is
///   what pushes the (up to) six slots of a node apart into distinct angular sectors so they don't
///   collapse together — essential for low-degree nodes with many face points sharing one patch.
/// - **Centrality** (the *anti-rim* force): distance from the patch border, normalized by the most
///   interior candidate's distance to `[0, 1]`. This keeps the chosen edge off the rim so the two
///   crossing loops pass through without detouring out and back (the zigzag cause).
///
/// `score = centrality + FP_DIRECTION_BIAS * alignment`, maximized. With `FP_DIRECTION_BIAS` above
/// ~0.55 the alignment force dominates enough to prevent center-collapse, while staying low enough
/// that the anti-rim force still pulls the placement inward — the equilibrium is "in the right
/// sector, a bit inside the rim." Candidates already chosen for another slot of the same node are
/// preferred-against (so two slots can't land on one edge); falls back to the best overall when all
/// are taken. With an empty border, centrality is 0 and placement is purely directional.
fn best_face_point_candidate(
    candidates: &[EdgeID],
    chosen: &HashSet<EdgeID>,
    node_pos: Vector3D,
    target_dir: Vector3D,
    border: &[Vector3D],
    mesh: &Mesh<INPUT>,
) -> Option<EdgeID> {
    let border_dist = |e: EdgeID| -> f64 {
        if border.is_empty() {
            0.0
        } else {
            let m = edge_midpoint_pos(e, mesh);
            border
                .iter()
                .map(|&p| (m - p).norm())
                .fold(f64::INFINITY, f64::min)
        }
    };
    // Interiority scale = how far the most interior candidate sits from the border.
    let max_border_dist = candidates
        .iter()
        .map(|&e| border_dist(e))
        .fold(0.0_f64, f64::max);

    let score = |e: EdgeID| -> f64 {
        let centrality = if max_border_dist > 0.0 {
            border_dist(e) / max_border_dist
        } else {
            0.0
        };
        let alignment = (edge_midpoint_pos(e, mesh) - node_pos)
            .normalize()
            .dot(&target_dir);
        centrality + FP_DIRECTION_BIAS * alignment
    };
    let cmp = |&e1: &EdgeID, &e2: &EdgeID| {
        score(e1)
            .partial_cmp(&score(e2))
            .unwrap_or(std::cmp::Ordering::Equal)
    };
    candidates
        .iter()
        .copied()
        .filter(|e| !chosen.contains(e))
        .max_by(cmp)
        .or_else(|| candidates.iter().copied().max_by(cmp))
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

/// Control-point "moat": a steep, distance-graded cost penalty for routing a loop's body
/// through faces near ANY control point. Indexed by face-hop distance to the nearest control
/// point; distances beyond the array length incur no penalty (factor 1.0).
///
/// Why: two perpendicular loops crossing at a control point must each leave room for the other.
/// Without this, a loop that turns one step after crossing runs its body straight down the
/// partner's exit corridor and walls it off. Making the crossing neighbourhood expensive pushes
/// loops to leave crossings promptly and swing wide when they must turn, so the partner can
/// always thread out. This is a pure cost (Dijkstra still finds any existing path), so it can
/// never make the loop being routed fail or reintroduce illegal crossings — it only reshapes the
/// chosen path to leave space for later loops.
const CP_MOAT_PENALTY: [f64; 4] = [30.0, 10.0, 4.0, 2.0];

/// Largest face-hop distance the moat field records (= last penalized distance). Beyond this,
/// faces are absent from the field and incur no penalty.
const CP_MOAT_RADIUS: u32 = (CP_MOAT_PENALTY.len() as u32) - 1;

/// Alignment routing cost (mirrors the flow-graph loop cost in `gui/src/main.rs`).
///
/// Each routing step turns within a face, entering by one edge and leaving by another. The
/// in-face travel direction `d` (between the two edge midpoints), rotated 90° about the face
/// normal `n`, gives the loop's local *separating* direction `d × n`, which should align with the
/// loop's *signed* target Δ (oriented by the loop's fixed winding direction; see `winding_sign` in
/// `surface_path_intermediates`). We charge a per-step penalty for misalignment, **integrated over
/// arc length**: `step_cost = length * (LAMBDA_DIST + W_ALIGN * misalignment)`, where
/// `misalignment = 1 - cos∠(d × n, ±Δ) ∈ [0, 2]` (0 = forward-aligned, 1 = perpendicular,
/// 2 = reversed winding).
///
/// - Signed, NOT folded. Folding with `|cos|` made winding forward and backward cost the same, so a
///   zigzag (alternating across the axis) was free — the zigzag source. Signing it against the
///   loop's single fixed winding makes reversal cost ~2×, so a smooth bulge always beats a zigzag
///   even past a badly-placed crossing.
/// - Scale-invariant: both terms scale with `length`, so relative path costs do not depend on mesh
///   size. This is the arc-length integral `∫ (λ + W·misalignment) ds`, the continuous "niceness"
///   functional, rather than the flow graph's per-turn `angle³` (its state is turns, ours is faces).
/// - `LAMBDA_DIST` is the distance floor: it keeps the old geodesic behavior as a regularizer so
///   that among equally-aligned paths the shorter/compacter one wins and the router cannot meander
///   for free. Alignment is *primary* (`W_ALIGN ≫ LAMBDA_DIST`); distance only breaks ties.
/// - The moat / shared-edge multipliers still apply on top, so this never makes a path infeasible.
const W_ALIGN: f64 = 8.0;

/// Distance-floor weight in the routing cost; see [`W_ALIGN`]. Keeping it at the old base unit
/// (1.0) preserves geodesic behavior as a tie-breaker/regularizer beneath the alignment term.
const LAMBDA_DIST: f64 = 1.0;

/// Weight of the directional (alignment) force relative to the centrality (anti-rim) force when
/// placing face points; both are normalized to ~unit scale, so this is dimensionless. It must be
/// large enough that alignment spreads a node's slots into distinct sectors (no center-collapse)
/// yet small enough that centrality still pulls each placement off the patch rim. See
/// [`best_face_point_candidate`].
const FP_DIRECTION_BIAS: f64 = 1.5;

/// Dijkstra's on the mesh dual graph to find the best intermediate path between two
/// control-point edges. The step cost is the alignment-primary, distance-regularized measure
/// described on [`W_ALIGN`] (a geodesic distance floor plus an arc-length-integrated misalignment
/// penalty that biases the path to follow the loop's principal axis), scaled by the moat /
/// shared-edge multipliers below.
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
    cp_distance: &HashMap<FaceID, u32>,
    loop_axis: PrincipalDirection,
    winding_sign: f64,
    mesh: &Mesh<INPUT>,
) -> Option<Vec<EdgeID>> {
    // Signed target Δ for the alignment cost (see `W_ALIGN`): the loop should locally separate
    // along its own principal axis, wound in a single CONSISTENT direction (`winding_sign`, fixed
    // per loop). Signing it — rather than folding with |cos| — makes a step that reverses the
    // winding cost ~2×, so a smooth bulge always beats a zigzag even past a badly-placed crossing.
    let signed_target: Vector3D = Vector3D::from(loop_axis) * winding_sign;
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
            let length = (centroid_a - edge_mid).norm() + (edge_mid - centroid_b).norm();

            // Alignment cost (see `W_ALIGN`). The in-face turn goes from the edge we ENTERED
            // `face` by (recorded in `prev`) to the edge `edge` we are leaving by; both midpoints
            // lie on `face`, so this is exactly the flow-graph quantity charged per face. Rotating
            // that displacement about the face normal and comparing to the target axis gives the
            // misalignment. Source faces have no entry edge, so the first step out of a source
            // incurs no alignment penalty.
            let mut misalignment = 0.0;
            if let Some((_, Some(entry_edge))) = prev.get(&face) {
                let d = edge_mid - edge_midpoint_pos(*entry_edge, mesh);
                let cross = d.cross(&mesh.normal(face));
                let cn = cross.norm();
                if cn > 1e-12 {
                    // Signed against the loop's fixed winding direction: 0 = forward-aligned,
                    // 1 = perpendicular, 2 = reversed winding. NOT folded, so reversal is costly.
                    misalignment = 1.0 - (cross / cn).dot(&signed_target);
                }
            }

            // Arc-length integral of (distance floor + misalignment): alignment is primary,
            // distance regularizes. Scale-invariant since both terms scale with `length`.
            let mut step_cost = length * (LAMBDA_DIST + W_ALIGN * misalignment);

            // Penalize entering a face adjacent to blocked edges.
            if face_touches_blocked(next_face, edge) {
                step_cost *= SHARED_EDGE_MULTIPLIER;
            }

            // Control-point moat: penalize routing the body near any crossing, steeply graded by
            // distance, so loops leave crossings promptly and swing wide rather than walling off a
            // perpendicular partner's exit corridor. Pure cost — never blocks an otherwise-valid path.
            if let Some(&d) = cp_distance.get(&next_face) {
                if (d as usize) < CP_MOAT_PENALTY.len() {
                    step_cost *= CP_MOAT_PENALTY[d as usize];
                }
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

/// Topological straightness (diamond) test for a boundary crossing at position `pos`:
/// the boundary's pre/post edges must be diagonal partners across the crossing edge, so the
/// boundary cuts straight through and leaves the complementary diagonal free for the
/// orthogonal loop. Pure connectivity (`quad_diagonal_partner` + `twin`), no geometry.
fn boundary_crossing_is_straight(edges: &[EdgeID], pos: usize, mesh: &Mesh<INPUT>) -> bool {
    let n = edges.len();
    if n < 3 {
        return false;
    }
    let cp = edges[pos];
    let prev = edges[(pos + n - 1) % n];
    let next = edges[(pos + 1) % n];
    quad_diagonal_partner(prev, cp, mesh) == Some(mesh.twin(next))
}

/// Cyclic open-interval test: is `pos` strictly between `lo_excl` and `hi_excl` going around
/// `[0, n)` (both endpoints excluded)?
fn in_open_cyclic(pos: usize, lo_excl: usize, hi_excl: usize) -> bool {
    if lo_excl == hi_excl {
        return pos != lo_excl;
    }
    if lo_excl < hi_excl {
        pos > lo_excl && pos < hi_excl
    } else {
        pos > lo_excl || pos < hi_excl
    }
}

/// Reachability gate: starting at face `start`, can a path that never crosses a boundary edge
/// (`walls` = every boundary loop's edges) reach a face touching NO boundary at all — i.e. the
/// patch INTERIOR? Reaching merely "off the current boundary" is too weak: a single clear edge
/// into a face that is then boxed in by another boundary one step later would falsely pass.
/// Requiring the interior means a crossing whose exit is trapped in any boundary pocket fails.
/// Pure connectivity, no geometry.
fn reaches_interior(start: FaceID, walls: &HashSet<EdgeID>, mesh: &Mesh<INPUT>) -> bool {
    let mut visited: HashSet<FaceID> = HashSet::new();
    let mut queue: VecDeque<FaceID> = VecDeque::new();
    visited.insert(start);
    queue.push_back(start);
    while let Some(f) = queue.pop_front() {
        if mesh.edges(f).all(|e| !walls.contains(&e)) {
            return true; // a fully-interior face: no boundary edge — escaped every pocket
        }
        for e in mesh.edges(f) {
            if walls.contains(&e) {
                continue;
            }
            let nf = mesh.face(mesh.twin(e));
            if visited.insert(nf) {
                queue.push_back(nf);
            }
        }
    }
    false
}

/// Searches outward (by boundary-sequence step distance) from `orig` for the nearest position
/// in the open cyclic interval `(lo_excl, hi_excl)` for which `ok` holds. Returns `None` if no
/// such position exists in the interval (then the crossing is left where it is).
fn nearest_ok_pos(
    n: usize,
    orig: usize,
    lo_excl: usize,
    hi_excl: usize,
    ok: impl Fn(usize) -> bool,
) -> Option<usize> {
    if in_open_cyclic(orig, lo_excl, hi_excl) && ok(orig) {
        return Some(orig);
    }
    for d in 1..n {
        let p = ((orig as isize + d as isize).rem_euclid(n as isize)) as usize;
        let q = ((orig as isize - d as isize).rem_euclid(n as isize)) as usize;
        let p_in = in_open_cyclic(p, lo_excl, hi_excl);
        let q_in = in_open_cyclic(q, lo_excl, hi_excl);
        if p_in && ok(p) {
            return Some(p);
        }
        if q_in && ok(q) {
            return Some(q);
        }
        if !p_in && !q_in {
            return None;
        }
    }
    None
}

/// Slides each boundary crossing onto a boundary edge that is both STRAIGHT (the boundary cuts
/// diagonally through it, so the orthogonal loop's complementary diagonal is free) and
/// THREADABLE (both faces of the crossing can escape this boundary's strip into the patch
/// interior, so the orthogonal loop isn't boxed in right after crossing). Movement is along the
/// boundary's own edge sequence and is constrained to the open interval between each crossing's
/// neighbours, so the cyclic order of the 4 crossings is preserved. Crossings with no valid
/// position in their interval are left in place and counted as unrepairable (a genuine
/// mesh-resolution limitation, surfaced via the routing-failure overlay).
///
/// Entirely topological: vertex-sharing straightness, connectivity BFS, and sequence-distance.
fn repair_boundary_crossings(
    map: &SlotMap<LoopID, Loop>,
    crossings: &mut CrossingMap,
    mesh: &Mesh<INPUT>,
) {
    // Walls = every boundary loop's edges (both halves). A crossing's side must escape into the
    // patch interior without crossing any boundary.
    let mut walls: HashSet<EdgeID> = HashSet::new();
    for l in map.values() {
        for &e in &l.edges {
            walls.insert(e);
            walls.insert(mesh.twin(e));
        }
    }

    let (mut invalid, mut repaired, mut unrepairable) = (0usize, 0usize, 0usize);
    let loop_ids: Vec<LoopID> = crossings.keys().copied().collect();
    for loop_id in loop_ids {
        let edges = &map[loop_id].edges;
        let n = edges.len();
        if n < 3 {
            continue;
        }
        let pos_of: HashMap<EdgeID, usize> =
            edges.iter().enumerate().map(|(i, &e)| (e, i)).collect();

        let ok = |pos: usize| -> bool {
            if !boundary_crossing_is_straight(edges, pos, mesh) {
                return false;
            }
            let cp = edges[pos];
            // Both sides of the crossing must reach the patch interior, so the orthogonal loop
            // isn't boxed in just after crossing.
            reaches_interior(mesh.face(cp), &walls, mesh)
                && reaches_interior(mesh.face(mesh.twin(cp)), &walls, mesh)
        };

        // Snapshot crossings of this boundary in current cyclic order; neighbours' ORIGINAL
        // positions bound each crossing's allowed interval, freezing the cyclic order.
        let mut entries: Vec<((PrincipalDirection, AxisSign), usize)> = {
            let Some(ds_map) = crossings.get(&loop_id) else { continue };
            if ds_map.is_empty() {
                continue;
            }
            ds_map
                .iter()
                .map(|(&k, &e)| (k, *pos_of.get(&e).expect("crossing must lie on its boundary")))
                .collect()
        };
        entries.sort_by_key(|(_, p)| *p);
        let m = entries.len();

        let mut new_pos: Vec<usize> = Vec::with_capacity(m);
        for k in 0..m {
            let orig = entries[k].1;
            let lo = entries[(k + m - 1) % m].1;
            let hi = entries[(k + 1) % m].1;
            let valid = ok(orig);
            if !valid {
                invalid += 1;
            }
            match nearest_ok_pos(n, orig, lo, hi, &ok) {
                Some(p) => {
                    if p != orig {
                        repaired += 1;
                    }
                    new_pos.push(p);
                }
                None => {
                    if !valid {
                        unrepairable += 1;
                    }
                    new_pos.push(orig);
                }
            }
        }

        let ds_mut = crossings.get_mut(&loop_id).unwrap();
        for (k, (key, _)) in entries.iter().enumerate() {
            ds_mut.insert(*key, edges[new_pos[k]]);
        }
    }

    if invalid > 0 || repaired > 0 {
        warn!(
            "boundary crossings: {} invalid (bend or boxed-in), {} repaired, {} unrepairable",
            invalid, repaired, unrepairable
        );
    }
}

/// Multi-source BFS giving each face its hop distance to the nearest control point, capped at
/// `CP_MOAT_RADIUS`. Faces farther than the cap are absent from the map (treated as no penalty).
fn control_point_distance_field(
    control_points: &HashSet<EdgeID>,
    mesh: &Mesh<INPUT>,
) -> HashMap<FaceID, u32> {
    let mut dist: HashMap<FaceID, u32> = HashMap::new();
    let mut queue: VecDeque<FaceID> = VecDeque::new();
    for &cp in control_points {
        for f in [mesh.face(cp), mesh.face(mesh.twin(cp))] {
            if dist.insert(f, 0).is_none() {
                queue.push_back(f);
            }
        }
    }
    while let Some(f) = queue.pop_front() {
        let d = dist[&f];
        if d >= CP_MOAT_RADIUS {
            continue;
        }
        for e in mesh.edges(f) {
            let nf = mesh.face(mesh.twin(e));
            if !dist.contains_key(&nf) {
                dist.insert(nf, d + 1);
                queue.push_back(nf);
            }
        }
    }
    dist
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

    // Face-hop distance from every face to the nearest control point, capped at the moat radius.
    // Used to steeply penalize routing a loop's body through crossing neighbourhoods so loops
    // leave room for the perpendicular partner crossing the same point (see `CP_MOAT_PENALTY`).
    let cp_distance = control_point_distance_field(&all_control_points, mesh);

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

            // Second pass: connect consecutive control points via surface path. Every crossing
            // must be diagonal AND oriented toward where the loop is actually going. For each
            // control point we pick its EXIT quad side as the one pointing toward the NEXT
            // control point (a globally-known direction — it never depends on the curving free
            // path, so no drift / spiral). The crossing's ENTRY side is that exit's diagonal
            // partner, keeping the crossing diagonal and leaving the perpendicular pair free for
            // the other loop. Each segment is then pinned at both ends: it must DEPART `src` via
            // `src`'s exit side and ARRIVE at `tgt` via the diagonal partner of `tgt`'s exit side.
            let n = control_points.len();
            // Winding sign of this loop about its axis, from the coarse control-point polygon.
            // Fixed for the whole loop so every segment's alignment cost references the SAME
            // winding direction — this is what makes reversing (zigzagging) consistently expensive
            // instead of free. If the convention is ever inverted, all loops would route backwards
            // (easy to spot), and flipping this sign fixes it.
            let winding_sign = if signed_area_about_axis(&control_points, loop_axis, mesh) >= 0.0 {
                -1.0
            } else {
                1.0
            };
            let mut loop_edges = Vec::new();
            let mut used_in_loop: HashSet<EdgeID> = HashSet::new();
            let mut path_ok = true;
            let mut seg0_first: Option<EdgeID> = None;
            for i in 0..n {
                let src = control_points[i];
                let tgt = control_points[(i + 1) % n];

                // Straight-through (topological) crossing: depart `src` via the unique diagonal
                // partner of the side the previous segment arrived on. `twin(prev_last)` is that
                // arrival edge as seen from `src`'s arrival triangle; its diagonal partner is the
                // opposite quad side, so the loop cuts straight across the two triangles. The
                // first segment has no predecessor, so it starts free.
                let forced_first = if i == 0 {
                    None
                } else {
                    let prev_last = *loop_edges.last().expect("previous segment pushed edges");
                    quad_diagonal_partner(mesh.twin(prev_last), src, mesh)
                };

                // The closing segment must arrive at the start control point matching the first
                // segment's departure, so that crossing is diagonal too. `forced_last` is the
                // edge crossed INTO the arrival triangle.
                let forced_last = if i == n - 1 {
                    seg0_first
                        .and_then(|f| quad_diagonal_partner(f, tgt, mesh))
                        .map(|partner| mesh.twin(partner))
                } else {
                    None
                };

                loop_edges.push(src);
                used_in_loop.insert(src);
                match surface_path_intermediates(src, tgt, forced_first, forced_last, &blocked, &used_in_loop, &cp_distance, loop_axis, winding_sign, mesh) {
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
