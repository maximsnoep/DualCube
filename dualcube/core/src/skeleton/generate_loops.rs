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

    // Face points (pinned interior crossings) are ONLY needed for a 1-node skeleton, whose loops are
    // all-interior with nothing to anchor on. With >=1 boundary, interior crossings happen naturally
    // via layered routing, so no face points are computed (and none are rendered).
    let single_node = skeleton.edge_references().count() == 0;
    // A 1-node skeleton has no boundary loops, so face points need no boundary-clearance repair.
    let face_points = if single_node {
        compute_face_points(skeleton, mesh)
    } else {
        FacePointMap::new()
    };

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

    // Routing produces one half-edge per crossed geometric edge; `Dual` requires both halves (it
    // canonicalizes crossings to the higher half-edge and its quad-walk expects each edge's twin
    // adjacent in the sequence). Expand AFTER pathing so blocking still sees the single-half form.
    for (_, l) in map.iter_mut() {
        let raw = std::mem::take(&mut l.edges);
        l.edges = expand_to_double_halfedges(raw, mesh);
    }

    // Normalize winding so all same-axis loops wind the same way. `Dual` derives each segment's side
    // label from the stored edge ORDER, so two parallel same-axis loops with opposite winding would
    // double a region's (axis, side) label → Property 3 ("Invalid face boundary"). Reversing the edge
    // list flips winding while preserving the twin/same-face alternation; only consistency matters.
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

/// Weight of the alignment penalty in the per-step routing cost:
/// `step_cost = length * (LAMBDA_DIST + W_ALIGN * θ^ALIGN_ALPHA)`, where `θ = ∠(d × n, ±Δ)` is the
/// angle between the loop's local separating direction (`d × n`, its in-face travel `d` rotated 90°
/// about the face normal) and its signed target axis Δ (signed by the loop's winding; 0 = forward-
/// aligned, π = reversed). The penalty is integrated over arc length and signed (not folded), so
/// winding reversal is maximal — this is what stops zigzagging. Alignment is primary
/// (`W_ALIGN ≫ LAMBDA_DIST`).
const W_ALIGN: f64 = 8.0;

/// Distance-floor weight in the routing cost (see [`W_ALIGN`]): a geodesic regularizer/tie-breaker
/// beneath the alignment term, so among equally-aligned paths the shorter one wins.
const LAMBDA_DIST: f64 = 1.0;

/// Exponent on the misalignment angle (radians), raw `θ^α` like the flow-graph / paper cost (paper
/// uses 10). Higher α punishes large angles ever more steeply (`α = 10`: 90° → `(π/2)^10 ≈ 92`,
/// 180° → `π^10 ≈ 9.4e4`), so loops strongly prefer the straight/aligned route even when longer,
/// while small turns stay cheap. See [`W_ALIGN`].
const ALIGN_ALPHA: i32 = 10;

/// Weight of the directional (alignment) force relative to the centrality (anti-rim) force when
/// placing face points; both are normalized to ~unit scale, so this is dimensionless. It must be
/// large enough that alignment spreads a node's slots into distinct sectors (no center-collapse)
/// yet small enough that centrality still pulls each placement off the patch rim. See
/// [`best_face_point_candidate`].
const FP_DIRECTION_BIAS: f64 = 1.5;

/// Dijkstra over the mesh dual graph (state = face) that routes from `source` to `target` while
/// crossing an ORDERED list of committed `partners` exactly once each, in order. The cost is the
/// alignment-primary, distance-regularized measure on [`W_ALIGN`]. The search state is
/// `(face, layer)` with `layer ∈ 0..=k` (`k = partners.len()`);
/// crossing an edge of `partners[layer]` is a one-way transition `layer -> layer+1`, realized as an
/// atomic straight-through (diagonal) crossing: the loop enters the crossing face on its arm, cuts
/// across the partner's edge, and exits on the diagonal partner (`quad_diagonal_partner`). Because
/// the partner's own arms are already in `blocked`, the loop necessarily enters/exits on the
/// complementary diagonal, so the 4 arms stay distinct *by construction* — no separate 4-arm check
/// is needed. Out-of-order partner crossings are forbidden. With `partners` empty this is a plain
/// shortest free path between the two anchors.
///
/// `forced_first`/`forced_last` pin the first/last step (the boundary-crossing diagonal at each
/// anchor). `blocked` holds committed loops' occupancy; `used` the current loop's own edges (self-
/// avoidance); `control_points` the boundary-crossing edges the body must not thread.
///
/// Returns the intermediate mesh edges (source/target exclusive), or `None` if no such path exists.
fn surface_path_layered(
    source: EdgeID,
    target: EdgeID,
    forced_first: Option<EdgeID>,
    forced_last: Option<EdgeID>,
    partners: &[HashSet<EdgeID>],
    blocked: &HashSet<EdgeID>,
    used: &HashSet<EdgeID>,
    control_points: &HashSet<EdgeID>,
    loop_axis: PrincipalDirection,
    winding_sign: f64,
    mesh: &Mesh<INPUT>,
) -> Option<Vec<EdgeID>> {
    let signed_target: Vector3D = Vector3D::from(loop_axis) * winding_sign;
    let k = partners.len();

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
    let in_partner = |e: EdgeID, j: usize| -> bool {
        partners[j].contains(&e) || partners[j].contains(&mesh.twin(e))
    };
    let in_any_partner = |e: EdgeID| -> bool { (0..k).any(|j| in_partner(e, j)) };
    let other_face = |edge: EdgeID, face: FaceID| -> Option<FaceID> {
        let a = mesh.root(edge);
        let b = mesh.toor(edge);
        let set_a: HashSet<FaceID> = mesh.faces(a).collect();
        mesh.faces(b).find(|&f| set_a.contains(&f) && f != face)
    };
    // Cost of stepping from `face` (entered via `entry`) across `edge` into `next_face`.
    let step_cost = |face: FaceID, entry: Option<EdgeID>, edge: EdgeID, next_face: FaceID| -> f64 {
        let centroid_a = face_centroid(face);
        let edge_mid = edge_midpoint_pos(edge, mesh);
        let centroid_b = face_centroid(next_face);
        let length = (centroid_a - edge_mid).norm() + (edge_mid - centroid_b).norm();
        let mut misalignment = 0.0;
        if let Some(en) = entry {
            let d = edge_mid - edge_midpoint_pos(en, mesh);
            let cross = d.cross(&mesh.normal(face));
            let cn = cross.norm();
            if cn > 1e-12 {
                // Angle (radians) between the loop's in-plane separating direction `d × n` and its
                // SIGNED target axis: 0 = forward-aligned, π = reversed winding. Penalized by the
                // paper's raw `θ^α` cost (NOT normalized): higher α punishes large angles ever more
                // steeply, so the router strongly prefers the straight (aligned) way even when it is
                // geometrically longer.
                let cos = (cross / cn).dot(&signed_target).clamp(-1.0, 1.0);
                misalignment = cos.acos().powi(ALIGN_ALPHA);
            }
        }
        length * (LAMBDA_DIST + W_ALIGN * misalignment)
    };

    let source_faces = faces_of(source);
    let target_face_set: HashSet<FaceID> = faces_of(target).into_iter().collect();

    // With no partners and an already-adjacent source/target, no intermediates needed.
    if k == 0 && source_faces.iter().any(|f| target_face_set.contains(f)) {
        return Some(vec![]);
    }

    type State = (FaceID, usize);
    let mut dist: HashMap<State, f64> = HashMap::new();
    // prev: state -> (parent_state, edges_used_to_reach_it). Empty edges for source states.
    let mut prev: HashMap<State, (State, Vec<EdgeID>)> = HashMap::new();
    let mut heap: BinaryHeap<(std::cmp::Reverse<OrderedFloat<f64>>, State)> = BinaryHeap::new();

    for &sf in &source_faces {
        let s = (sf, 0usize);
        dist.insert(s, 0.0);
        prev.insert(s, (s, Vec::new()));
        heap.push((std::cmp::Reverse(OrderedFloat(0.0)), s));
    }

    let mut found: Option<State> = None;

    'dijkstra: while let Some((std::cmp::Reverse(OrderedFloat(cost)), state)) = heap.pop() {
        let (face, layer) = state;
        if dist.get(&state).copied().unwrap_or(f64::INFINITY) < cost {
            continue; // stale
        }
        // Entry edge into `face` AS SEEN IN `face` is the twin of the last edge used to reach it.
        let entry_in_face: Option<EdgeID> = prev
            .get(&state)
            .and_then(|(_, edges)| edges.last().copied())
            .map(|e| mesh.twin(e));
        let is_source = entry_in_face.is_none();

        let face_verts: Vec<VertID> = mesh.vertices(face).collect();
        let nfv = face_verts.len();
        for i in 0..nfv {
            let a = face_verts[i];
            let b = face_verts[(i + 1) % nfv];
            let Some((edge, _)) = mesh.edge_between_verts(a, b) else { continue };
            let Some(next_face) = other_face(edge, face) else { continue };

            // One-way transition: cross partner[layer] exactly once, straight through.
            if layer < k && in_partner(edge, layer) {
                let Some(entry) = entry_in_face else { continue }; // need an arm to cut across
                let Some(exit_arm) = quad_diagonal_partner(entry, edge, mesh) else { continue };
                // The exit arm is the loop's own new edge — must be free (the partner's own arm is
                // in `blocked`, so this rejects the degenerate same-diagonal-as-partner crossing).
                if blocked.contains(&exit_arm) || used.contains(&exit_arm) {
                    continue;
                }
                if control_points.contains(&exit_arm) || control_points.contains(&mesh.twin(exit_arm)) {
                    continue;
                }
                let Some(land) = other_face(exit_arm, next_face) else { continue };
                let c1 = step_cost(face, entry_in_face, edge, next_face);
                let c2 = step_cost(next_face, Some(edge), exit_arm, land);
                let new_cost = cost + c1 + c2;
                let ns: State = (land, layer + 1);
                if new_cost < dist.get(&ns).copied().unwrap_or(f64::INFINITY) {
                    dist.insert(ns, new_cost);
                    prev.insert(ns, (state, vec![edge, exit_arm]));
                    if ns.1 == k && target_face_set.contains(&land) {
                        found = Some(ns);
                        break 'dijkstra;
                    }
                    heap.push((std::cmp::Reverse(OrderedFloat(new_cost)), ns));
                }
                continue;
            }

            // Never cross any other partner (wrong order); these are also in `blocked` already.
            if in_any_partner(edge) {
                continue;
            }

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
            if (control_points.contains(&edge) || control_points.contains(&mesh.twin(edge)))
                && Some(edge) != forced_first
                && Some(edge) != forced_last
            {
                continue;
            }

            // Target faces only finish at the final layer; the closing diagonal is forced there.
            let entering_target = target_face_set.contains(&next_face);
            let finishes = entering_target && layer == k;
            if finishes {
                if let Some(fl) = forced_last {
                    if edge != fl {
                        continue;
                    }
                }
            }

            let new_cost = cost + step_cost(face, entry_in_face, edge, next_face);
            let ns: State = (next_face, layer);
            if new_cost < dist.get(&ns).copied().unwrap_or(f64::INFINITY) {
                dist.insert(ns, new_cost);
                prev.insert(ns, (state, vec![edge]));
                if finishes {
                    found = Some(ns);
                    break 'dijkstra;
                }
                heap.push((std::cmp::Reverse(OrderedFloat(new_cost)), ns));
            }
        }
    }

    // Reconstruct intermediate edges (source/target exclusive).
    let mut state = found?;
    let mut rev: Vec<EdgeID> = Vec::new();
    loop {
        let (parent, edges) = &prev[&state];
        if edges.is_empty() {
            break; // reached a source state
        }
        for &e in edges.iter().rev() {
            rev.push(e);
        }
        state = *parent;
    }
    rev.reverse();
    Some(rev)
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

/// Records a committed loop's edge occupancy into `blocked`. For every edge `e` the loop uses we
/// block `twin(e)` (no other loop may cross it the opposite way); for non-control-point edges we
/// also block `e` itself, fully occupying the geometric edge. Control-point (boundary-crossing)
/// edges keep their forward half open so the designated crossing can still occur there.
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

fn pathing_for_loops(
    boundary_map: BiHashMap<EdgeIndex, LoopID>,
    crossings: CrossingMap,
    face_points: FacePointMap,
    skeleton: &LabeledCurveSkeleton,
    mesh: &Mesh<INPUT>,
    map: &mut SlotMap<LoopID, Loop>,
    diagnostics: &mut RoutingDiagnostics,
) {
    // A skeleton with no edges is a single patch (1 node): it has NO boundaries, so every loop is
    // all-interior and there is nothing to anchor on. That is the ONLY case that keeps pinned face
    // points. With >=1 boundary, every loop necessarily starts/ends at a boundary crossing.
    let single_node = skeleton.edge_references().count() == 0;

    // Pinned anchor edges = the patch-boundary crossings only. (Interior crossings are no longer
    // pinned; they happen naturally via layered routing, so the body must be free to route where one
    // would have been.) Edges adjacent to anchors in a committed loop are fully blocked to guarantee
    // 4 distinct arms at the boundary crossings; interior 4-arm is enforced dynamically.
    let all_control_points: HashSet<EdgeID> = crossings
        .values()
        .flat_map(|m| m.values().copied())
        .collect();

    // Interior-crossing partner resolution. An interior crossing is keyed `(node, slot)` and shared
    // by exactly the two loops of axes ≠ slot.dir. As each loop commits we register its interior
    // events here, so a later-routed loop can look up its already-committed partner at that crossing
    // (the perpendicular loop whose body it must cross once). When the partner is NOT yet committed,
    // the crossing is instead created later, when that partner routes and crosses THIS loop's body.
    let mut interior_partner: HashMap<(NodeIndex, (PrincipalDirection, AxisSign)), LoopID> =
        HashMap::new();

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

        // Blocked edges: geometric-edge occupancy of all loops committed before this axis. Both
        // halves of every non-anchor edge are blocked (true edge-disjointness), and only the reverse
        // half of boundary-crossing anchors, so the boundary crossings stay shareable. Edges adjacent
        // to anchors are fully blocked for the 4-arm guarantee at boundary crossings. Interior
        // crossings are instead realized by `surface_path_layered`'s one-way transitions, which
        // override `blocked` to cross a designated partner exactly once.
        let mut blocked: HashSet<EdgeID> = HashSet::new();
        for l in map.values() {
            block_loop_occupancy(&l.edges, &all_control_points, &mut blocked, mesh);
        }
        for l in map.values() {
            block_adjacent_to_control_points(&l.edges, &all_control_points, &mut blocked, mesh);
        }

        // Repeatedly pick any unvisited point and trace the full loop it belongs to. Loops are
        // seeded from boundary crossings; only a 1-node skeleton (no crossings) seeds from face
        // points (its loops are all-interior). With boundaries present, every loop is reached from a
        // crossing, so any leftover unvisited face points would indicate a missing loop.
        while !unvisited_crossings.is_empty() || (single_node && !unvisited_face_points.is_empty()) {
            let start = if let Some(&(loop_id, dir_sign)) = unvisited_crossings.iter().next() {
                NextPoint::Crossing { loop_id, dir_sign }
            } else {
                let &(patch, dir_sign) = unvisited_face_points.iter().next().unwrap();
                NextPoint::FacePoint { patch, dir_sign }
            };

            // First pass: collect the loop's crossing events in cyclic order.
            let mut current = start;
            let mut events: Vec<Event> = Vec::new();
            loop {
                let event = match current {
                    NextPoint::Crossing { loop_id, dir_sign } => {
                        unvisited_crossings.remove(&(loop_id, dir_sign));
                        Event::Boundary { edge: crossings[&loop_id][&dir_sign] }
                    }
                    NextPoint::FacePoint { patch, dir_sign } => {
                        unvisited_face_points.remove(&(patch, dir_sign));
                        Event::Interior { patch, slot: dir_sign }
                    }
                };
                events.push(event);
                current = next_point(current, loop_axis, skeleton, &boundary_map);
                if current == start {
                    break;
                }
            }

            // Second pass: route the loop. BOUNDARY events are pinned anchors; between consecutive
            // anchors the loop crosses an ORDERED list of committed partner loops (the interior
            // events) exactly once each, at router-chosen locations, via `surface_path_layered`.
            // Interior events whose partner is not yet committed are skipped here — that crossing is
            // created later, when the partner routes and crosses THIS loop's committed body.
            //
            // Winding sign about the axis, from a coarse polygon over the event positions (boundary
            // crossing midpoints + interior patch centres). Fixed for the whole loop so the alignment
            // cost references one consistent winding.
            let winding_sign = {
                let project = |p: Vector3D| -> (f64, f64) {
                    match loop_axis {
                        PrincipalDirection::X => (p.y, p.z),
                        PrincipalDirection::Y => (p.z, p.x),
                        PrincipalDirection::Z => (p.x, p.y),
                    }
                };
                let pts: Vec<(f64, f64)> = events
                    .iter()
                    .map(|e| {
                        project(match *e {
                            Event::Boundary { edge } => edge_midpoint_pos(edge, mesh),
                            Event::Interior { patch, .. } => {
                                skeleton[patch].skeleton_node.position
                            }
                        })
                    })
                    .collect();
                let mut area = 0.0;
                let np = pts.len();
                for i in 0..np {
                    let (x0, y0) = pts[i];
                    let (x1, y1) = pts[(i + 1) % np];
                    area += x0 * y1 - x1 * y0;
                }
                if area >= 0.0 { -1.0 } else { 1.0 }
            };

            let mut loop_edges = Vec::new();
            let mut used_in_loop: HashSet<EdgeID> = HashSet::new();
            let mut path_ok = true;

            // Cyclic indices of the boundary anchors within `events`.
            let anchor_idx: Vec<usize> = events
                .iter()
                .enumerate()
                .filter(|(_, e)| matches!(e, Event::Boundary { .. }))
                .map(|(i, _)| i)
                .collect();

            if anchor_idx.is_empty() {
                // A loop with no boundary anchor can ONLY occur on a 1-node skeleton (no boundaries
                // at all). With boundaries present every loop has an anchor, so reaching here
                // otherwise is a bug — fail loudly rather than silently mis-routing.
                assert!(
                    single_node,
                    "all-interior {:?}-loop on a multi-node skeleton (should be impossible)",
                    loop_axis
                );
                // 1-node skeleton: route between pinned face points (the only anchorless case).
                let control_points: Vec<EdgeID> = events
                    .iter()
                    .map(|e| match *e {
                        Event::Interior { patch, slot } => face_points[&patch][&slot],
                        Event::Boundary { edge } => edge,
                    })
                    .collect();
                let n = control_points.len();
                let mut seg0_first: Option<EdgeID> = None;
                for i in 0..n {
                    let src = control_points[i];
                    let tgt = control_points[(i + 1) % n];
                    let forced_first = if i == 0 {
                        None
                    } else {
                        let prev_last = *loop_edges.last().expect("previous segment pushed edges");
                        quad_diagonal_partner(mesh.twin(prev_last), src, mesh)
                    };
                    let forced_last = if i == n - 1 {
                        seg0_first
                            .and_then(|f| quad_diagonal_partner(f, tgt, mesh))
                            .map(|partner| mesh.twin(partner))
                    } else {
                        None
                    };
                    loop_edges.push(src);
                    used_in_loop.insert(src);
                    match surface_path_layered(src, tgt, forced_first, forced_last, &[], &blocked, &used_in_loop, &all_control_points, loop_axis, winding_sign, mesh) {
                        Some(inter) => {
                            if i == 0 {
                                seg0_first = inter.first().copied();
                            }
                            for &e in &inter {
                                used_in_loop.insert(e);
                                used_in_loop.insert(mesh.twin(e));
                            }
                            loop_edges.extend(inter);
                        }
                        None => {
                            error!("No surface path from {:?} to {:?} for {:?}-loop (all-interior {}/{})", src, tgt, loop_axis, i + 1, n);
                            path_ok = false;
                            diagnostics.failed_segments.push((src, tgt));
                        }
                    }
                }
            } else {
                let nb = anchor_idx.len();
                let ne = events.len();
                let mut seg0_first: Option<EdgeID> = None;
                for bi in 0..nb {
                    let start_pos = anchor_idx[bi];
                    let end_pos = anchor_idx[(bi + 1) % nb];
                    let src = events[start_pos].boundary_edge().expect("anchor is a boundary");
                    let tgt = events[end_pos].boundary_edge().expect("anchor is a boundary");

                    // Ordered, already-committed partners strictly between the two anchors (cyclic).
                    let mut partners: Vec<HashSet<EdgeID>> = Vec::new();
                    let mut j = (start_pos + 1) % ne;
                    while j != end_pos {
                        if let Event::Interior { patch, slot, .. } = events[j] {
                            if let Some(&p) = interior_partner.get(&(patch, slot)) {
                                partners.push(map[p].edges.iter().copied().collect());
                            }
                        }
                        j = (j + 1) % ne;
                    }

                    // Boundary-crossing 4-arm at each anchor (unchanged): depart `src` on the
                    // diagonal of how the previous chord arrived; the closing chord arrives matching
                    // the first chord's departure.
                    let forced_first = if bi == 0 {
                        None
                    } else {
                        let prev_last = *loop_edges.last().expect("previous chord pushed edges");
                        quad_diagonal_partner(mesh.twin(prev_last), src, mesh)
                    };
                    let forced_last = if bi == nb - 1 {
                        seg0_first
                            .and_then(|f| quad_diagonal_partner(f, tgt, mesh))
                            .map(|partner| mesh.twin(partner))
                    } else {
                        None
                    };

                    loop_edges.push(src);
                    used_in_loop.insert(src);
                    match surface_path_layered(src, tgt, forced_first, forced_last, &partners, &blocked, &used_in_loop, &all_control_points, loop_axis, winding_sign, mesh) {
                        Some(inter) => {
                            if bi == 0 {
                                seg0_first = inter.first().copied();
                            }
                            for &e in &inter {
                                used_in_loop.insert(e);
                                used_in_loop.insert(mesh.twin(e));
                            }
                            loop_edges.extend(inter);
                        }
                        None => {
                            error!("No surface path from {:?} to {:?} for {:?}-loop (chord {}/{}, k={})", src, tgt, loop_axis, bi + 1, nb, partners.len());
                            path_ok = false;
                            diagnostics.failed_segments.push((src, tgt));
                        }
                    }
                }
            }

            if path_ok {
                block_loop_occupancy(&loop_edges, &all_control_points, &mut blocked, mesh);
                block_adjacent_to_control_points(&loop_edges, &all_control_points, &mut blocked, mesh);
                let loop_id = map.insert(Loop { edges: loop_edges, direction: loop_axis });
                // Register this loop's interior crossings so later perpendicular partners that must
                // cross here resolve to this committed loop.
                for ev in &events {
                    if let Event::Interior { patch, slot, .. } = *ev {
                        interior_partner.insert((patch, slot), loop_id);
                    }
                }
            } else {
                // Dropped loop: record how far it got so the GUI can show where it broke.
                diagnostics.dropped_loops.push((loop_axis, loop_edges));
            }
        }
    }

    // INVARIANT CHECK: with natural interior crossings, two PERPENDICULAR loops legitimately share a
    // single edge wherever they cross — that is no longer an error. The router must still never
    // produce: (a) an edge shared by >=3 loops, or (b) an edge shared by two SAME-axis loops (same-
    // axis loops must never cross). Both are real structural bugs; the 4-arm validity of legitimate
    // crossings is then checked by the dual. Cheap, permanent safety net.
    {
        let mut occ: HashMap<EdgeID, HashSet<LoopID>> = HashMap::new();
        for (lid, l) in map.iter() {
            for &e in &l.edges {
                occ.entry(e).or_default().insert(lid);
                occ.entry(mesh.twin(e)).or_default().insert(lid);
            }
        }
        let mut visited: HashSet<EdgeID> = HashSet::new();
        let mut three_plus = 0usize;
        let mut same_axis = 0usize;
        for (&e, lids) in &occ {
            if !visited.insert(e) {
                continue;
            }
            visited.insert(mesh.twin(e));
            if lids.len() >= 3 {
                three_plus += 1;
                warn!("AUDIT >=3: geo-edge {:?} used by loops {:?}", e, lids);
            } else if lids.len() == 2 {
                let dirs: Vec<PrincipalDirection> =
                    lids.iter().map(|&l| map[l].direction).collect();
                if dirs[0] == dirs[1] {
                    same_axis += 1;
                }
            }
        }
        if three_plus > 0 || same_axis > 0 {
            warn!(
                "AUDIT: {} geo-edges with >=3 loops, {} geo-edges shared by two SAME-axis loops",
                three_plus, same_axis
            );
        }
    }
}

/// A point on the surface that lies on a loop, produced by the `next_point` traversal.
///
/// - `Crossing`: a patch-boundary crossing (a pinned anchor). `dir_sign = (A, s)` is the CrossingMap
///   key, where `A = third(loop_axis, boundary_dir)` and `s` is the sign of the slot the loop was at
///   *before* crossing this boundary. An L-loop only visits crossings whose `dir_sign` direction ≠ L.
/// - `FacePoint`: an interior crossing on a node patch, identified by `(patch, dir_sign)`. It maps to
///   an `Event::Interior` (no pinned edge); the perpendicular partner and location are resolved at
///   routing time. (Only on a 1-node skeleton, where there are no boundaries, does it resolve to a
///   pinned `FacePointMap[patch][dir_sign]` edge.) An L-loop only visits these whose direction ≠ L.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum NextPoint {
    Crossing { loop_id: LoopID, dir_sign: (PrincipalDirection, AxisSign) },
    FacePoint { patch: NodeIndex, dir_sign: (PrincipalDirection, AxisSign) },
}

/// A crossing event in a loop's cyclic order, produced by the `next_point` traversal.
///
/// - `Boundary`: a pinned patch-boundary crossing (anchor). Its `edge` is fixed up-front by
///   `get_boundaries_and_crossing_points` / `repair_boundary_crossings`.
/// - `Interior`: a crossing inside a patch with the perpendicular partner loop of the slot,
///   identified topologically by `(patch, slot)`. It carries NO pinned edge — the partner loop and
///   the mesh location are resolved at routing time (layered routing). Interior crossings exist for
///   boundary-anchored loops; a 1-node skeleton (no boundaries) is the only case where a whole loop
///   is interior, and that case alone falls back to pinned face points.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Event {
    Boundary { edge: EdgeID },
    Interior { patch: NodeIndex, slot: (PrincipalDirection, AxisSign) },
}

impl Event {
    /// The pinned anchor edge, for `Boundary` events only.
    fn boundary_edge(&self) -> Option<EdgeID> {
        match *self {
            Event::Boundary { edge } => Some(edge),
            Event::Interior { .. } => None,
        }
    }
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
