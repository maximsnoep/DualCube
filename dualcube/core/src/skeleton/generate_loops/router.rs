//! Surface router: the layered Dijkstra that connects two surface crossings, plus the topological
//! helpers it relies on (diagonal-partner crossing rule, patch-locality region, occupancy blocking).

use std::collections::{BinaryHeap, HashMap, HashSet};

use bimap::BiHashMap;
use ordered_float::OrderedFloat;
use mehsh::prelude::{HasEdges, HasFaces, HasNormal, HasPosition, HasVertices, Mesh, Vector3D};
use petgraph::graph::{EdgeIndex, NodeIndex};

use crate::{
    prelude::{EdgeID, FaceID, PrincipalDirection, VertID, INPUT},
    skeleton::{geometry::edge_midpoint_pos, orthogonalize::LabeledCurveSkeleton},
    solutions::LoopID,
};

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

/// All mesh faces incident to any vertex of `patch` (the "vertex-touch" region). Because
/// `patch_vertices` is a complete vertex partition, this set includes the patch interior, the full
/// boundary band (both faces of every boundary-crossing edge of the patch, since each such edge has
/// an endpoint vertex in the patch), and a one-face margin spilling into neighbouring patches. That
/// is exactly the region a single segment legitimately needs: enough to reach its boundary anchors
/// and its partner-crossing quads, while forbidding it from wandering off-patch.
pub(super) fn patch_region(
    skeleton: &LabeledCurveSkeleton,
    patch: NodeIndex,
    mesh: &Mesh<INPUT>,
) -> HashSet<FaceID> {
    let mut region = HashSet::new();
    for &v in &skeleton[patch].skeleton_node.patch_vertices {
        for f in mesh.faces(v) {
            region.insert(f);
        }
    }
    region
}

/// The single skeleton patch a segment lives in. A through-segment's two boundary anchors lie on two
/// distinct skeleton edges that share exactly one endpoint node — that node is the patch. A U-turn /
/// cap segment has both anchors on the SAME boundary loop (`bs == be`), so the two-edge intersection
/// is undefined; we fall back to the patch of the segment's interior events (`wrap_patch`). A
/// through-segment with zero interior events is still resolved by the two-edge intersection.
pub(super) fn segment_patch(
    bs: Option<LoopID>,
    be: Option<LoopID>,
    wrap_patch: Option<NodeIndex>,
    boundary_map: &BiHashMap<EdgeIndex, LoopID>,
    skeleton: &LabeledCurveSkeleton,
) -> Option<NodeIndex> {
    if let (Some(bs), Some(be)) = (bs, be) {
        if bs != be {
            if let (Some(&e1), Some(&e2)) =
                (boundary_map.get_by_right(&bs), boundary_map.get_by_right(&be))
            {
                if let (Some((a1, b1)), Some((a2, b2))) =
                    (skeleton.edge_endpoints(e1), skeleton.edge_endpoints(e2))
                {
                    let shared: Vec<NodeIndex> =
                        [a1, b1].into_iter().filter(|&n| n == a2 || n == b2).collect();
                    if shared.len() == 1 {
                        return Some(shared[0]);
                    }
                }
            }
        }
    }
    wrap_patch
}

/// Everything `surface_path_layered` needs to connect one pair of crossings, minus the mesh. Bundled
/// so the router has a single, readable parameter instead of a dozen positional arguments.
pub(super) struct RouteRequest<'a> {
    /// Anchor the path starts from (the source crossing edge).
    pub source: EdgeID,
    /// Anchor the path must reach (the target crossing edge).
    pub target: EdgeID,
    /// If set, pins the first step out of `source` (the boundary-crossing diagonal).
    pub forced_first: Option<EdgeID>,
    /// If set, pins the last step into `target` (the boundary-crossing diagonal).
    pub forced_last: Option<EdgeID>,
    /// Ordered committed partner segments to cross exactly once each, in order.
    pub partners: &'a [HashSet<EdgeID>],
    /// Committed loops' edge occupancy (edges no new loop may use).
    pub blocked: &'a HashSet<EdgeID>,
    /// The current loop's own already-routed edges (within-loop self-avoidance).
    pub used: &'a HashSet<EdgeID>,
    /// Boundary-crossing edges the body must not thread (except the forced first/last steps).
    pub control_points: &'a HashSet<EdgeID>,
    /// Patch-locality region: `Some` confines the search to these faces; `None` searches globally.
    pub allowed_faces: Option<&'a HashSet<FaceID>>,
    /// Axis of the loop being routed (the alignment target direction).
    pub loop_axis: PrincipalDirection,
    /// Winding sign about `loop_axis`; signs the alignment target so reversal is penalized.
    pub winding_sign: f64,
}

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
/// `allowed_faces`, when `Some`, restricts the search to that set of mesh faces (patch-locality):
/// the segment may only traverse faces in the set, so it cannot wander off its own patch. `None`
/// leaves the search global (the prior behaviour).
///
/// Returns the intermediate mesh edges (source/target exclusive), or `None` if no such path exists.
pub(super) fn surface_path_layered(req: RouteRequest, mesh: &Mesh<INPUT>) -> Option<Vec<EdgeID>> {
    let RouteRequest {
        source,
        target,
        forced_first,
        forced_last,
        partners,
        blocked,
        used,
        control_points,
        allowed_faces,
        loop_axis,
        winding_sign,
    } = req;
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
        if allowed_faces.is_some_and(|s| !s.contains(&sf)) {
            continue; // patch-locality: don't seed outside the segment's patch region
        }
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
            // Patch-locality: never step (or cross a partner) into a face outside the region. This
            // covers both the straight step below and the partner-crossing intermediate face.
            if allowed_faces.is_some_and(|s| !s.contains(&next_face)) {
                continue;
            }

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
                if allowed_faces.is_some_and(|s| !s.contains(&land)) {
                    continue; // patch-locality: partner crossing must land inside the region
                }
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
pub(super) fn block_adjacent_to_control_points(
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
pub(super) fn block_loop_occupancy(
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
pub(super) fn quad_diagonal_partner(side: EdgeID, cp: EdgeID, mesh: &Mesh<INPUT>) -> Option<EdgeID> {
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
