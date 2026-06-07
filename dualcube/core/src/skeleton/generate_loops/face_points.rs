//! Interior "face point" computation: pinned interior crossings for the single-node skeleton case,
//! whose loops are all-interior and have no boundary to anchor on.

use std::collections::{HashMap, HashSet};

use log::warn;
use mehsh::prelude::{HasFaces, HasPosition, HasVertices, Mesh, Vector3D};
use petgraph::{
    graph::NodeIndex,
    visit::{EdgeRef, IntoNodeReferences},
};

use crate::{
    prelude::{EdgeID, PrincipalDirection, VertID, INPUT},
    skeleton::orthogonalize::{AxisSign, LabeledCurveSkeleton},
};

use super::geom::{edge_midpoint_pos, ALL_DIRS, ALL_SIGNS};
use super::FacePointMap;

/// Weight of the directional (alignment) force relative to the centrality (anti-rim) force when
/// placing face points; both are normalized to ~unit scale, so this is dimensionless. It must be
/// large enough that alignment spreads a node's slots into distinct sectors (no center-collapse)
/// yet small enough that centrality still pulls each placement off the patch rim. See
/// [`best_face_point_candidate`].
const FP_DIRECTION_BIAS: f64 = 1.5;

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

/// Midpoint positions of the patch border around a skeleton node: the edge midpoints of every
/// incident skeleton edge's boundary loop. Used to score face-point placement by distance from the
/// border (see [`best_face_point_candidate`]).
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
