//! Surface-embedded dual-loop generation from a labeled curve skeleton.
//!
//! The pipeline runs in stages, one per submodule:
//! - [`crossings`] places the boundary crossings (where each pair of orthogonal loops crosses every
//!   patch-patch boundary) and repairs them onto straight, threadable edges.
//! - [`planner`] walks the cube/dual structure to decide each loop's ordered crossing events.
//! - [`router`] connects consecutive crossings with a layered, alignment-primary Dijkstra.
//! - [`geom`] / [`diagnostics`] hold shared helpers and the GUI failure overlay data.
//!
//! [`generate_loops`] is the orchestrator that wires these together.

mod audit;
mod axes;
mod crossings;
mod diagnostics;
mod planner;
mod router;

use std::collections::{HashMap, HashSet};

use mehsh::prelude::{Mesh, Vector3D};
use slotmap::SlotMap;

use crate::{
    prelude::{EdgeID, PrincipalDirection, INPUT},
    skeleton::{
        orthogonalize::{AxisSign, LabeledCurveSkeleton},
        SkeletonData,
    },
    solutions::{Loop, LoopID},
};

use crate::skeleton::geometry::edge_midpoint_pos;
use crossings::{get_boundaries_and_crossing_points, repair_boundary_crossings};
use planner::{pathing_for_loops, LoopPlan};

pub use diagnostics::{BlockedFailure, RoutingDiagnostics};

/// Per boundary loop, the crossing points for each orthogonal (direction, sign).
pub type CrossingMap = HashMap<LoopID, HashMap<(PrincipalDirection, AxisSign), EdgeID>>;

pub enum LoopGenerationError {
    MissingLabeledSkeleton,
    // TODO other error variants
}

/// Generates surface-embedded loops from a labeled skeleton and surface mesh.
///
/// Assumes the skeleton has at least one edge (so every loop anchors on a patch boundary). The
/// degenerate single-node case — an all-interior loop set with nothing to anchor on — is handled by
/// the normal-initialization pipeline, not here, so it produces no loops.
pub fn generate_loops(
    skeleton_data: &SkeletonData,
    mesh: &Mesh<INPUT>,
) -> Result<(SlotMap<LoopID, Loop>, CrossingMap, RoutingDiagnostics), LoopGenerationError> {
    let mut map: SlotMap<LoopID, Loop> = SlotMap::with_key();
    let mut diagnostics = RoutingDiagnostics::default();

    let skeleton: &LabeledCurveSkeleton = skeleton_data
        .labeled_skeleton
        .as_ref()
        .ok_or_else(|| LoopGenerationError::MissingLabeledSkeleton)?;
    let (boundary_map, mut crossings) = get_boundaries_and_crossing_points(skeleton, mesh, &mut map);

    // Slide each boundary crossing onto a straight, threadable boundary edge so the boundary
    // loop crosses diagonally (leaving the complementary diagonal free for the orthogonal loop)
    // and the orthogonal loop can actually pass through without being boxed in. Topological.
    repair_boundary_crossings(&map, &mut crossings, mesh);

    // Trace paths between boundary crossings to create the loops. The planner only reads these maps,
    // so they are borrowed (no clone) — `crossings` is still returned to the caller below.
    pathing_for_loops(
        LoopPlan {
            boundary_map: &boundary_map,
            crossings: &crossings,
            skeleton,
            mesh,
        },
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

    Ok((map, crossings, diagnostics))
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
