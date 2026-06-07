//! Loop planner: walks the cube/dual structure to decide each loop's ordered crossing events, then
//! drives the router between consecutive boundary anchors and commits the resulting loops.

use std::collections::{HashMap, HashSet};

use bimap::BiHashMap;
use log::{error, warn};
use mehsh::prelude::{Mesh, Vector3D};
use petgraph::{
    graph::{EdgeIndex, NodeIndex},
    visit::EdgeRef,
};
use slotmap::SlotMap;

use crate::{
    prelude::{EdgeID, FaceID, PrincipalDirection, INPUT},
    skeleton::{
        geometry::edge_midpoint_pos,
        orthogonalize::{AxisSign, LabeledCurveSkeleton, LabeledSkeletonSignExt},
    },
    solutions::{Loop, LoopID},
};

use super::audit;
use super::axes::{third, ALL_DIRS};
use super::diagnostics::{BlockedFailure, RoutingDiagnostics};
use super::router::{
    block_adjacent_to_control_points, block_loop_occupancy, patch_region, quad_diagonal_partner,
    segment_patch, surface_path_layered, RouteRequest,
};
use super::CrossingMap;

/// Read-only inputs to the planner
pub(super) struct LoopPlan<'a> {
    pub boundary_map: &'a BiHashMap<EdgeIndex, LoopID>,
    pub crossings: &'a CrossingMap,
    pub skeleton: &'a LabeledCurveSkeleton,
}

pub(super) fn pathing_for_loops(
    plan: LoopPlan,
    map: &mut SlotMap<LoopID, Loop>,
    diagnostics: &mut RoutingDiagnostics,
) {
    let LoopPlan {
        boundary_map,
        crossings,
        skeleton,
    } = plan;

    for &loop_axis in &ALL_DIRS {
        // Crossings visited by this loop axis: a crossing on a boundary with direction D and
        // dir_sign (A, s) is visited by loops with axis third(D, A).
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


        // Repeatedly pick any unvisited boundary crossing and trace the full loop it seeds.
        while let Some(&(loop_id, dir_sign)) = unvisited_crossings.iter().next() {
            let start = NextPoint::Crossing { loop_id, dir_sign };

            // Collect the loop's crossing events in cyclic order.
            let mut current = start;
            let mut events: Vec<Event> = Vec::new();
            loop {
                let event = match current {
                    NextPoint::Crossing { loop_id, dir_sign } => {
                        unvisited_crossings.remove(&(loop_id, dir_sign));
                        Event::Boundary {
                            edge: crossings[&loop_id][&dir_sign],
                        }
                    }
                    NextPoint::FacePoint { patch, dir_sign } => {
                        Event::Interior {
                            patch,
                            slot: dir_sign,
                        }
                    }
                };
                events.push(event);
                current = next_point(current, loop_axis, skeleton, boundary_map);
                if current == start {
                    break;
                }
            }
        }
    }

    // Post-hoc structural-invariant check (>=3-loop edges, same-axis crossings, segment-bigons).
    // Diagnostics only — emits AUDIT/DIAG warnings, never alters the committed loops.
    // audit::audit_loops(map, &committed_segments, &all_control_points, mesh);
}

/// A point on the surface that lies on a loop, produced by the `next_point` traversal.
///
/// - `Crossing`: a patch-boundary crossing (a pinned anchor). `dir_sign = (A, s)` is the CrossingMap
///   key, where `A = third(loop_axis, boundary_dir)` and `s` is the sign of the slot the loop was at
///   *before* crossing this boundary. An L-loop only visits crossings whose `dir_sign` direction ≠ L.
/// - `FacePoint`: an interior crossing on a node patch, identified by `(patch, dir_sign)`. It maps to
///   an `Event::Interior` (no pinned edge); the perpendicular partner and mesh location are resolved
///   at routing time (layered routing). An L-loop only visits these whose direction ≠ L.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum NextPoint {
    Crossing {
        loop_id: LoopID,
        dir_sign: (PrincipalDirection, AxisSign),
    },
    FacePoint {
        patch: NodeIndex,
        dir_sign: (PrincipalDirection, AxisSign),
    },
}

/// A crossing event in a loop's cyclic order, produced by the `next_point` traversal.
///
/// - `Boundary`: a pinned patch-boundary crossing (anchor). Its `edge` is fixed up-front by
///   `get_boundaries_and_crossing_points` / `repair_boundary_crossings`.
/// - `Interior`: a crossing inside a patch with the perpendicular partner loop of the slot,
///   identified topologically by `(patch, slot)`. It carries NO pinned edge — the partner loop and
///   the mesh location are resolved at routing time (layered routing).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Event {
    Boundary {
        edge: EdgeID,
    },
    Interior {
        patch: NodeIndex,
        slot: (PrincipalDirection, AxisSign),
    },
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
    boundary_map: &BiHashMap<EdgeIndex, LoopID>,
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
            let &loop_id = boundary_map
                .get_by_left(&edge_idx)
                .expect("edge must have a loop");
            NextPoint::Crossing {
                loop_id,
                dir_sign: slot,
            }
        }
        None => {
            // Rotate within the same patch.
            NextPoint::FacePoint {
                patch: node,
                dir_sign: next_slot,
            }
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
    boundary_map: &BiHashMap<EdgeIndex, LoopID>,
) -> NextPoint {
    match current {
        NextPoint::FacePoint { patch, dir_sign } => {
            next_from_node_slot(patch, dir_sign, loop_axis, skeleton, boundary_map)
        }

        NextPoint::Crossing {
            loop_id,
            dir_sign: (a, s),
        } => {
            // The boundary direction D = third(loop_axis, a).
            // ccw_next tells us which direction+sign we move through the boundary.
            let (move_dir, move_sign) = ccw_next(loop_axis, a, s);
            // move_dir == third(loop_axis, a) == the boundary's direction.

            // Find the boundary edge and determine which endpoint node we enter.
            let &edge_idx = boundary_map
                .get_by_right(&loop_id)
                .expect("loop must have an edge");
            let (src, tgt) = skeleton.edge_endpoints(edge_idx).expect("edge must exist");

            // The edge is stored with a sign from src->tgt. Compare to move_sign to pick the node.
            let edge_sign_from_src = skeleton
                .edge_sign_from(edge_idx, src)
                .expect("src is an endpoint");
            let entered_node = if move_sign == edge_sign_from_src {
                tgt // moving in the same direction as src->tgt means we enter tgt
            } else {
                src
            };

            // We entered through the face opposite to move_sign.
            let incoming_slot = (move_dir, move_sign.flipped());
            next_from_node_slot(
                entered_node,
                incoming_slot,
                loop_axis,
                skeleton,
                boundary_map,
            )
        }
    }
}
