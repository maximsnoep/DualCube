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

/// Read-only inputs to the planner: the boundary↔loop mapping, the placed boundary crossings, and
/// the skeleton + mesh. Bundled so the planner takes one borrowed plan rather than four separate
/// arguments — and so the caller need not clone the maps.
pub(super) struct LoopPlan<'a> {
    pub boundary_map: &'a BiHashMap<EdgeIndex, LoopID>,
    pub crossings: &'a CrossingMap,
    pub skeleton: &'a LabeledCurveSkeleton,
    pub mesh: &'a Mesh<INPUT>,
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
        mesh,
    } = plan;
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
    //
    // The value is `(partner loop, partner SEGMENT index)` — the chord (0-based, in commit order)
    // of the partner whose body owns this crossing. The router crosses *that specific segment*, not
    // the whole partner loop: a single chord may legitimately cross the same partner loop twice when
    // the two crossings are on DIFFERENT partner segments (the cube model visits a patch in two
    // separate chords). Using the whole loop as the partner collapses both crossings onto whichever
    // segment's body the router happens to reach — a spurious bigon. Per-segment partners keep the
    // two crossings on the two distinct segments, so they are non-adjacent in each loop's cyclic
    // order and the dual's 4-arm/quad-walk is satisfied.
    let mut interior_partner: HashMap<(NodeIndex, (PrincipalDirection, AxisSign)), (LoopID, usize)> =
        HashMap::new();
    // Per committed loop, the ORDERED edges of each of its chord-segments (in commit order), so a
    // partner lookup can hand the router exactly the owning segment's edges, and the blocker overlay
    // can draw a specific segment as a polyline.
    let mut committed_seg_ordered: HashMap<LoopID, Vec<Vec<EdgeID>>> = HashMap::new();

    // DIAG: committed loops' chord-segments (each chord = edges between two consecutive boundary
    // anchors). Invariant (ALWAYS): any two segments cross at most once. Checked at the end.
    let mut committed_segments: Vec<(LoopID, PrincipalDirection, Vec<Vec<EdgeID>>)> = Vec::new();

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

        // Repeatedly pick any unvisited boundary crossing and trace the full loop it seeds. Every
        // loop on a boundary-bearing skeleton is reached from a crossing.
        while let Some(&(loop_id, dir_sign)) = unvisited_crossings.iter().next() {
            let start = NextPoint::Crossing { loop_id, dir_sign };

            // First pass: collect the loop's crossing events in cyclic order.
            let mut current = start;
            let mut events: Vec<Event> = Vec::new();
            let mut event_bnd: Vec<Option<LoopID>> = Vec::new(); // DIAG: boundary id per anchor
            loop {
                let event = match current {
                    NextPoint::Crossing { loop_id, dir_sign } => {
                        unvisited_crossings.remove(&(loop_id, dir_sign));
                        event_bnd.push(Some(loop_id));
                        Event::Boundary { edge: crossings[&loop_id][&dir_sign] }
                    }
                    NextPoint::FacePoint { patch, dir_sign } => {
                        event_bnd.push(None);
                        Event::Interior { patch, slot: dir_sign }
                    }
                };
                events.push(event);
                current = next_point(current, loop_axis, skeleton, boundary_map);
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
            let mut loop_chords: Vec<Vec<EdgeID>> = Vec::new(); // DIAG: per-chord segment edges
            let mut used_in_loop: HashSet<EdgeID> = HashSet::new();
            let mut path_ok = true;

            // Cyclic indices of the boundary anchors within `events`.
            let anchor_idx: Vec<usize> = events
                .iter()
                .enumerate()
                .filter(|(_, e)| matches!(e, Event::Boundary { .. }))
                .map(|(i, _)| i)
                .collect();

            // Every loop on a boundary-bearing skeleton has at least one boundary anchor; the
            // anchorless single-node case is handled by the normal-initialization pipeline, not here.
            debug_assert!(
                !anchor_idx.is_empty(),
                "anchorless {loop_axis:?}-loop reached the skeleton router"
            );
            {
                let nb = anchor_idx.len();
                let ne = events.len();
                let mut seg0_first: Option<EdgeID> = None;
                for bi in 0..nb {
                    let start_pos = anchor_idx[bi];
                    let end_pos = anchor_idx[(bi + 1) % nb];
                    let src = events[start_pos].boundary_edge().expect("anchor is a boundary");
                    let tgt = events[end_pos].boundary_edge().expect("anchor is a boundary");

                    // The first interior-event patch between the two anchors. For a U-turn/cap this is
                    // the wrap patch; it also identifies the segment's patch when the two boundary
                    // edges coincide (so `segment_patch`'s two-edge intersection is undefined).
                    let wrap_patch: Option<NodeIndex> = {
                        let mut jj = (start_pos + 1) % ne;
                        let mut wp = None;
                        while jj != end_pos {
                            if let Event::Interior { patch, .. } = events[jj] {
                                wp = Some(patch);
                                break;
                            }
                            jj = (jj + 1) % ne;
                        }
                        wp
                    };

                    audit::diag_same_boundary_chord(
                        loop_axis,
                        bi,
                        nb,
                        event_bnd[start_pos],
                        event_bnd[end_pos],
                        wrap_patch,
                        skeleton,
                    );

                    // Ordered, already-committed partners strictly between the two anchors (cyclic).
                    let mut partners: Vec<HashSet<EdgeID>> = Vec::new();
                    let mut partner_ids: Vec<(LoopID, NodeIndex, (PrincipalDirection, AxisSign))> =
                        Vec::new(); // DIAG
                    let mut j = (start_pos + 1) % ne;
                    while j != end_pos {
                        if let Event::Interior { patch, slot, .. } = events[j] {
                            if let Some(&(p, pseg)) = interior_partner.get(&(patch, slot)) {
                                // Cross the SPECIFIC partner segment that owns this crossing, not the
                                // whole partner loop (prevents the spurious same-segment bigon).
                                let edges: HashSet<EdgeID> = committed_seg_ordered
                                    .get(&p)
                                    .and_then(|segs| segs.get(pseg))
                                    .map(|v| v.iter().copied().collect())
                                    .unwrap_or_else(|| map[p].edges.iter().copied().collect());
                                partners.push(edges);
                                partner_ids.push((p, patch, slot)); // DIAG
                            }
                        }
                        j = (j + 1) % ne;
                    }
                    audit::diag_double_partner(loop_axis, bi, nb, &partner_ids, skeleton);

                    // Patch-locality: confine this segment's route to its own patch region so it
                    // cannot wander off-patch (which manufactures geometric bigons and false
                    // contention for other loops). The vertex-touch region already includes the
                    // boundary band + a 1-ring margin, so src/tgt and the partner-crossing quads stay
                    // reachable. We also explicitly add src/tgt faces, and the crossing quads of any
                    // partner edge adjacent to the region (so a one-way crossing is never barred),
                    // without pulling in whole partner loops. Falls back to global (None) only if the
                    // patch can't be identified — which should not happen on a multi-node skeleton.
                    let region: Option<HashSet<FaceID>> = match segment_patch(
                        event_bnd[start_pos],
                        event_bnd[end_pos],
                        wrap_patch,
                        boundary_map,
                        skeleton,
                    ) {
                        Some(p) => {
                            let mut r = patch_region(skeleton, p, mesh);
                            for &e in &[src, tgt] {
                                r.insert(mesh.face(e));
                                r.insert(mesh.face(mesh.twin(e)));
                            }
                            for pe in &partners {
                                for &e in pe {
                                    let fa = mesh.face(e);
                                    let fb = mesh.face(mesh.twin(e));
                                    if r.contains(&fa) || r.contains(&fb) {
                                        r.insert(fa);
                                        r.insert(fb);
                                    }
                                }
                            }
                            Some(r)
                        }
                        None => {
                            warn!(
                                "patch-locality: could not identify patch for {:?}-loop chord {}/{}; \
                                 routing globally",
                                loop_axis, bi + 1, nb
                            );
                            None
                        }
                    };

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
                    // Route the chord, self-avoiding WITHIN the chord too. The layered Dijkstra has
                    // state `(face, layer)`, so it can revisit a face at a higher layer and fold back
                    // over its own body (a self-crossing — invalid, loops must be simple). `used_in_loop`
                    // only blocks the loop's PRIOR chords. So if the returned path reuses a geometric
                    // edge, we block those edges and re-route; only if no simple path exists do we drop.
                    let mut try_used = used_in_loop.clone();
                    let mut routed: Option<Vec<EdgeID>> = None;
                    for _attempt in 0..4 {
                        let Some(inter) = surface_path_layered(
                            RouteRequest {
                                source: src,
                                target: tgt,
                                forced_first,
                                forced_last,
                                partners: &partners,
                                blocked: &blocked,
                                used: &try_used,
                                control_points: &all_control_points,
                                allowed_faces: region.as_ref(),
                                loop_axis,
                                winding_sign,
                            },
                            mesh,
                        ) else {
                            break;
                        };
                        // Detect a geometric edge used twice within this chord ([src] + inter).
                        let mut seen: HashSet<EdgeID> = HashSet::new();
                        seen.insert(src);
                        seen.insert(mesh.twin(src));
                        let mut folded: Vec<EdgeID> = Vec::new();
                        for &e in &inter {
                            if seen.contains(&e) || seen.contains(&mesh.twin(e)) {
                                folded.push(e);
                            }
                            seen.insert(e);
                            seen.insert(mesh.twin(e));
                        }
                        if folded.is_empty() {
                            routed = Some(inter);
                            break;
                        }
                        // Block the folded edges (both halves) and re-route a simple path.
                        for &e in &folded {
                            try_used.insert(e);
                            try_used.insert(mesh.twin(e));
                        }
                    }
                    match routed {
                        Some(inter) => {
                            if bi == 0 {
                                seg0_first = inter.first().copied();
                            }
                            let mut seg = vec![src]; // DIAG: this chord's edges (src + interior)
                            for &e in &inter {
                                used_in_loop.insert(e);
                                used_in_loop.insert(mesh.twin(e));
                                seg.push(e);
                            }
                            loop_chords.push(seg); // DIAG
                            loop_edges.extend(inter);
                        }
                        None => {
                            error!("No surface path from {:?} to {:?} for {:?}-loop (chord {}/{}, k={})", src, tgt, loop_axis, bi + 1, nb, partners.len());
                            diagnostics.blocked_failures.push(BlockedFailure {
                                src,
                                tgt,
                                axis: loop_axis,
                            });
                            path_ok = false;
                            diagnostics.failed_segments.push((src, tgt));
                        }
                    }
                }
            }

            if path_ok {
                audit::diag_self_cross(loop_axis, &loop_edges, &loop_chords, mesh);
                block_loop_occupancy(&loop_edges, &all_control_points, &mut blocked, mesh);
                block_adjacent_to_control_points(&loop_edges, &all_control_points, &mut blocked, mesh);
                let loop_id = map.insert(Loop { edges: loop_edges, direction: loop_axis });
                // Per-segment ordered edges (commit order = chord `bi` order), so a later partner can
                // cross exactly the owning segment and the overlay can draw a blocking segment.
                committed_seg_ordered.insert(loop_id, loop_chords.clone());
                committed_segments.push((loop_id, loop_axis, loop_chords)); // DIAG
                // Register this loop's interior crossings, tagged with the segment (chord) index they
                // lie on, so a later perpendicular partner resolves to the SPECIFIC owning segment.
                // The segment index = number of boundary anchors passed since the first anchor, which
                // matches the chord order (`bi`) of the second pass and `committed_seg_edges`.
                let nev = events.len();
                let first_anchor = anchor_idx[0];
                let mut seg: usize = 0;
                for off in 0..nev {
                    let pos = (first_anchor + off) % nev;
                    match events[pos] {
                        Event::Boundary { .. } => {
                            if off != 0 {
                                seg += 1;
                            }
                        }
                        Event::Interior { patch, slot } => {
                            interior_partner.insert((patch, slot), (loop_id, seg));
                        }
                    }
                }
            } else {
                // Dropped loop: record how far it got so the GUI can show where it broke.
                diagnostics.dropped_loops.push((loop_axis, loop_edges));
            }
        }
    }

    // Post-hoc structural-invariant check (>=3-loop edges, same-axis crossings, segment-bigons).
    // Diagnostics only — emits AUDIT/DIAG warnings, never alters the committed loops.
    audit::audit_loops(map, &committed_segments, &all_control_points, mesh);
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
    Crossing { loop_id: LoopID, dir_sign: (PrincipalDirection, AxisSign) },
    FacePoint { patch: NodeIndex, dir_sign: (PrincipalDirection, AxisSign) },
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
    boundary_map: &BiHashMap<EdgeIndex, LoopID>,
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
