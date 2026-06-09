//! Surface router: per patch-segment, one layered Dijkstra threading that segment's interior
//! crossings in order and closing on its exit boundary anchor, plus the topological helpers it
//! relies on (diagonal-partner crossing rule, patch-locality region, occupancy blocking).
//!
//! A loop is split at its boundary anchors into patch-segments and routed ONE patch at a time:
//! within a segment the layered search cannot self-cross (every layer is a loop-crossing, and by the
//! Jordan-curve property a path cannot return across a loop it has already crossed), and blocking
//! every committed edge keeps successive segments (and later loops) disjoint — so the assembled loop
//! is simple by construction. (The previous design routed the whole loop as one global graph, which
//! had no such guarantee.) Each segment is a SMALL layered graph: every interior crossing it makes is one
//! ordered layer transition realized atomically by [`quad_diagonal_partner`] (the 4 distinct arms
//! come out by construction, as the partner's own arms are pre-blocked), and the exit anchor is the
//! final closing layer. Boundary loops are permanent anchors: we never alter them, we only add the
//! routed loops that make the boundaries work as a dual.

use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use bimap::BiHashMap;
use log::{error, info};
use mehsh::prelude::{HasEdges, HasFaces, HasNormal, HasPosition, HasVertices, Mesh, Vector3D};
use ordered_float::OrderedFloat;
use petgraph::graph::{EdgeIndex, NodeIndex};
use slotmap::SlotMap;

use crate::{
    prelude::{EdgeID, FaceID, PrincipalDirection, VertID, INPUT},
    skeleton::{
        generate_loops::{
            planner::{entered_patch_after_crossing, Event, SegmentPlan},
            BlockedFailure, CrossingMap, RoutingDiagnostics,
        },
        geometry::edge_midpoint_pos,
        orthogonalize::LabeledCurveSkeleton,
    },
    solutions::{Loop, LoopID},
};

use super::axes::third;

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

/// The router connects the crossings to create a complete polycube loop structure, using the plan
/// from the planner. Boundary loops are the structural anchors (already in `map`, never altered);
/// the loops we route here are added as NEW entries — the "plumbing" that makes the placed
/// boundaries get accepted as a dual.
pub fn route_segments(
    map: &mut SlotMap<LoopID, Loop>,
    segment_plan: &SegmentPlan,
    crossings: &CrossingMap,
    boundary_map: &BiHashMap<EdgeIndex, LoopID>,
    skeleton: &LabeledCurveSkeleton,
    mesh: &Mesh<INPUT>,
    diagnostics: &mut RoutingDiagnostics,
) {
    // Every boundary-crossing edge across all boundary loops. Their forward half stays crossable (so
    // the designated crossing can happen there); the body of any loop must not thread them otherwise.
    let all_crossing_edges: HashSet<EdgeID> = crossings
        .values()
        .flat_map(|m| m.values().copied())
        .collect();

    // Occupancy seed: the boundary loops are permanent and always block (except their own crossing
    // edges' forward half). Routed loops add their occupancy as they commit.
    let mut blocked: HashSet<EdgeID> = HashSet::new();
    for l in map.values() {
        block_loop_occupancy(&l.edges, &all_crossing_edges, &mut blocked, mesh);
    }

    // Already-routed loops, keyed by the planner id, so an interior crossing can hand the layered
    // graph its committed partner's edges. A partner not yet here is skipped (it adjusts to us).
    let mut routed: HashMap<LoopID, Vec<EdgeID>> = HashMap::new();
    let (mut ok, mut dropped) = (0usize, 0usize);

    for (&loop_id, events) in &segment_plan.plan {
        match route_one_loop(
            events,
            &routed,
            crossings,
            boundary_map,
            skeleton,
            &blocked,
            &all_crossing_edges,
            mesh,
        ) {
            LoopRoute::Routed(axis, edges) => {
                block_loop_occupancy(&edges, &all_crossing_edges, &mut blocked, mesh);
                routed.insert(loop_id, edges.clone());
                map.insert(Loop {
                    edges,
                    direction: axis,
                });
                ok += 1;
            }
            LoopRoute::Dropped(axis, partial, gap) => {
                dropped += 1;
                // Show how far the loop got before being abandoned, and the gap it stalled at.
                diagnostics.dropped_loops.push((axis, partial));
                if let Some((src, tgt)) = gap {
                    diagnostics.failed_segments.push((src, tgt));
                    diagnostics
                        .blocked_failures
                        .push(BlockedFailure { src, tgt, axis });
                }
            }
        }
    }
    info!(
        "ROUTING: {} loops planned, {} routed, {} dropped",
        segment_plan.plan.len(),
        ok,
        dropped
    );
}

/// One ordered crossing the loop makes: the partner edge-set whose crossing advances past it, and
/// the patch-region the loop travels in to REACH it (the band before this crossing).
struct Layer {
    /// Edges (either half) whose crossing realizes this layer's transition.
    partner: HashSet<EdgeID>,
    /// Faces the loop may traverse while travelling toward this crossing (patch locality).
    region: HashSet<FaceID>,
    /// Human-readable tag for the crossing (boundary/interior + the crossed loop's direction), for
    /// the failure diagnostic.
    label: String,
}

/// The loop's axis from its first boundary event: a boundary of direction `D` crossed at slot
/// `(A, _)` is threaded by the `third(D, A)`-loop.
fn loop_axis_of(
    events: &[Event],
    boundary_map: &BiHashMap<EdgeIndex, LoopID>,
    skeleton: &LabeledCurveSkeleton,
) -> Option<PrincipalDirection> {
    let (boundary, slot) = events.iter().find_map(|e| match *e {
        Event::Boundary { boundary, slot } => Some((boundary, slot)),
        _ => None,
    })?;
    let edge = *boundary_map.get_by_right(&boundary)?;
    let boundary_dir = skeleton.edge_weight(edge)?.direction;
    Some(third(boundary_dir, slot.0))
}

/// Of the two faces of the anchor (crossing) edge, the one on `patch`'s side: its apex vertex (the
/// vertex not on the edge) belongs to `patch`'s vertex partition. Pure topology — no positions.
fn anchor_face_on_patch_side(
    anchor: EdgeID,
    patch: NodeIndex,
    skeleton: &LabeledCurveSkeleton,
    mesh: &Mesh<INPUT>,
) -> FaceID {
    let pv: HashSet<VertID> = skeleton[patch]
        .skeleton_node
        .patch_vertices
        .iter()
        .copied()
        .collect();
    let a = mesh.root(anchor);
    let b = mesh.toor(anchor);
    for f in [mesh.face(anchor), mesh.face(mesh.twin(anchor))] {
        if let Some(apex) = mesh.vertices(f).find(|&v| v != a && v != b) {
            if pv.contains(&apex) {
                return f;
            }
        }
    }
    mesh.face(anchor)
}

/// Coarse winding sign about `loop_axis` from the polygon over the loop's event positions (boundary
/// crossing midpoints + interior patch centres), projected to the axis-perpendicular plane. Fixed
/// for the whole loop so the alignment cost references one consistent winding.
fn winding_sign_of(
    events: &[Event],
    loop_axis: PrincipalDirection,
    crossings: &CrossingMap,
    skeleton: &LabeledCurveSkeleton,
    mesh: &Mesh<INPUT>,
) -> f64 {
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
                Event::Boundary { boundary, slot } => {
                    edge_midpoint_pos(crossings[&boundary][&slot], mesh)
                }
                Event::InteriorCrossing { patch, .. } => skeleton[patch].skeleton_node.position,
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
    if area >= 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Outcome of routing one loop.
enum LoopRoute {
    /// Routed: the loop's axis and its ordered half-edge cycle.
    Routed(PrincipalDirection, Vec<EdgeID>),
    /// Dropped: the loop's axis, the partial path it managed (for the GUI overlay), and the
    /// `(stuck_edge, next_crossing)` gap where it stalled (if it got far enough to have one).
    Dropped(PrincipalDirection, Vec<EdgeID>, Option<(EdgeID, EdgeID)>),
}

/// Routes one loop patch-by-patch. A loop is a cyclic sequence of boundary anchors with optional
/// interior crossings between them; each span between two consecutive anchors lives in exactly one
/// skeleton patch. We route each such segment as its own layered Dijkstra confined to that patch
/// (see [`route_segment_layered`]) and block every committed edge before the next segment, so the
/// whole loop — and every later loop — stays edge-disjoint and therefore simple by construction.
/// This trades the old single-graph global optimum for a hard no-self-intersection guarantee.
///
/// The boundary crossing at an anchor is owned by the segment that ENDS on it: that segment records
/// the anchor edge and lands on the NEXT patch's side of it, which is exactly the next segment's
/// seed. The single free diagonal the boundary occupancy leaves at each anchor forces the two
/// segments' arms to agree across the seam without any shared state (see [`route_segment_layered`]).
#[allow(clippy::too_many_arguments)]
fn route_one_loop(
    events: &[Event],
    routed: &HashMap<LoopID, Vec<EdgeID>>,
    crossings: &CrossingMap,
    boundary_map: &BiHashMap<EdgeIndex, LoopID>,
    skeleton: &LabeledCurveSkeleton,
    blocked: &HashSet<EdgeID>,
    all_crossing_edges: &HashSet<EdgeID>,
    mesh: &Mesh<INPUT>,
) -> LoopRoute {
    let axis = loop_axis_of(events, boundary_map, skeleton).unwrap_or(PrincipalDirection::X);

    let k = events.len();
    if k == 0 {
        error!("Dropped {axis:?}-loop: empty plan");
        return LoopRoute::Dropped(axis, Vec::new(), None);
    }
    // Rotate so a boundary anchor is rot[0]: segments are the spans between consecutive anchors.
    let Some(cut) = events.iter().position(|e| matches!(e, Event::Boundary { .. })) else {
        error!("Dropped {axis:?}-loop: no boundary anchor to segment on");
        return LoopRoute::Dropped(axis, Vec::new(), None);
    };
    let rot: Vec<Event> = (0..k).map(|i| events[(cut + i) % k]).collect();
    let Event::Boundary {
        boundary: boundary0,
        slot: slot0,
    } = rot[0]
    else {
        unreachable!("rot[0] is a boundary by construction")
    };
    let loop_axis = boundary_map
        .get_by_right(&boundary0)
        .and_then(|&e| skeleton.edge_weight(e))
        .map(|w| third(w.direction, slot0.0))
        .unwrap_or(axis);
    // One consistent winding for the whole loop as the default heuristic reference; per-segment the
    // `invert` flag still re-picks the orientation, so this is only a tie-break baseline.
    let winding_sign = winding_sign_of(&rot, loop_axis, crossings, skeleton, mesh);

    // Boundary anchor positions in `rot` (rot[0] is one), and the placed anchor edge of an event.
    let bidx: Vec<usize> = (0..k)
        .filter(|&i| matches!(rot[i], Event::Boundary { .. }))
        .collect();
    let nb = bidx.len();
    let cp_of = |ev: &Event| -> Option<EdgeID> {
        match *ev {
            Event::Boundary { boundary, slot } => crossings.get(&boundary)?.get(&slot).copied(),
            _ => None,
        }
    };

    // Route each patch-segment in cyclic order, blocking its edges before the next so the loop stays
    // edge-disjoint. `loop_blocked` is a private clone: the shared occupancy is untouched until the
    // whole loop succeeds, so a dropped loop rolls back cleanly (its partial edges block nothing).
    let mut loop_blocked = blocked.clone();
    let mut loop_edges: Vec<EdgeID> = Vec::new();

    for s in 0..nb {
        let bi = bidx[s];
        let bj = bidx[(s + 1) % nb];

        // The patch this segment travels in = the node the loop ENTERS when it crosses the entry
        // anchor. (Not `band_patch`: when both anchors of a cap/leaf segment lie on the SAME boundary
        // edge — a degree-1 skeleton node — the shared-endpoint rule can't tell which side the loop
        // is on, so it picks the wrong patch and seeds the search on the wrong side of the boundary.)
        let Event::Boundary {
            boundary: b_start,
            slot: slot_start,
        } = rot[bi]
        else {
            unreachable!("bidx only indexes boundary events")
        };
        let Some(patch) =
            entered_patch_after_crossing(b_start, slot_start, loop_axis, skeleton, boundary_map)
        else {
            error!("Dropped {loop_axis:?}-loop: segment {s} entry anchor resolves to no patch");
            return LoopRoute::Dropped(loop_axis, loop_edges, None);
        };
        let (Some(cp_start), Some(cp_end)) = (cp_of(&rot[bi]), cp_of(&rot[bj])) else {
            error!("Dropped {loop_axis:?}-loop: segment {s} anchor has no placed crossing");
            return LoopRoute::Dropped(loop_axis, loop_edges, None);
        };

        let region = patch_region(skeleton, patch, mesh);
        // Seed: the in-patch (P_s) face of the entry anchor — where the previous segment's crossing
        // landed (a near-side query, reliable for the patch we actually travel in).
        let seed = anchor_face_on_patch_side(cp_start, patch, skeleton, mesh);
        // Target: where crossing the exit anchor out of this patch lands — the OTHER of cp_end's two
        // faces from its in-patch side. We flip the reliable near-side query rather than querying the
        // far patch directly: a repaired/slid anchor's far apex can be missing from that patch's
        // vertex partition, which would point `target` at the wrong face and stall the close. This
        // equals the next segment's own near-side seed query of the same anchor, so the seam matches.
        let entry_face = anchor_face_on_patch_side(cp_end, patch, skeleton, mesh);
        let (f0, f1) = (mesh.face(cp_end), mesh.face(mesh.twin(cp_end)));
        let target = if entry_face == f0 { f1 } else { f0 };

        // Layers: each committed interior crossing inside this patch (partner already routed), in
        // order, then the exit anchor as the closing crossing. Uncommitted interiors are skipped —
        // the partner adjusts to us when it routes later.
        let mut layers: Vec<Layer> = Vec::new();
        let mut idx = (bi + 1) % k;
        while idx != bj {
            if let Event::InteriorCrossing {
                other_loop, slot, ..
            } = rot[idx]
            {
                if let Some(partner) = routed.get(&other_loop) {
                    layers.push(Layer {
                        partner: partner.iter().copied().collect(),
                        region: region.clone(),
                        label: format!("I:{:?}@{:?}", third(loop_axis, slot.0), other_loop),
                    });
                }
            }
            idx = (idx + 1) % k;
        }
        let exit_dir = match rot[bj] {
            Event::Boundary { boundary, .. } => boundary_map
                .get_by_right(&boundary)
                .and_then(|&e| skeleton.edge_weight(e))
                .map(|w| w.direction),
            _ => None,
        };
        layers.push(Layer {
            partner: [cp_end].into_iter().collect(),
            region,
            label: format!("B:{exit_dir:?}"),
        });

        // No self-avoidance pass is needed: every layer is a loop-crossing, and by the Jordan-curve
        // property a path cannot return across a loop it has already crossed — so within one segment
        // the layered search is simple by construction.
        match route_segment_layered(
            seed,
            target,
            &layers,
            &loop_blocked,
            all_crossing_edges,
            loop_axis,
            winding_sign,
            mesh,
        ) {
            Ok(edges) => {
                block_loop_occupancy(&edges, all_crossing_edges, &mut loop_blocked, mesh);
                loop_edges.extend(edges);
            }
            Err(fail) => {
                error!(
                    "Dropped {loop_axis:?}-loop: segment {s}/{nb} stalled after routing {}/{} crossings",
                    fail.reached, fail.total
                );
                loop_edges.extend(fail.partial);
                return LoopRoute::Dropped(loop_axis, loop_edges, fail.gap);
            }
        }
    }

    LoopRoute::Routed(loop_axis, loop_edges)
}

pub fn face_centroid(f: FaceID, mesh: &Mesh<INPUT>) -> Vector3D {
    let mut sum = Vector3D::zeros();
    let mut n = 0.0;
    for v in mesh.vertices(f) {
        sum += mesh.position(v);
        n += 1.0;
    }
    sum / n
}

/// Layered Dijkstra over the mesh dual graph for ONE patch-segment (state = `(face, layer, invert)`).
/// Starting on `seed` (the patch-side face of the segment's entry anchor, layer 0), the loop crosses
/// `layers[0].partner`, …, `layers[k-1].partner` in order — each an atomic straight-through crossing
/// via [`quad_diagonal_partner`]. The final layer is the exit anchor: crossing it must land on
/// `target` (the next segment's seed), stitching the segments together. Every layer confines travel
/// to the segment's single-patch `region`. The cost is the alignment-primary, distance-regularized
/// measure on [`W_ALIGN`].
///
/// `invert` is the per-band orientation of the alignment heuristic: the alignment cost is measured
/// against `signed_target` when `false` and against `-signed_target` when `true`. The search seeds
/// BOTH orientations and only lets the flag flip AT a crossing (a layer transition), so the
/// heuristic stays consistent within each band but each band independently picks whichever
/// orientation the local surface prefers (e.g. the inside vs. the outside of a torus, where the
/// face normals — and thus `d × n` — point opposite ways). No global pre-computation of the
/// orientation is needed: Dijkstra keeps the cheaper of the two branches at every crossing.
///
/// Because every partner's own arms are in `blocked`, the loop necessarily takes the complementary
/// diagonal at each crossing, so the 4 arms stay distinct by construction — no separate 4-arm check
/// is needed. `control_points` are all boundary-crossing edges: the body may not thread them as a
/// straight step (a designated crossing is realized only through the layer transition).
///
/// How far a stalled loop got: the partial path to the furthest-reached crossing layer, the
/// `(stuck_edge, next_crossing)` gap where it stalled, and the crossing-layer progress
/// (`reached` of `total`) — so both the GUI overlay and the error log can show where it broke down.
struct RouteFail {
    partial: Vec<EdgeID>,
    gap: Option<(EdgeID, EdgeID)>,
    reached: usize,
    total: usize,
}
fn route_segment_layered(
    seed: FaceID,
    target: FaceID,
    layers: &[Layer],
    blocked: &HashSet<EdgeID>,
    control_points: &HashSet<EdgeID>,
    loop_axis: PrincipalDirection,
    winding_sign: f64,
    mesh: &Mesh<INPUT>,
) -> Result<Vec<EdgeID>, RouteFail> {
    let k = layers.len();
    let signed_target: Vector3D = Vector3D::from(loop_axis) * winding_sign;
    let other_face = |edge: EdgeID, face: FaceID| -> Option<FaceID> {
        let a = mesh.root(edge);
        let b = mesh.toor(edge);
        let set_a: HashSet<FaceID> = mesh.faces(a).collect();
        mesh.faces(b).find(|&f| set_a.contains(&f) && f != face)
    };
    let in_partner = |e: EdgeID, j: usize| -> bool {
        layers[j].partner.contains(&e) || layers[j].partner.contains(&mesh.twin(e))
    };
    let in_any_partner = |e: EdgeID| -> bool { (0..k).any(|j| in_partner(e, j)) };

    // Cost of stepping from `face` (entered via `entry`) across `edge` into `next_face`. `invert`
    // selects the band's heuristic orientation: align against `signed_target` (false) or its
    // negation (true).
    let step_cost = |face: FaceID,
                     entry: Option<EdgeID>,
                     edge: EdgeID,
                     next_face: FaceID,
                     invert: bool|
     -> f64 {
        let centroid_a = face_centroid(face, mesh);
        let edge_mid = edge_midpoint_pos(edge, mesh);
        let centroid_b = face_centroid(next_face, mesh);
        let length = (centroid_a - edge_mid).norm() + (edge_mid - centroid_b).norm();
        let target = if invert { -signed_target } else { signed_target };
        let mut misalignment = 0.0;
        if let Some(en) = entry {
            let d = edge_mid - edge_midpoint_pos(en, mesh);
            let cross = d.cross(&mesh.normal(face));
            let cn = cross.norm();
            if cn > 1e-12 {
                let cos = (cross / cn).dot(&target).clamp(-1.0, 1.0);
                misalignment = cos.acos().powi(ALIGN_ALPHA);
            }
        }
        length * (LAMBDA_DIST + W_ALIGN * misalignment)
    };

    type State = (FaceID, usize, bool);
    let mut dist: HashMap<State, f64> = HashMap::new();
    // prev: state -> (parent_state, edges_used_to_reach_it). Empty edges for the seed state.
    let mut prev: HashMap<State, (State, Vec<EdgeID>)> = HashMap::new();
    let mut heap: BinaryHeap<(std::cmp::Reverse<OrderedFloat<f64>>, State)> = BinaryHeap::new();

    // Seed BOTH heuristic orientations at layer 0; the cheaper one survives. A band's orientation is
    // only allowed to change at a crossing (see the layer transition below).
    let start: State = (seed, 0, false);
    for &invert in &[false, true] {
        let s: State = (seed, 0, invert);
        dist.insert(s, 0.0);
        prev.insert(s, (s, Vec::new()));
        heap.push((std::cmp::Reverse(OrderedFloat(0.0)), s));
    }

    let mut found: Option<State> = None;
    // Furthest progress (max layer, first-popped => min cost) for the failure diagnostic.
    let mut best: State = start;

    'dijkstra: while let Some((std::cmp::Reverse(OrderedFloat(cost)), state)) = heap.pop() {
        let (face, layer, invert) = state;
        if dist.get(&state).copied().unwrap_or(f64::INFINITY) < cost {
            continue; // stale
        }
        if layer > best.1 {
            best = state;
        }
        // Entry edge into `face` AS SEEN IN `face` is the twin of the last edge used to reach it.
        let entry_in_face: Option<EdgeID> = prev
            .get(&state)
            .and_then(|(_, edges)| edges.last().copied())
            .map(|e| mesh.twin(e));

        let face_verts: Vec<VertID> = mesh.vertices(face).collect();
        let nfv = face_verts.len();
        for i in 0..nfv {
            let a = face_verts[i];
            let b = face_verts[(i + 1) % nfv];
            let Some((edge, _)) = mesh.edge_between_verts(a, b) else {
                continue;
            };
            let Some(next_face) = other_face(edge, face) else {
                continue;
            };

            // One-way transition: cross layers[layer].partner exactly once.
            if layer < k && in_partner(edge, layer) {
                let target_layer = layer + 1;

                if target_layer == k {
                    // Closing on the exit anchor: cross it straight onto `target`, the next segment's
                    // seed. The boundary loop's own arms at the anchor are blocked, so this last face
                    // was necessarily entered on the free diagonal AND the next segment will leave
                    // `target` on that diagonal's partner arm (its only free exit) — so the 4 arms
                    // stay distinct across the seam by construction. Only the anchor edge is recorded
                    // here; its exit arm is the next segment's first body step.
                    if next_face != target {
                        continue;
                    }
                    // Terminal crossing: no band follows, so the orientation is irrelevant here,
                    // keep the current flag and just close.
                    let new_cost = cost + step_cost(face, entry_in_face, edge, next_face, invert);
                    let ns: State = (next_face, target_layer, invert);
                    if new_cost < dist.get(&ns).copied().unwrap_or(f64::INFINITY) {
                        dist.insert(ns, new_cost);
                        prev.insert(ns, (state, vec![edge]));
                        found = Some(ns);
                        break 'dijkstra;
                    }
                    continue;
                }

                // Non-closing crossing: atomic straight-through (diagonal) crossing.
                let Some(entry) = entry_in_face else { continue }; // need an arm to cut across
                let Some(exit_arm) = quad_diagonal_partner(entry, edge, mesh) else {
                    continue;
                };
                // The exit arm is the loop's own new edge — must be free (the partner's own arm is
                // in `blocked`, so this rejects the degenerate same-diagonal-as-partner crossing).
                if blocked.contains(&exit_arm) {
                    continue;
                }
                let Some(land) = other_face(exit_arm, next_face) else {
                    continue;
                };
                // The landing band must be the next layer's region; the mid-crossing face may sit in
                // either band (their patch-regions overlap on the shared boundary).
                if !layers[target_layer].region.contains(&land) {
                    continue;
                }
                if !layers[layer].region.contains(&next_face)
                    && !layers[target_layer].region.contains(&next_face)
                {
                    continue;
                }
                // `c1` (leaving the current band, into the crossing quad) keeps this band's flag.
                let c1 = step_cost(face, entry_in_face, edge, next_face, invert);
                // A crossing is the ONLY place the heuristic may flip: branch into both orientations
                // for the band we land in, computing `c2` (the landing step) with the new flag, and
                // let Dijkstra keep whichever is cheaper. This is how each segment auto-picks the
                // orientation the local surface prefers without any pre-computation.
                for new_invert in [false, true] {
                    let c2 = step_cost(next_face, Some(edge), exit_arm, land, new_invert);
                    let new_cost = cost + c1 + c2;
                    let ns: State = (land, target_layer, new_invert);
                    if new_cost < dist.get(&ns).copied().unwrap_or(f64::INFINITY) {
                        dist.insert(ns, new_cost);
                        prev.insert(ns, (state, vec![edge, exit_arm]));
                        heap.push((std::cmp::Reverse(OrderedFloat(new_cost)), ns));
                    }
                }
                continue;
            }

            // Never cross any partner out of order (those edges are also blocked already).
            if in_any_partner(edge) {
                continue;
            }
            if blocked.contains(&edge) {
                continue;
            }
            // The body may not thread a boundary-crossing edge as a straight step; designated
            // crossings happen only via the layer transition above.
            if control_points.contains(&edge) || control_points.contains(&mesh.twin(edge)) {
                continue;
            }
            // Patch locality: stay within this layer's band.
            if !layers[layer].region.contains(&next_face) {
                continue;
            }

            // Body travel stays within the band, so the orientation flag is carried unchanged.
            let new_cost = cost + step_cost(face, entry_in_face, edge, next_face, invert);
            let ns: State = (next_face, layer, invert);
            if new_cost < dist.get(&ns).copied().unwrap_or(f64::INFINITY) {
                dist.insert(ns, new_cost);
                prev.insert(ns, (state, vec![edge]));
                heap.push((std::cmp::Reverse(OrderedFloat(new_cost)), ns));
            }
        }
    }

    // Reconstruct the edge sequence reached at `end` (seed -> end), in traversal order.
    let reconstruct = |end: State| -> Vec<EdgeID> {
        let mut state = end;
        let mut rev: Vec<EdgeID> = Vec::new();
        loop {
            let (parent, edges) = &prev[&state];
            if edges.is_empty() {
                break; // reached the seed state
            }
            for &e in edges.iter().rev() {
                rev.push(e);
            }
            state = *parent;
        }
        rev.reverse();
        rev
    };

    match found {
        Some(end) => Ok(reconstruct(end)),
        None => {
            // DEBUG: at the stall band, can the furthest face even reach the next crossing under the
            // actual walls? Distinguishes a fenced pocket from a crossing-mechanics rejection.
            {
                let sl = best.1;
                let reg = &layers[sl].region;
                let pf: HashSet<FaceID> = layers[sl]
                    .partner
                    .iter()
                    .flat_map(|&e| [mesh.face(e), mesh.face(mesh.twin(e))])
                    .collect();
                let mut vis: HashSet<FaceID> = HashSet::new();
                let mut q: VecDeque<FaceID> = VecDeque::new();
                vis.insert(best.0);
                q.push_back(best.0);
                let mut reached_p = false;
                while let Some(f) = q.pop_front() {
                    if pf.contains(&f) {
                        reached_p = true;
                    }
                    for e in mesh.edges(f) {
                        let nf = mesh.face(mesh.twin(e));
                        let walled = in_any_partner(e)
                            || blocked.contains(&e)
                            || control_points.contains(&e)
                            || control_points.contains(&mesh.twin(e));
                        if !walled && reg.contains(&nf) && vis.insert(nf) {
                            q.push_back(nf);
                        }
                    }
                }
                // The full crossing itinerary, marking what's DONE, what it's STUCK on, and TODO.
                let itinerary: String = layers
                    .iter()
                    .enumerate()
                    .map(|(j, l)| {
                        let mark = match j.cmp(&sl) {
                            std::cmp::Ordering::Less => "^",
                            std::cmp::Ordering::Equal => "-STUCK-",
                            std::cmp::Ordering::Greater => ">",
                        };
                        format!("{}{}", l.label, mark)
                    })
                    .collect::<Vec<_>>()
                    .join(" ");
                error!(
                    "STALL {loop_axis:?}: {sl}/{k} done | stuck on {} (reg_n={} pocket={} reached_partner={} pf_in_reg={}/{})",
                    layers[sl].label,
                    reg.len(),
                    vis.len(),
                    reached_p,
                    pf.iter().filter(|f| reg.contains(f)).count(),
                    pf.len(),
                );
                error!("  itinerary {loop_axis:?}: {itinerary}");
            }
            // Stalled: report the furthest path and the next crossing it could not reach. Even with
            // zero progress (still at the seed, no partial edges), anchor the gap on a seed-face edge
            // so the overlay still marks WHERE and toward WHICH crossing the loop got stuck.
            let partial = reconstruct(best);
            let stuck = partial
                .last()
                .copied()
                .or_else(|| mesh.edges(best.0).next());
            let gap = stuck.and_then(|stuck| {
                layers
                    .get(best.1)
                    .and_then(|l| l.partner.iter().next().copied())
                    .map(|next_crossing| (stuck, next_crossing))
            });
            Err(RouteFail {
                partial,
                gap,
                reached: best.1,
                total: k,
            })
        }
    }
}

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
