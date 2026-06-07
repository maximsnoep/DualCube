//! Post-hoc invariant checks and inline diagnostics for the planner. None of this affects routing —
//! it is the permanent safety net (structural-invariant warnings) plus the `DIAG`/`AUDIT` log lines
//! the planner emits while tracing loops. Kept out of [`super::planner`] so the planning logic reads
//! cleanly; every function here only ever emits `warn!` lines.

use std::collections::{HashMap, HashSet};

use log::warn;
use mehsh::prelude::Mesh;
use petgraph::{graph::NodeIndex, visit::EdgeRef};
use slotmap::SlotMap;

use crate::{
    prelude::{EdgeID, PrincipalDirection, INPUT},
    skeleton::{
        geometry::edge_midpoint_pos,
        orthogonalize::{AxisSign, LabeledCurveSkeleton, LabeledSkeletonSignExt},
    },
    solutions::{Loop, LoopID},
};

/// DIAG: a chord whose two anchors lie on the SAME patch boundary (same skeleton edge / boundary
/// loop). Per the user this should only be valid at a node that is a cap for THIS loop family — i.e.
/// has exactly one skeleton edge of direction != loop_axis (only one crossable boundary), forcing a
/// U-turn. Reports the wrap patch (from the interior events between the anchors), its skeleton
/// degree, and its crossable-boundary count for this family, to catch genuine misroutes.
pub(super) fn diag_same_boundary_chord(
    loop_axis: PrincipalDirection,
    bi: usize,
    nb: usize,
    bs: Option<LoopID>,
    be: Option<LoopID>,
    wrap_patch: Option<NodeIndex>,
    skeleton: &LabeledCurveSkeleton,
) {
    let (Some(bs), Some(be)) = (bs, be) else { return };
    if bs != be {
        return;
    }
    let (deg, crossable) = match wrap_patch {
        Some(p) => {
            let deg = skeleton.edges(p).count();
            let crossable = skeleton
                .edges(p)
                .filter(|e| e.weight().direction != loop_axis)
                .count();
            (deg as i64, crossable as i64)
        }
        None => (-1, -1),
    };
    warn!(
        "DIAG same-boundary chord: {:?}-loop chord {}/{} on boundary {:?}; \
         wrap-patch {:?} skel-degree {} crossable-for-family {}",
        loop_axis, bi + 1, nb, bs, wrap_patch, deg, crossable
    );
}

/// DIAG: a partner appearing twice in one chord => the layered router crosses the same pair of loops
/// twice (the observed in-patch double-crossing).
pub(super) fn diag_double_partner(
    loop_axis: PrincipalDirection,
    bi: usize,
    nb: usize,
    partner_ids: &[(LoopID, NodeIndex, (PrincipalDirection, AxisSign))],
    skeleton: &LabeledCurveSkeleton,
) {
    let mut seen_p: HashMap<LoopID, Vec<(NodeIndex, (PrincipalDirection, AxisSign))>> =
        HashMap::new();
    for &(p, patch, slot) in partner_ids {
        seen_p.entry(p).or_default().push((patch, slot));
    }
    for (p, slots) in &seen_p {
        if slots.len() > 1 {
            let patch = slots[0].0;
            let dirs: Vec<_> = skeleton
                .edges(patch)
                .map(|e| {
                    let s = skeleton.edge_sign_from(e.id(), patch).expect("endpoint");
                    (e.weight().direction, s)
                })
                .collect();
            warn!(
                "DIAG double-partner: {:?}-loop chord {}/{} crosses partner {:?} \
                 {} times at {:?}; patch {:?} degree {} boundaries {:?}",
                loop_axis, bi + 1, nb, p, slots.len(), slots,
                patch, dirs.len(), dirs
            );
        }
    }
}

/// DIAG: self-intersection. A valid dual loop is SIMPLE — it must never reuse a geometric edge
/// (either half). Reports each reuse with its chord indices and position.
pub(super) fn diag_self_cross(
    loop_axis: PrincipalDirection,
    loop_edges: &[EdgeID],
    loop_chords: &[Vec<EdgeID>],
    mesh: &Mesh<INPUT>,
) {
    let mut first_seen: HashMap<EdgeID, usize> = HashMap::new();
    // Map each edge position to its chord index (loop_chords order).
    let mut pos_chord: Vec<usize> = Vec::with_capacity(loop_edges.len());
    for (ci, seg) in loop_chords.iter().enumerate() {
        for _ in seg {
            pos_chord.push(ci);
        }
    }
    for (idx, &e) in loop_edges.iter().enumerate() {
        let ci = pos_chord.get(idx).copied().unwrap_or(usize::MAX);
        let t = mesh.twin(e);
        if let Some(&pidx) = first_seen.get(&e).or_else(|| first_seen.get(&t)) {
            let pci = pos_chord.get(pidx).copied().unwrap_or(usize::MAX);
            let p = edge_midpoint_pos(e, mesh);
            warn!(
                "DIAG self-cross: {:?}-loop reuses geo-edge (chord {} & chord {}) @({:.1},{:.1},{:.1})",
                loop_axis, pci, ci, p.x, p.y, p.z
            );
        }
        first_seen.entry(e).or_insert(idx);
        first_seen.entry(t).or_insert(idx);
    }
}

/// INVARIANT CHECK: with natural interior crossings, two PERPENDICULAR loops legitimately share a
/// single edge wherever they cross — that is no longer an error. The router must still never produce:
/// (a) an edge shared by >=3 loops, or (b) an edge shared by two SAME-axis loops (same-axis loops
/// must never cross). Both are real structural bugs; the 4-arm validity of legitimate crossings is
/// then checked by the dual. Also flags the segment-bigon (two chord-SEGMENTS crossing more than
/// once). Cheap, permanent safety net — emits `AUDIT`/`DIAG` warnings only.
pub(super) fn audit_loops(
    map: &SlotMap<LoopID, Loop>,
    committed_segments: &[(LoopID, PrincipalDirection, Vec<Vec<EdgeID>>)],
    all_control_points: &HashSet<EdgeID>,
    mesh: &Mesh<INPUT>,
) {
    let mut occ: HashMap<EdgeID, HashSet<LoopID>> = HashMap::new();
    for (lid, l) in map.iter() {
        for &e in &l.edges {
            occ.entry(e).or_default().insert(lid);
            occ.entry(mesh.twin(e)).or_default().insert(lid);
        }
    }
    let mut visited: HashSet<EdgeID> = HashSet::new();
    let mut three_plus: usize = 0;
    let mut same_axis: usize = 0;
    for (&e, lids) in &occ {
        if !visited.insert(e) {
            continue;
        }
        visited.insert(mesh.twin(e));
        if lids.len() >= 3 {
            three_plus += 1;
            warn!("AUDIT >=3: geo-edge {:?} used by loops {:?}", e, lids);
        } else if lids.len() == 2 {
            let v: Vec<LoopID> = lids.iter().copied().collect();
            if map[v[0]].direction == map[v[1]].direction {
                same_axis += 1;
            }
        }
    }

    // Count vertex-disjoint clusters among a set of geo-edges (= number of distinct crossing
    // locations). Two edges are in the same cluster if they share a mesh vertex.
    fn uf_find(parent: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while parent[r] != r {
            r = parent[r];
        }
        let mut c = x;
        while parent[c] != r {
            let nx = parent[c];
            parent[c] = r;
            c = nx;
        }
        r
    }
    let cluster_count = |edges: &[EdgeID]| -> usize {
        let n = edges.len();
        let mut parent: Vec<usize> = (0..n).collect();
        for i in 0..n {
            let vi = [mesh.root(edges[i]), mesh.toor(edges[i])];
            for j in (i + 1)..n {
                let vj = [mesh.root(edges[j]), mesh.toor(edges[j])];
                if vi.iter().any(|x| vj.contains(x)) {
                    let (ri, rj) = (uf_find(&mut parent, i), uf_find(&mut parent, j));
                    parent[ri] = rj;
                }
            }
        }
        (0..n).map(|i| uf_find(&mut parent, i)).collect::<HashSet<_>>().len()
    };

    // THE invariant: two chord-SEGMENTS (each between consecutive boundary anchors) cross at most
    // once. Two whole loops crossing twice across the model is fine (different segment pairs); two
    // *segments* crossing twice is the illegal bigon. Build each segment's geo-edge set (both halves)
    // and, for every cross-loop segment pair, cluster the shared geo-edges: >=2 clusters means that
    // pair crosses at >=2 separate locations.
    let seg_flat: Vec<(LoopID, PrincipalDirection, usize, HashSet<EdgeID>)> = committed_segments
        .iter()
        .flat_map(|(lid, dir, chords)| {
            chords.iter().enumerate().map(move |(ci, edges)| {
                let mut set: HashSet<EdgeID> = HashSet::new();
                for &e in edges {
                    set.insert(e);
                    set.insert(mesh.twin(e));
                }
                (*lid, *dir, ci, set)
            })
        })
        .collect();
    let mut bigons: usize = 0;
    for i in 0..seg_flat.len() {
        for j in (i + 1)..seg_flat.len() {
            if seg_flat[i].0 == seg_flat[j].0 {
                continue; // same loop: consecutive segments legitimately share the anchor
            }
            // Shared geo-edges, deduped by canonical half.
            let mut shared: Vec<EdgeID> = Vec::new();
            let mut seen: HashSet<EdgeID> = HashSet::new();
            for &e in &seg_flat[i].3 {
                if seg_flat[j].3.contains(&e) && seen.insert(e) {
                    seen.insert(mesh.twin(e));
                    shared.push(e);
                }
            }
            if shared.len() >= 2 && cluster_count(&shared) >= 2 {
                bigons += 1;
                let detail: Vec<String> = shared
                    .iter()
                    .map(|&e| {
                        let is_cp = all_control_points.contains(&e)
                            || all_control_points.contains(&mesh.twin(e));
                        let p = edge_midpoint_pos(e, mesh);
                        format!("{:?}{} @({:.1},{:.1},{:.1})", e, if is_cp { "[CP]" } else { "" }, p.x, p.y, p.z)
                    })
                    .collect();
                warn!(
                    "DIAG segment-bigon: {:?}({:?}) seg{} & {:?}({:?}) seg{} cross at {} locations; shared {:?}",
                    seg_flat[i].0, seg_flat[i].1, seg_flat[i].2,
                    seg_flat[j].0, seg_flat[j].1, seg_flat[j].2,
                    cluster_count(&shared), detail
                );
            }
        }
    }
    if three_plus > 0 || same_axis > 0 || bigons > 0 {
        warn!(
            "AUDIT: {} geo-edges with >=3 loops, {} same-axis shares, {} segment-bigons",
            three_plus, same_axis, bigons
        );
    }
}
