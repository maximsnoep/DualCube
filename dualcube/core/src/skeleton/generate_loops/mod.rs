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

mod axes;
mod crossings;
mod diagnostics;
mod planner;
mod router;

use std::collections::{HashMap, HashSet};

use mehsh::prelude::{HasEdges, HasNormal, HasPosition, HasVertices, Mesh, Vector3D};
use petgraph::unionfind::UnionFind;
use slotmap::SlotMap;

use crate::{
    prelude::{EdgeID, FaceID, INPUT, PrincipalDirection},
    skeleton::{
        SkeletonData, generate_loops::router::{face_centroid, route_segments}, orthogonalize::{AxisSign, LabeledCurveSkeleton}
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
    let (boundary_map, mut crossings) =
        get_boundaries_and_crossing_points(skeleton, mesh, &mut map);

    // Slide each boundary crossing onto a straight, threadable boundary edge so the boundary
    // loop crosses diagonally (leaving the complementary diagonal free for the orthogonal loop)
    // and the orthogonal loop can actually pass through without being boxed in. Topological.
    repair_boundary_crossings(&map, &mut crossings, mesh);

    // Trace paths between boundary crossings to create the loops. The planner only reads these maps,
    // so they are borrowed (no clone) — `crossings` is still returned to the caller below.
    let segment_plan = pathing_for_loops(LoopPlan {
        boundary_map: &boundary_map,
        crossings: &crossings,
        skeleton,
    });

    // Take the produced plan and actually route the loops on the mesh.
    route_segments(
        &mut map,
        &segment_plan,
        &crossings,
        &boundary_map,
        skeleton,
        mesh,
        &mut diagnostics,
    );

    // Routing produces one half-edge per crossed geometric edge; `Dual` requires both halves (it
    // canonicalizes crossings to the higher half-edge and its quad-walk expects each edge's twin
    // adjacent in the sequence). Expand AFTER pathing so blocking still sees the single-half form.
    for (_, l) in map.iter_mut() {
        let raw = std::mem::take(&mut l.edges);
        l.edges = expand_to_double_halfedges(raw, mesh);
    }

    // Make level graphs consistent.
    orient_loops(&mut map, mesh);

    Ok((map, crossings, diagnostics))
}

/// Surface-aware co-orientation of a loop along `axis`: how much the loop's in-surface co-normal
/// points toward `+axis`, summed (length-weighted) over the loop. For each step the traversal
/// tangent `t` (between consecutive edge midpoints) is rotated 90 degrees within the surface via the
/// real surface normal `n` (`t x n`, the in-surface direction pointing across the loop), and we
/// accumulate its `axis` component. The sign flips when the loop is reversed (traversal `t` flips),
/// so it picks a winding.
///
/// Unlike a flat projected signed area, this uses the *actual* surface normal, so it stays consistent
/// where the surface faces different ways -- e.g. the inner-hole vs outer loops of a torus, which a
/// projection-winding sign gets backwards (causing level-graph cycles).
fn axis_coorientation(edges: &[EdgeID], axis: PrincipalDirection, mesh: &Mesh<INPUT>) -> f64 {
    let a = Vector3D::from(axis);
    let n = edges.len();
    if n < 2 {
        return 0.0;
    }
    let mut acc = 0.0;
    for i in 0..n {
        let tangent =
            edge_midpoint_pos(edges[(i + 1) % n], mesh) - edge_midpoint_pos(edges[i], mesh);
        let normal = mesh.normal(edges[i]);
        acc += tangent.cross(&normal).dot(&a);
    }
    acc
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

/// Orients every loop so the per-axis level graphs are acyclic.
fn orient_loops(map: &mut SlotMap<LoopID, Loop>, mesh: &Mesh<INPUT>) {
    // First orient everything based on geometry.
    for (_, l) in map.iter_mut() {
        if axis_coorientation(&l.edges, l.direction, mesh) < 0.0 {
            l.edges.reverse();
        }
    }
    // Fix any cycles using brute force.
    repair_level_graph_cycles(map, mesh);
}



/// Returns true iff the directed multigraph is acyclic, with each loop's edges reversed when its bit
/// is set in `flips`. Edges are `(from_zone, to_zone, loop_local_index)`; `nzones` is the node count.
fn level_graph_acyclic(edges: &[(usize, usize, usize)], flips: u64, nzones: usize) -> bool {
    let mut indeg = vec![0usize; nzones];
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nzones];
    for &(u, v, li) in edges {
        let (s, t) = if (flips >> li) & 1 == 1 {
            (v, u)
        } else {
            (u, v)
        };
        adj[s].push(t);
        indeg[t] += 1;
    }
    let mut queue: Vec<usize> = (0..nzones).filter(|&z| indeg[z] == 0).collect();
    let mut done = 0usize;
    while let Some(u) = queue.pop() {
        done += 1;
        for &t in &adj[u] {
            indeg[t] -= 1;
            if indeg[t] == 0 {
                queue.push(t);
            }
        }
    }
    done == nzones
}

/// Finds one directed cycle (with `flips` applied) and returns the loop-local indices on it, or
/// `None` if acyclic. Used to focus repair flips on an actual offending cycle.
fn find_cycle_loops(
    edges: &[(usize, usize, usize)],
    flips: u64,
    nzones: usize,
) -> Option<Vec<usize>> {
    // Adjacency carrying the loop index per directed edge.
    let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); nzones];
    for &(u, v, li) in edges {
        let (s, t) = if (flips >> li) & 1 == 1 {
            (v, u)
        } else {
            (u, v)
        };
        adj[s].push((t, li));
    }
    // Iterative DFS with colors: 0 = unseen, 1 = on stack, 2 = done.
    let mut color = vec![0u8; nzones];
    // For each node: the (predecessor, loop) edge used to enter it on the current path.
    let mut enter: Vec<(usize, usize)> = vec![(usize::MAX, usize::MAX); nzones];
    for start in 0..nzones {
        if color[start] != 0 {
            continue;
        }
        // Stack of (node, index into adj[node]).
        let mut stack: Vec<(usize, usize)> = vec![(start, 0)];
        color[start] = 1;
        while let Some(&(u, i)) = stack.last() {
            if i < adj[u].len() {
                stack.last_mut().unwrap().1 += 1;
                let (w, li) = adj[u][i];
                match color[w] {
                    0 => {
                        enter[w] = (u, li);
                        color[w] = 1;
                        stack.push((w, 0));
                    }
                    1 => {
                        // Back edge u -> w: reconstruct the cycle w .. u plus this edge.
                        let mut loops = vec![li];
                        let mut x = u;
                        while x != w {
                            let (p, el) = enter[x];
                            loops.push(el);
                            x = p;
                        }
                        return Some(loops);
                    }
                    _ => {}
                }
            } else {
                color[u] = 2;
                stack.pop();
            }
        }
    }
    None
}

/// Per axis, reverses loops as needed so that axis's level graph is acyclic.
fn repair_level_graph_cycles(map: &mut SlotMap<LoopID, Loop>, mesh: &Mesh<INPUT>) {
    let faces = mesh.face_ids();
    let face_index: HashMap<FaceID, usize> =
        faces.iter().enumerate().map(|(i, &f)| (f, i)).collect();

    for axis in [
        PrincipalDirection::X,
        PrincipalDirection::Y,
        PrincipalDirection::Z,
    ] {
        let a_loops: Vec<LoopID> = map
            .iter()
            .filter(|(_, l)| l.direction == axis)
            .map(|(id, _)| id)
            .collect();
        if a_loops.len() < 2 {
            continue;
        }
        let loop_local: HashMap<LoopID, usize> =
            a_loops.iter().enumerate().map(|(i, &id)| (id, i)).collect();

        // Walls = this axis's loop edges. Zones = face components when only walls block.
        let walls: HashSet<EdgeID> = a_loops
            .iter()
            .flat_map(|&id| map[id].edges.iter().copied())
            .collect();
        let mut uf = UnionFind::<usize>::new(faces.len());
        for &f in &faces {
            for e in mesh.edges(f) {
                if !walls.contains(&e) {
                    uf.union(face_index[&f], face_index[&mesh.face(mesh.twin(e))]);
                }
            }
        }

        // One directed edge per (loop, zone-pair): a loop is a single cut, so it must contribute a
        // single consistent direction between two zones -- not one edge per half-edge (whose local
        // co-normal can flip along the loop, fabricating a flip-invariant 2-cycle). We accumulate a
        // signed vote over the loop's half-edges (which zone its in-surface co-normal points toward,
        // i.e. its `+axis` side) and emit one edge in the winning direction.
        let mut zone_id: HashMap<usize, usize> = HashMap::new();
        let next_zone = |zone_id: &mut HashMap<usize, usize>, r: usize| -> usize {
            let len = zone_id.len();
            *zone_id.entry(r).or_insert(len)
        };
        // votes[(loop_local, za, zb)] with za < zb: positive => zb is the `+` side, negative => za is.
        let mut votes: HashMap<(usize, usize, usize), f64> = HashMap::new();
        for &lid in &a_loops {
            let li = loop_local[&lid];
            let le = &map[lid].edges;
            let n = le.len();
            for i in 0..n {
                let e = le[i];
                let (fu, fv) = (mesh.face(e), mesh.face(mesh.twin(e)));
                let (ru, rv) = (uf.find(face_index[&fu]), uf.find(face_index[&fv]));
                if ru == rv {
                    continue;
                }
                let mid = edge_midpoint_pos(e, mesh);
                let tangent = edge_midpoint_pos(le[(i + 1) % n], mesh) - mid;
                let conormal = tangent.cross(&mesh.normal(e));
                // > 0 means fv's zone (rv) is on the co-normal (`+`) side.
                let plus_v = (face_centroid(fv, mesh) - mid).dot(&conormal)
                    - (face_centroid(fu, mesh) - mid).dot(&conormal);
                let (zu, zv) = (next_zone(&mut zone_id, ru), next_zone(&mut zone_id, rv));
                // Normalize to the ordered key (za < zb); `sign` re-expresses `plus_v` as "is zb the + side".
                let (za, zb, sign) = if zu < zv {
                    (zu, zv, 1.0)
                } else {
                    (zv, zu, -1.0)
                };
                *votes.entry((li, za, zb)).or_insert(0.0) += plus_v * sign;
            }
        }
        let nzones = zone_id.len();
        let edges: Vec<(usize, usize, usize)> = votes
            .into_iter()
            .map(|((li, za, zb), w)| if w >= 0.0 { (za, zb, li) } else { (zb, za, li) })
            .collect();
        let k = a_loops.len();

        // Find a flip mask (bit i set => additionally reverse a_loops[i]) giving an acyclic graph.
        let flips = if level_graph_acyclic(&edges, 0, nzones) {
            Some(0u64)
        } else if k <= 16 {
            // Exhaustive over flip masks (bounded by 2^16): pick an acyclic one with the fewest flips,
            // staying as close to the geometric init as possible. A valid orientation is guaranteed.
            (0u64..(1u64 << k))
                .filter(|&m| level_graph_acyclic(&edges, m, nzones))
                .min_by_key(|m| m.count_ones())
        } else {
            // Too many loops for exhaustive search: greedily flip a loop on a detected cycle.
            let mut m = 0u64;
            let mut ok = false;
            for _ in 0..(8 * k) {
                match find_cycle_loops(&edges, m, nzones) {
                    None => {
                        ok = true;
                        break;
                    }
                    Some(cycle) => {
                        // Flip the cycle loop with the fewest flips so far (avoid oscillation).
                        if let Some(&li) = cycle.iter().min_by_key(|&&li| (m >> li) & 1) {
                            m ^= 1 << li;
                        }
                    }
                }
            }
            ok.then_some(m)
        };

        match flips {
            Some(m) => {
                let reversed = (0..k).filter(|&i| (m >> i) & 1 == 1).count();
                for i in 0..k {
                    if (m >> i) & 1 == 1 {
                        map[a_loops[i]].edges.reverse();
                    }
                }
                if reversed > 0 {
                    log::info!("orient_loops: {axis} level graph had a cycle; repaired by reversing {reversed}/{k} loop(s)");
                }
            }
            None => log::warn!(
                "orient_loops: could not find an acyclic orientation for the {axis} level graph ({k} loops); it may still contain cycles."
            ),
        }
    }
}