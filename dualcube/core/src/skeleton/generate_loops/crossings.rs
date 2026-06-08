//! Boundary-crossing placement: chooses where each pair of orthogonal loops crosses every
//! patch-patch boundary, then slides those crossings onto straight, threadable boundary edges.

use std::collections::{HashMap, HashSet, VecDeque};

use bimap::BiHashMap;
use log::{error, info};
use mehsh::prelude::{HasEdges, HasPosition, Mesh, Vector3D};
use petgraph::{
    graph::EdgeIndex,
    visit::{EdgeRef, IntoEdgeReferences},
};
use slotmap::SlotMap;

use crate::{
    prelude::{EdgeID, FaceID, PrincipalDirection, INPUT},
    skeleton::{
        boundary_loop::BoundaryLoop,
        geometry::angle_distance,
        orthogonalize::{AxisSign, LabeledCurveSkeleton},
    },
    solutions::{Loop, LoopID},
};

use super::axes::{third, ALL_DIRS, ALL_SIGNS};
use super::router::quad_diagonal_partner;
use super::CrossingMap;

/// Builds a `Loop` from a boundary loop: its edge list is the boundary's
/// own edge midpoints, tagged with the boundary's direction.
fn get_loop(boundary: BoundaryLoop, direction: PrincipalDirection) -> Loop {
    Loop {
        edges: boundary.edge_midpoints,
        direction,
    }
}

/// Calculates for each patch-patch boundary the appropriate loop and crossing points for the other two loop types.
///
/// For each boundary loop, places 4 crossings by:
/// 1. Projecting the boundary points onto the loop's plane (perpendicular to the skeleton edge).
/// 2. Computing angles from the centroid in that plane.
/// 3. For each orthogonal (direction, sign), projecting the axis direction onto the plane to
///    get a target angle, then picking the boundary point closest to that target angle.
///
/// This ensures crossings are naturally spread around the loop (~90 deg apart for axis-aligned geometry)
/// without needing direction propagation between nodes.
pub(super) fn get_boundaries_and_crossing_points(
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
pub(super) fn repair_boundary_crossings(
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
            let Some(ds_map) = crossings.get(&loop_id) else {
                continue;
            };
            if ds_map.is_empty() {
                continue;
            }
            ds_map
                .iter()
                .map(|(&k, &e)| {
                    (
                        k,
                        *pos_of.get(&e).expect("crossing must lie on its boundary"),
                    )
                })
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
        if unrepairable > 0 {
            error!(
                "boundary crossings: {} invalid (bend or boxed-in), {} repaired, {} unrepairable",
                invalid, repaired, unrepairable
            );
        } else {
            info!(
                "boundary crossings: {} invalid (bend or boxed-in), {} repaired, {} unrepairable",
                invalid, repaired, unrepairable
            );
        }
    }
}
