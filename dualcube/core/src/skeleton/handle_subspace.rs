//! Compute the **handle subspace** `K = ker(H₁(S; ℤ/2) → H₁(V; ℤ/2))`
//! given the input surface mesh `S` and a tetrahedralization `T` of the
//! solid `V` bounded by it.
//!
//! The output is `g` linearly independent 1-cycles `c₁, …, c_g` on `S`,
//! each of which bounds a 2-chain in `T` (= bounds inside the solid =
//! handle loop, per the design note in `HOMOLOGY_PRESERVING_SURGERY.md`).
//!
//! Algorithm:
//! 1. Enumerate surface edges and faces with stable `u32` indices.
//! 2. Build `∂₂^S` over GF(2) and reduce it; its row span is `B₁(S)`, the
//!    surface-boundary subspace.
//! 3. Build a spanning tree of the surface 1-skeleton. Every non-tree edge
//!    `e` produces a *fundamental cycle*: `e` plus the unique tree path
//!    between its endpoints. The set of fundamental cycles is a basis for
//!    the cycle space `Z₁(S)`. There are `E_S − V_S + 1` of them.
//! 4. For each fundamental cycle, lift it to a 1-chain in `C₁(T)` by
//!    translating surface edge indices to T-edge column indices (input
//!    vertices keep their indices in T by the [`tetrahedralize`]
//!    invariant). Reduce the lifted chain against the RREF of `∂₂^T`;
//!    if it reduces to zero, the cycle bounds in `T` — it's a handle.
//! 5. Among the collected handle candidates, keep only those that are
//!    linearly independent in `H₁(S) = Z₁(S) / B₁(S)`. The result is a
//!    basis for `K`.

use std::collections::{HashMap, HashSet, VecDeque};

use mehsh::prelude::{HasVertices, Mesh, VertKey};

use super::f2_rref::F2Matrix;
use super::tet_boundary::TetBoundaryComplex;
use crate::prelude::INPUT;

/// A basis for the handle subspace AND a basis for its tunnel complement,
/// plus the surface-edge indexing both cycle families are expressed over.
#[derive(Debug, Clone)]
pub struct HandleSubspace {
    /// `g` linearly independent handle cycles (= elements of `K`, the
    /// kernel of `H₁(S) → H₁(V)`). Each is a sorted list of surface-edge
    /// indices (indices into `surface_edges`).
    pub cycles: Vec<Vec<u32>>,
    /// `g` linearly independent tunnel cycles — chosen as a complement
    /// to `K` in `H₁(S)`, so the natural map `H₁(S)/K → H₁(V)` sends
    /// them to a basis. Each is a sorted list of surface-edge indices.
    pub tunnel_cycles: Vec<Vec<u32>>,
    /// Surface edges, each as a sorted pair of vertex indices. The vertex
    /// index is the position of the vertex in `mesh.vert_ids()` — i.e. the
    /// same indexing the [`super::tetrahedralize::TetMesh`] uses for the
    /// first `n_input_vertices` of its vertex list.
    pub surface_edges: Vec<[u32; 2]>,
    /// Lookup: sorted edge pair → surface edge index.
    pub edge_to_index: HashMap<[u32; 2], u32>,
}

impl HandleSubspace {
    /// `g` = number of handle cycles = genus of the solid (when input was
    /// a closed orientable surface).
    pub fn genus(&self) -> usize {
        self.cycles.len()
    }
}

/// Computes a basis for the handle subspace `K`.
///
/// The input mesh `mesh` should be the original closed orientable input
/// surface; `tet_boundary` must come from `build_tet_boundary` applied to
/// the result of `tetrahedralize(mesh)`.
pub fn compute_handle_subspace(
    mesh: &Mesh<INPUT>,
    tet_boundary: &TetBoundaryComplex,
) -> HandleSubspace {
    // ---- Step 1: enumerate surface vertices, edges, and faces. ----
    let vert_ids: Vec<VertKey<INPUT>> = mesh.vert_ids();
    let vert_index: HashMap<VertKey<INPUT>, u32> = vert_ids
        .iter()
        .enumerate()
        .map(|(i, &v)| (v, i as u32))
        .collect();
    let n_verts = vert_ids.len();

    let mut surface_edges: Vec<[u32; 2]> = Vec::new();
    let mut edge_to_index: HashMap<[u32; 2], u32> = HashMap::new();
    let mut surface_faces: Vec<[u32; 3]> = Vec::new();
    for face_id in mesh.face_ids() {
        let verts: Vec<_> = mesh.vertices(face_id).collect();
        debug_assert_eq!(verts.len(), 3, "compute_handle_subspace requires a triangulated mesh");
        let [a, b, c] = [
            vert_index[&verts[0]],
            vert_index[&verts[1]],
            vert_index[&verts[2]],
        ];
        let face = sort_triple(a, b, c);
        surface_faces.push(face);
        for (x, y) in [(face[0], face[1]), (face[1], face[2]), (face[0], face[2])] {
            let pair = sort_pair(x, y);
            edge_to_index.entry(pair).or_insert_with(|| {
                let id = surface_edges.len() as u32;
                surface_edges.push(pair);
                id
            });
        }
    }

    // ---- Step 2: build and reduce ∂₂^S so its rows span B₁(S). ----
    let mut surface_d2 = F2Matrix::from_rows(
        surface_faces
            .iter()
            .map(|face| {
                let mut cols = [
                    edge_to_index[&sort_pair(face[0], face[1])],
                    edge_to_index[&sort_pair(face[1], face[2])],
                    edge_to_index[&sort_pair(face[0], face[2])],
                ];
                cols.sort();
                cols.to_vec()
            })
            .collect(),
    );
    surface_d2.reduce();
    let surface_b1_pivots = surface_d2.pivot_map();

    // ---- Step 3: build a spanning tree of the surface 1-skeleton. ----
    let mut surface_adj: HashMap<u32, Vec<(u32, u32)>> = HashMap::new();
    for (edge_idx, &[a, b]) in surface_edges.iter().enumerate() {
        surface_adj.entry(a).or_default().push((b, edge_idx as u32));
        surface_adj.entry(b).or_default().push((a, edge_idx as u32));
    }
    let mut parent_vertex: Vec<Option<u32>> = vec![None; n_verts];
    let mut parent_edge: Vec<Option<u32>> = vec![None; n_verts];
    let mut visited: Vec<bool> = vec![false; n_verts];
    let mut tree_edges: HashSet<u32> = HashSet::new();
    visited[0] = true;
    let mut queue: VecDeque<u32> = VecDeque::new();
    queue.push_back(0);
    while let Some(v) = queue.pop_front() {
        if let Some(neighbors) = surface_adj.get(&v) {
            for &(n, edge_idx) in neighbors {
                if !visited[n as usize] {
                    visited[n as usize] = true;
                    parent_vertex[n as usize] = Some(v);
                    parent_edge[n as usize] = Some(edge_idx);
                    tree_edges.insert(edge_idx);
                    queue.push_back(n);
                }
            }
        }
    }

    // ---- Step 4: reduce ∂₂^T and build its pivot map. ----
    let mut tet_d2 = tet_boundary.boundary_2.clone();
    tet_d2.reduce();
    let tet_b1_pivots = tet_d2.pivot_map();

    // Translation table: surface edge index → T-edge column index.
    // Input surface vertices keep their indices in T (see TetMesh docs),
    // and the vendored tritet uses TetGen's `M` + `Y` flags to preserve
    // every input triangle, so every surface edge must appear as a
    // T-edge.
    let surface_to_t_col: Vec<u32> = surface_edges
        .iter()
        .map(|&[a, b]| {
            tet_boundary
                .edge_col(a, b)
                .expect("every surface edge must appear in the tet complex")
        })
        .collect();

    // ---- Step 5: for each non-tree edge, build the fundamental cycle and ----
    // ---- test whether it bounds in T (= handle loop). ----
    let mut handle_candidates: Vec<Vec<u32>> = Vec::new();
    let mut tunnel_candidates: Vec<Vec<u32>> = Vec::new();
    for edge_idx in 0..(surface_edges.len() as u32) {
        if tree_edges.contains(&edge_idx) {
            continue;
        }
        let cycle = fundamental_cycle_on_surface(
            edge_idx,
            &surface_edges,
            &parent_vertex,
            &parent_edge,
        );

        // Lift to a 1-chain in C₁(T).
        let mut lifted: Vec<u32> = cycle.iter().map(|&i| surface_to_t_col[i as usize]).collect();
        lifted.sort();

        if tet_d2.reduce_against(&tet_b1_pivots, &mut lifted) {
            // Reduced to zero — the cycle bounds in T, it's a handle.
            handle_candidates.push(cycle);
        } else {
            // Did NOT bound in T — it's a tunnel candidate.
            tunnel_candidates.push(cycle);
        }
    }

    // ---- Step 6: extract a basis modulo B₁(S). ----
    // Stage the surface-boundary rows as the initial pivot system, then add
    // candidates one at a time; a candidate joins the basis iff reducing it
    // against the existing pivots yields a non-zero residue.
    let mut pivots: HashMap<u32, Vec<u32>> = HashMap::new();
    for row in surface_d2.rows() {
        if let Some(&leading) = row.first() {
            // After `reduce()` each non-empty row already pivots its leading
            // column, so the insert is unique.
            pivots.insert(leading, row.clone());
        }
    }
    let mut chosen: Vec<Vec<u32>> = Vec::new();
    for candidate in handle_candidates {
        let mut residue = candidate.clone();
        while let Some(&leading) = residue.first() {
            if let Some(pivot_row) = pivots.get(&leading) {
                residue = xor_sorted(&residue, pivot_row);
            } else {
                break;
            }
        }
        if !residue.is_empty() {
            // The candidate is independent of B₁(S) and the already-chosen
            // handle cycles. Keep the *original* cycle (a clean fundamental
            // cycle on S) as the basis element, but file the *residue* as
            // the new pivot.
            let pivot_col = residue[0];
            pivots.insert(pivot_col, residue);
            chosen.push(candidate);
        }
    }

    // ---- Step 7: extract a tunnel basis as a complement to K. ----
    // Pivots now include both B₁(S) rows and the residues of the chosen
    // handle cycles, so reducing tunnel candidates against them yields a
    // non-zero residue exactly when the candidate is independent of K
    // mod B₁(S) — i.e. it represents a class in H₁(S)/K ≅ H₁(V) not yet
    // covered by previously-chosen tunnels.
    let mut chosen_tunnels: Vec<Vec<u32>> = Vec::new();
    for candidate in tunnel_candidates {
        let mut residue = candidate.clone();
        while let Some(&leading) = residue.first() {
            if let Some(pivot_row) = pivots.get(&leading) {
                residue = xor_sorted(&residue, pivot_row);
            } else {
                break;
            }
        }
        if !residue.is_empty() {
            let pivot_col = residue[0];
            pivots.insert(pivot_col, residue);
            chosen_tunnels.push(candidate);
        }
    }

    HandleSubspace {
        cycles: chosen,
        tunnel_cycles: chosen_tunnels,
        surface_edges,
        edge_to_index,
    }
}

/// Builds the fundamental cycle for a non-tree edge `e = (u, v)` as the
/// symmetric difference of `path_to_root(u)` and `path_to_root(v)` plus the
/// edge itself. Output is a sorted `Vec<u32>` of surface-edge indices.
fn fundamental_cycle_on_surface(
    non_tree_edge_idx: u32,
    surface_edges: &[[u32; 2]],
    parent_vertex: &[Option<u32>],
    parent_edge: &[Option<u32>],
) -> Vec<u32> {
    let [u, v] = surface_edges[non_tree_edge_idx as usize];

    let mut path_u: HashSet<u32> = HashSet::new();
    let mut cur = u;
    while let Some(pe) = parent_edge[cur as usize] {
        path_u.insert(pe);
        cur = parent_vertex[cur as usize].expect("parent_edge present implies parent_vertex");
    }
    let mut path_v: HashSet<u32> = HashSet::new();
    let mut cur = v;
    while let Some(pe) = parent_edge[cur as usize] {
        path_v.insert(pe);
        cur = parent_vertex[cur as usize].expect("parent_edge present implies parent_vertex");
    }

    let mut cycle: Vec<u32> = path_u
        .symmetric_difference(&path_v)
        .copied()
        .chain(std::iter::once(non_tree_edge_idx))
        .collect();
    cycle.sort();
    cycle
}

/// Symmetric difference of two sorted-distinct `u32` slices.
fn xor_sorted(a: &[u32], b: &[u32]) -> Vec<u32> {
    use std::cmp::Ordering;
    let mut out = Vec::with_capacity(a.len() + b.len());
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        match a[i].cmp(&b[j]) {
            Ordering::Less => {
                out.push(a[i]);
                i += 1;
            }
            Ordering::Greater => {
                out.push(b[j]);
                j += 1;
            }
            Ordering::Equal => {
                i += 1;
                j += 1;
            }
        }
    }
    out.extend_from_slice(&a[i..]);
    out.extend_from_slice(&b[j..]);
    out
}

#[inline]
fn sort_pair(a: u32, b: u32) -> [u32; 2] {
    if a < b { [a, b] } else { [b, a] }
}

#[inline]
fn sort_triple(a: u32, b: u32, c: u32) -> [u32; 3] {
    let mut x = [a, b, c];
    x.sort();
    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::skeleton::tet_boundary::build_tet_boundary;
    use crate::skeleton::tetrahedralize::tetrahedralize;
    use mehsh::prelude::Vector3D;

    fn make_mesh(positions: Vec<Vector3D>, faces: Vec<Vec<usize>>) -> Mesh<INPUT> {
        let (mesh, _, _) = Mesh::<INPUT>::from(&faces, &positions).expect("mesh build failed");
        mesh
    }

    fn build_tetrahedron_surface() -> Mesh<INPUT> {
        let positions = vec![
            Vector3D::new(0.0, 0.0, 0.0),
            Vector3D::new(1.0, 0.0, 0.0),
            Vector3D::new(0.0, 1.0, 0.0),
            Vector3D::new(0.0, 0.0, 1.0),
        ];
        let faces = vec![
            vec![0, 2, 1],
            vec![0, 1, 3],
            vec![0, 3, 2],
            vec![1, 2, 3],
        ];
        make_mesh(positions, faces)
    }

    fn build_cube_surface() -> Mesh<INPUT> {
        let positions = vec![
            Vector3D::new(0.0, 0.0, 0.0),
            Vector3D::new(1.0, 0.0, 0.0),
            Vector3D::new(1.0, 1.0, 0.0),
            Vector3D::new(0.0, 1.0, 0.0),
            Vector3D::new(0.0, 0.0, 1.0),
            Vector3D::new(1.0, 0.0, 1.0),
            Vector3D::new(1.0, 1.0, 1.0),
            Vector3D::new(0.0, 1.0, 1.0),
        ];
        let faces = vec![
            vec![0, 2, 1], vec![0, 3, 2],
            vec![4, 5, 6], vec![4, 6, 7],
            vec![0, 1, 5], vec![0, 5, 4],
            vec![3, 7, 6], vec![3, 6, 2],
            vec![0, 4, 7], vec![0, 7, 3],
            vec![1, 2, 6], vec![1, 6, 5],
        ];
        make_mesh(positions, faces)
    }

    /// A small triangulated torus. Parameters: `n_major` divisions around
    /// the big circle, `n_minor` divisions around the tube. Outer/inner
    /// radii baked in. Output is a closed, oriented genus-1 surface.
    fn build_torus_surface(n_major: usize, n_minor: usize) -> Mesh<INPUT> {
        use std::f64::consts::PI;
        let r_major = 2.0;
        let r_minor = 0.6;
        let mut positions: Vec<Vector3D> = Vec::with_capacity(n_major * n_minor);
        let idx = |i: usize, j: usize| -> usize {
            (i % n_major) * n_minor + (j % n_minor)
        };
        for i in 0..n_major {
            let theta = 2.0 * PI * (i as f64) / (n_major as f64);
            let (ct, st) = (theta.cos(), theta.sin());
            for j in 0..n_minor {
                let phi = 2.0 * PI * (j as f64) / (n_minor as f64);
                let (cp, sp) = (phi.cos(), phi.sin());
                let x = (r_major + r_minor * cp) * ct;
                let y = (r_major + r_minor * cp) * st;
                let z = r_minor * sp;
                positions.push(Vector3D::new(x, y, z));
            }
        }
        let mut faces: Vec<Vec<usize>> = Vec::new();
        for i in 0..n_major {
            for j in 0..n_minor {
                let v00 = idx(i, j);
                let v10 = idx(i + 1, j);
                let v01 = idx(i, j + 1);
                let v11 = idx(i + 1, j + 1);
                // Two triangles per grid cell, consistent outward normal.
                faces.push(vec![v00, v10, v11]);
                faces.push(vec![v00, v11, v01]);
            }
        }
        make_mesh(positions, faces)
    }

    /// A tetrahedron (genus 0) has trivial handle subspace.
    #[test]
    fn tetrahedron_has_empty_handle_subspace() {
        let mesh = build_tetrahedron_surface();
        let tm = tetrahedralize(&mesh).expect("tetgen failed");
        let tb = build_tet_boundary(&tm);
        let k = compute_handle_subspace(&mesh, &tb);
        assert_eq!(k.genus(), 0, "tetrahedron is genus 0 → no handles");
        assert_eq!(k.cycles.len(), 0);
    }

    /// A cube surface (genus 0) has trivial handle subspace.
    #[test]
    fn cube_has_empty_handle_subspace() {
        let mesh = build_cube_surface();
        let tm = tetrahedralize(&mesh).expect("tetgen failed");
        let tb = build_tet_boundary(&tm);
        let k = compute_handle_subspace(&mesh, &tb);
        assert_eq!(k.genus(), 0, "cube is genus 0 → no handles");
        assert_eq!(k.cycles.len(), 0);
    }

    /// A torus (genus 1) has a handle subspace of dimension 1.
    #[test]
    fn torus_has_one_dimensional_handle_subspace() {
        let mesh = build_torus_surface(20, 10);
        let tm = tetrahedralize(&mesh).expect("tetgen failed");
        let tb = build_tet_boundary(&tm);
        let k = compute_handle_subspace(&mesh, &tb);
        assert_eq!(k.genus(), 1, "torus is genus 1 → exactly one handle");
        assert_eq!(k.cycles.len(), 1);
        assert!(!k.cycles[0].is_empty(), "handle cycle should have edges");

        // Genus-1 surface also has exactly one tunnel class.
        assert_eq!(k.tunnel_cycles.len(), 1, "torus has one tunnel");
        assert!(!k.tunnel_cycles[0].is_empty(), "tunnel cycle should have edges");
    }

    /// Genus-0 surfaces have no tunnels either.
    #[test]
    fn cube_has_empty_tunnel_subspace() {
        let mesh = build_cube_surface();
        let tm = tetrahedralize(&mesh).expect("tetgen failed");
        let tb = build_tet_boundary(&tm);
        let k = compute_handle_subspace(&mesh, &tb);
        assert_eq!(k.cycles.len(), 0);
        assert_eq!(k.tunnel_cycles.len(), 0);
    }

    /// The returned cycles' edges should all reference valid surface-edge
    /// indices.
    #[test]
    fn handle_cycles_reference_valid_edges() {
        let mesh = build_torus_surface(16, 8);
        let tm = tetrahedralize(&mesh).expect("tetgen failed");
        let tb = build_tet_boundary(&tm);
        let k = compute_handle_subspace(&mesh, &tb);
        let n_edges = k.surface_edges.len() as u32;
        for cycle in &k.cycles {
            for &edge_idx in cycle {
                assert!(edge_idx < n_edges, "cycle references out-of-range edge {}", edge_idx);
            }
        }
    }
}
