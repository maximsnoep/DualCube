//! Connectivity Surgery: Edge collapse to extract a 1D curve skeleton.
//!
//! This module implements the connectivity surgery step from the
//! "Skeleton Extraction by Mesh Contraction" paper. After mesh contraction,
//! the mesh is thin but still 2D. This step collapses edges until only
//! a 1D skeleton remains.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use log::{error, info, warn};
use mehsh::prelude::{HasNeighbors, HasPosition, HasVertices, Mesh, Vector3D, VertKey};
use nalgebra::{Matrix4, Vector4};

use super::contraction::CONTRACTION;
use super::curve_skeleton::{CurveSkeleton, CurveSkeletonSpatial};
use super::f2_rref::F2Matrix;
use super::handle_subspace::compute_handle_subspace;
use super::tet_boundary::build_tet_boundary;
use super::tetrahedralize::tetrahedralize;
use crate::prelude::INPUT;
use crate::skeleton::boundary_loop::BoundaryLoop;
use crate::skeleton::curve_skeleton::SkeletonNode;

/// Internal vertex index type for contraction mesh.
type VIdx = VertKey<CONTRACTION>;

/// Collapse candidate for the priority queue.
#[derive(Debug)]
struct CollapseCandidate {
    /// Source vertex (will be removed).
    u: VIdx,
    /// Target vertex (will absorb u).
    v: VIdx,
    /// Collapse cost (lower is better).
    cost: f64,
}

impl Ord for CollapseCandidate {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap behavior
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for CollapseCandidate {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for CollapseCandidate {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for CollapseCandidate {}

/// Boundary matrix ∂₂ over GF(2) plus the maps needed to address rows by
/// face triple and columns by undirected edge, plus a basis of the handle
/// subspace `K ⊆ H₁(S; ℤ/2)`. Maintained throughout surgery so the
/// legality check can probe what β₁ and dim(K) would become after a
/// hypothetical collapse.
#[derive(Clone)]
struct BoundaryMatrix {
    matrix: F2Matrix,
    /// Sorted face triple -> row index in `matrix`. Row indices are stable
    /// across collapses; deleted faces are removed from this map and their
    /// rows in `matrix` are soft-cleared (left as the zero row).
    face_to_row: HashMap<[VIdx; 3], usize>,
    /// Sorted edge pair -> column index in `matrix`. Column indices are
    /// stable across collapses; merged columns are dropped from this map
    /// and become the zero column in `matrix`.
    edge_to_col: HashMap<[VIdx; 2], u32>,
    /// Handle-subspace generators: `g` 1-chains over the current edge
    /// basis (sorted column indices, exactly like a `matrix` row). Each
    /// generator is the chain-map image of one of the precomputed handle
    /// cycles `c_i ∈ K = ker(H₁(S) → H₁(V))`. Used by the cost bias to
    /// drive greedy toward handle-killing collapses, and as the extra
    /// rows that augment `∂₂` when computing `aug_pivots` below. Empty
    /// for genus 0.
    handle_cycles: Vec<Vec<u32>>,
    /// Cached RREF of `[∂₂; handle_cycles]`. Its row span is `B₁(X) + K`,
    /// so reducing a 1-chain against [`Self::aug_pivot_map`] returns the
    /// zero residue iff the chain is null-homologous *modulo handles* —
    /// i.e. iff it has no tunnel content. Used by the legality check
    /// [`SurgeryContext::check_link_condition`] on every candidate, so
    /// it must be refreshed by [`Self::rebuild_pivots`] after every
    /// mutation to `matrix` or `handle_cycles`.
    aug_pivots: F2Matrix,
    /// Pivot map of [`Self::aug_pivots`].
    aug_pivot_map: HashMap<u32, usize>,
}

impl BoundaryMatrix {
    /// Refreshes [`Self::aug_pivots`] and [`Self::aug_pivot_map`] from the
    /// current `matrix` and `handle_cycles`. Must be called after every
    /// mutation that could have invalidated the previous RREF — i.e. after
    /// every accepted collapse and every free-face collapse.
    ///
    /// Construction: take the live `∂₂` rows and append each handle cycle
    /// as an extra row, then reduce. The resulting pivot set spans
    /// `B₁(X) + K`. The live `matrix` is left untouched so its rows still
    /// correspond to face row indices via `face_to_row`.
    fn rebuild_pivots(&mut self) {
        let mut combined: Vec<Vec<u32>> = self.matrix.rows().to_vec();
        combined.extend(self.handle_cycles.iter().cloned());
        let mut m = F2Matrix::from_rows(combined);
        m.reduce();
        self.aug_pivot_map = m.pivot_map();
        self.aug_pivots = m;
    }
}

/// The set of changes a candidate collapse u → v would make to the
/// boundary-matrix state. Computed once per legality check and re-used to
/// either probe (on a snapshot) or commit (on the live matrix).
struct CollapseDelta {
    /// Faces incident to edge (u, v): rows to clear; map entries to drop.
    faces_removed: Vec<[VIdx; 3]>,
    /// Faces (u, x, y) where (v, x, y) already exists: row of the (u, x, y)
    /// face is cleared (after column merging the two are identical), and the
    /// (u, x, y) entry is dropped from face_to_row. The (v, x, y) row stays.
    faces_merged: Vec<([VIdx; 3], [VIdx; 3])>,
    /// Faces (u, x, y) with no corresponding (v, x, y): row is unchanged in
    /// content, only its key in face_to_row needs to be relabeled.
    faces_renamed: Vec<([VIdx; 3], [VIdx; 3])>,
    /// Column-merge ops to apply: `(keep, drop)`. `keep` survives in
    /// edge_to_col; `drop` becomes the zero column.
    edge_col_merges: Vec<(u32, u32)>,
    /// Edges (u, w) for non-shared neighbors w: their column survives but
    /// the key in edge_to_col is renamed from (u, w) to (v, w).
    edge_renames: Vec<([VIdx; 2], [VIdx; 2])>,
    /// The (u, v) edge itself: its column is implicitly emptied (every
    /// incident face is in `faces_removed`) and its key dropped from
    /// edge_to_col.
    removed_uv_edge: [VIdx; 2],
    delta_v: usize,
    delta_e: usize,
    delta_f: usize,
}

/// Sorts two vertex keys into canonical order for edge identity.
#[inline]
fn sort_edge(a: VIdx, b: VIdx) -> [VIdx; 2] {
    if a < b { [a, b] } else { [b, a] }
}

/// Symmetric difference of two sorted-distinct `u32` slices, returned as a
/// fresh sorted `Vec<u32>` — the GF(2) row-XOR primitive used when reducing
/// a handle-cycle residue against a pivot row.
fn xor_sorted(a: &[u32], b: &[u32]) -> Vec<u32> {
    let mut out = Vec::with_capacity(a.len() + b.len());
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        match a[i].cmp(&b[j]) {
            Ordering::Less => { out.push(a[i]); i += 1; }
            Ordering::Greater => { out.push(b[j]); j += 1; }
            Ordering::Equal => { i += 1; j += 1; }
        }
    }
    out.extend_from_slice(&a[i..]);
    out.extend_from_slice(&b[j..]);
    out
}

/// In-place column-merge on a single sorted GF(2) chain. Same operation
/// as `F2Matrix::merge_columns`, but for one chain — used to evolve handle
/// cycles through edge collapses. As with `merge_columns`, the caller's
/// `(keep, drop)` is honoured exactly: `drop` is XOR'd into `keep`, and
/// the `drop` column is removed from the chain.
fn merge_columns_in_chain(chain: &mut Vec<u32>, keep: u32, drop: u32) {
    debug_assert!(keep != drop);
    if let Ok(drop_pos) = chain.binary_search(&drop) {
        chain.remove(drop_pos);
        match chain.binary_search(&keep) {
            Ok(kp) => { chain.remove(kp); }
            Err(ip) => { chain.insert(ip, keep); }
        }
    }
}

/// Toggles `col` in a sorted GF(2) 1-chain: if present, remove; if
/// absent, insert. Used by the free-face collapse's chain map, which on
/// 1-chains sends the free edge to the sum of the face's other two
/// edges — i.e. each "other" edge's coefficient gets XOR'd in.
fn toggle_col_in_chain(chain: &mut Vec<u32>, col: u32) {
    match chain.binary_search(&col) {
        Ok(pos) => { chain.remove(pos); }
        Err(pos) => { chain.insert(pos, col); }
    }
}

/// Ephemeral state used during the surgery process.
///
/// We maintain a "virtual" adjacency structure separate from the DCEL mesh
/// to perform edge collapses without modifying the original mesh structure.
/// (This was simpler to implement than implementing structural modifications there).
struct SurgeryContext {
    /// Normalized vertex positions for numerical stability.
    positions: HashMap<VIdx, Vector3D>,

    /// Stored normalization factors to restore original positions later.
    center: Vector3D,
    scale: f64,

    /// Mutable adjacency structure (using HashSet for efficient lookup).
    neighbors: HashMap<VIdx, HashSet<VIdx>>,

    /// Active faces tracker for topology checks.
    /// Each face is stored as a sorted triple of vertex keys.
    active_faces: HashSet<[VIdx; 3]>,

    /// Quadric Error Matrices for shape cost (Eq 5 in paper).
    quadrics: HashMap<VIdx, Matrix4<f64>>,

    /// Flags for deleted vertices.
    is_dead: HashSet<VIdx>,

    /// Maps each skeleton vertex to the list of original mesh vertex keys.
    /// Initially each vertex maps to itself, then accumulates as vertices merge.
    vertex_to_original: HashMap<VIdx, Vec<VertKey<INPUT>>>,

    /// Genus of the closed orientable input surface. The legality check
    /// requires `β₁(X) − dim K(X) = g` throughout surgery — i.e. the
    /// quotient `H₁(X) / K(X) ≅ H₁(V)` keeps full rank.
    target_g: usize,

    /// Boundary matrix ∂₂ tracked across collapses, plus the handle-subspace
    /// generators and the surface 2-cycle chain. Always populated for a
    /// valid closed orientable input.
    boundary_matrix: BoundaryMatrix,
}

impl SurgeryContext {
    /// Creates a new SurgeryContext from a contracted mesh.
    ///
    /// The `original_mesh` is used solely to compute the handle subspace
    /// `K ⊆ H₁(S; ℤ/2)` once via TetGen — the contracted mesh shares its
    /// topology and vertex indexing (raw `DefaultKey` values match), so the
    /// computed handle cycles translate directly into surgery's edge basis.
    ///
    /// Returns `None` (after logging an error) when the input mesh fails the
    /// preprocessing checks: not a closed connected 2-manifold, or the
    /// derived genus isn't a non-negative integer. Callers should treat
    /// `None` as "produce an empty skeleton".
    fn new(mesh: &Mesh<CONTRACTION>, original_mesh: &Mesh<INPUT>) -> Option<Self> {
        let vert_ids = mesh.vert_ids();

        // Compute bounding box for normalization. Normalization is necessary for
        // the costs to balance properly.
        let mut min = Vector3D::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max = Vector3D::new(f64::MIN, f64::MIN, f64::MIN);
        for &v in &vert_ids {
            let p = mesh.position(v);
            min = Vector3D::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
            max = Vector3D::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        }

        let center = (min + max) * 0.5;
        let max_dim = (max - min).max();
        let scale = if max_dim > 1e-8 { 1.0 / max_dim } else { 1.0 };

        // Store normalized positions
        let mut positions = HashMap::new();
        for &v in &vert_ids {
            let p = mesh.position(v);
            let normalized = (p - center) * scale;
            positions.insert(v, normalized);
        }

        // Build adjacency sets from mesh
        let mut neighbors: HashMap<VIdx, HashSet<VIdx>> = HashMap::new();
        for &v in &vert_ids {
            let neighbor_set: HashSet<_> = mesh.neighbors(v).collect();
            neighbors.insert(v, neighbor_set);
        }

        // Build active faces set
        let mut active_faces = HashSet::new();
        for face_id in mesh.face_ids() {
            let verts: Vec<_> = mesh.vertices(face_id).collect();
            if verts.len() == 3 {
                active_faces.insert(sort_face(verts[0], verts[1], verts[2]));
            } else {
                // For non-triangular faces, triangulate using fan
                for i in 1..verts.len() - 1 {
                    active_faces.insert(sort_face(verts[0], verts[i], verts[i + 1]));
                }
            }
        }

        // Compute initial quadrics from edges
        let mut quadrics: HashMap<VIdx, Matrix4<f64>> = HashMap::new();
        for &v in &vert_ids {
            quadrics.insert(v, Matrix4::zeros());
        }

        for &u in &vert_ids {
            for &v in &neighbors[&u] {
                if u < v {
                    let p_u = positions[&u];
                    let p_v = positions[&v];
                    let edge_vec = p_v - p_u;
                    let len = edge_vec.norm();
                    if len < 1e-12 {
                        continue;
                    }
                    let edge_vec = edge_vec / len;

                    let q_edge = compute_edge_quadric(p_u, edge_vec);

                    *quadrics.get_mut(&u).unwrap() += q_edge;
                    *quadrics.get_mut(&v).unwrap() += q_edge;
                }
            }
        }

        // Initialize vertex_to_original: each vertex maps to itself (converted to INPUT key)
        let mut vertex_to_original = HashMap::new();
        for &v in &vert_ids {
            // Convert CONTRACTION key to INPUT key (same raw value)
            let input_key = VertKey::<INPUT>::new(v.raw());
            vertex_to_original.insert(v, vec![input_key]);
        }

        // Topology preprocessing: verify the simplicial complex is a closed
        // connected 2-manifold, derive genus, and build the boundary matrix
        // ∂₂ over GF(2). When genus > 0, also compute a basis of the handle
        // subspace K = ker(H₁(S) → H₁(V)) via tetrahedralization of the
        // input surface, and attach those cycles to the boundary matrix.
        // At construction time no vertex has been collapsed yet.
        let is_dead: HashSet<VIdx> = HashSet::new();
        let (target_g, mut boundary_matrix) =
            preprocess_topology(&active_faces, &neighbors, &is_dead, vert_ids.len())?;

        if target_g > 0 {
            attach_handle_cycles(&mut boundary_matrix, original_mesh, target_g)?;
        }

        Some(Self {
            positions,
            center,
            scale,
            neighbors,
            active_faces,
            quadrics,
            is_dead,
            vertex_to_original,
            target_g,
            boundary_matrix,
        })
    }

    /// Computes the collapse cost for edge u -> v (Eq 8 in paper).
    fn compute_collapse_cost(&self, u: VIdx, v: VIdx) -> f64 {
        const WA: f64 = 1.0; // Shape weight
        const WB: f64 = 0.1; // Sampling weight
        // Handle-cycle bonus: a very negative cost added per cycle this
        // edge participates in. Pulls handle-killing collapses to the
        // front of the heap so greedy whittles the handle down before
        // exhausting the easy non-handle candidates and getting stuck at
        // the manifold floor. Provably homotopy-safe: bias only affects
        // ordering, the legality criterion still gates whether the
        // collapse is allowed.
        const W_HANDLE_BONUS: f64 = 1.0e6;

        // Shape Cost: v^T * (Q_u + Q_v) * v
        let p_v = self.positions[&v];
        let p_hom = Vector4::new(p_v.x, p_v.y, p_v.z, 1.0);

        let q_sum = self.quadrics[&u] + self.quadrics[&v];
        let shape_cost = p_hom.dot(&(q_sum * p_hom));

        // Sampling Cost: sum of squared distances to new edges
        let mut sampling_cost = 0.0;
        for &n in &self.neighbors[&u] {
            if n != v {
                let dist_sq = (self.positions[&v] - self.positions[&n]).norm_squared();
                sampling_cost += dist_sq;
            }
        }

        // Handle bonus: subtract one full bonus per handle cycle that
        // currently includes the column for edge (u, v).
        let mut handle_bonus: f64 = 0.0;
        if let Some(&col) = self.boundary_matrix.edge_to_col.get(&sort_edge(u, v)) {
            for cycle in &self.boundary_matrix.handle_cycles {
                if cycle.binary_search(&col).is_ok() {
                    handle_bonus += W_HANDLE_BONUS;
                }
            }
        }

        WA * shape_cost + WB * sampling_cost - handle_bonus
    }

    /// Checks if an edge (u, v) is part of any active face.
    /// Collapsing edges with no faces would destroy the skeleton.
    fn edge_has_faces(&self, u: VIdx, v: VIdx) -> bool {
        let n_u = &self.neighbors[&u];
        let n_v = &self.neighbors[&v];

        // Iterate the smaller set for efficiency
        let (smaller, larger) = if n_u.len() < n_v.len() {
            (n_u, n_v)
        } else {
            (n_v, n_u)
        };

        for &w in smaller {
            if w == u || w == v {
                continue;
            }

            // If w is a shared neighbor, check if triangle (u, v, w) is active
            if larger.contains(&w) {
                if self.active_faces.contains(&sort_face(u, v, w)) {
                    return true;
                }
            }
        }
        false
    }

    /// Classical Dey–Edelsbrunner manifold-preservation check (every
    /// shared neighbour `w` of `u` and `v` must form a face `(u, v, w)`).
    /// **Not** in the legality gate any more — the new criterion permits
    /// non-manifold collapses that kill handle cycles — but kept around so
    /// the existing tests and diagnostics can ask "would this collapse have
    /// preserved manifoldness?".
    #[allow(dead_code)]
    fn check_manifold_preserved(&self, u: VIdx, v: VIdx) -> bool {
        let n_u = &self.neighbors[&u];
        let n_v = &self.neighbors[&v];
        for &w in n_u {
            if w == v {
                continue;
            }
            if n_v.contains(&w) && !self.active_faces.contains(&sort_face(u, v, w)) {
                return false;
            }
        }
        true
    }

    /// Homology-aware legality check, structurally mirroring the original
    /// Dey–Edelsbrunner edge-link condition.
    ///
    /// For each shared neighbour `w` of `u` and `v`, the three edges
    /// `(u,v)`, `(v,w)`, `(u,w)` form a 1-cycle `γ_w` in the current
    /// 1-skeleton. The edge-collapse chain map `q_#` sends `γ_w` to zero
    /// (the (u,v) edge is killed; the (u,w) and (v,w) columns merge),
    /// so the homology class `[γ_w] ∈ H₁(X)` is exactly the class the
    /// collapse would kill at this corner. In fact every class killed
    /// by the collapse is a sum of such `[γ_w]`, so this set of probes
    /// is complete (see `HOMOLOGY_PRESERVING_SURGERY.md`).
    ///
    /// Three cases per `w`:
    /// - **Face `(u,v,w)` exists.** Then `γ_w = ∂(u,v,w) ∈ B₁(X)`, so
    ///   `[γ_w] = 0` — nothing is being killed. This recovers the strict
    ///   link condition's accept case.
    /// - **`[γ_w]` is a pure handle class** (lies in `K`). The collapse
    ///   kills only a handle generator; that's *desired* — handles need
    ///   to die for surgery to reach a 1-skeleton on positive-genus
    ///   inputs. Accept.
    /// - **`[γ_w]` has tunnel content** (i.e. its image in `H₁(V)` is
    ///   non-zero). The collapse would destroy an essential cycle of the
    ///   underlying solid. Reject.
    ///
    /// Test: reduce `γ_w` against `aug_pivot_map`, the cached pivots of
    /// `B₁(X) + K`. Zero residue ⇒ no tunnel content (accept this `w`);
    /// non-zero residue ⇒ tunnel content (reject the whole collapse).
    fn check_link_condition(&self, u: VIdx, v: VIdx) -> bool {
        let bm = &self.boundary_matrix;
        let uv_col = match bm.edge_to_col.get(&sort_edge(u, v)) {
            Some(&c) => c,
            None => return true, // no live (u,v) edge — nothing to probe
        };
        let n_u = &self.neighbors[&u];
        let n_v = &self.neighbors[&v];

        for &w in n_u {
            if w == v {
                continue;
            }
            if n_v.contains(&w) {
                // Fast path — strict Dey–Edelsbrunner link condition at
                // this corner: if face `(u,v,w)` exists, then
                // `γ_w = ∂(u,v,w) ∈ B₁(X)` is already a boundary, so
                // `[γ_w] = 0` has no tunnel content. Skip the reduction
                // and move on to the next shared neighbour.
                if self.active_faces.contains(&sort_face(u, v, w)) {
                    continue;
                }
                // Slow path: build `γ_w` and test for tunnel content via
                // reduction against `B₁(X) + K`.
                let vw_col = bm.edge_to_col[&sort_edge(v, w)];
                let uw_col = bm.edge_to_col[&sort_edge(u, w)];
                let mut residue = [uv_col, vw_col, uw_col];
                residue.sort();
                let mut residue = residue.to_vec();
                if !bm
                    .aug_pivots
                    .reduce_against(&bm.aug_pivot_map, &mut residue)
                {
                    return false;
                }
            }
        }
        true
    }

    /// Diagnostic-only: returns post-collapse `(β₁, dim K, β₂)` so the
    /// stuck-state and per-edge dumps can report what each rejected
    /// collapse would have done. Not used by the legality criterion
    /// (which is the cheap local check `check_link_condition`).
    fn simulate_post_collapse_stats(
        &self,
        u: VIdx,
        v: VIdx,
    ) -> Option<(i64, i64, i64)> {
        let bm = &self.boundary_matrix;
        let delta = self.compute_collapse_delta(u, v)?;

        let mut snap = bm.clone();
        Self::apply_collapse_delta(&mut snap, &delta);
        snap.matrix.reduce();
        let new_rank = snap.matrix.rank();

        let v_now = (self.positions.len() - self.is_dead.len()) as i64;
        let e_now = bm.edge_to_col.len() as i64;
        let new_v = v_now - delta.delta_v as i64;
        let new_e = e_now - delta.delta_e as i64;
        let new_beta_1 = new_e - new_v + 1 - new_rank as i64;

        let pivot_map = snap.matrix.pivot_map();

        let mut new_dim_k: i64 = 0;
        for cycle in &snap.handle_cycles {
            let mut residue = cycle.clone();
            if !snap.matrix.reduce_against(&pivot_map, &mut residue) {
                new_dim_k += 1;
            }
        }

        let new_f = (self.active_faces.len() as i64) - delta.delta_f as i64;
        let new_beta_2 = new_f - new_rank as i64;

        Some((new_beta_1, new_dim_k, new_beta_2))
    }

    /// Computes what the (face_to_row, edge_to_col, matrix) state would
    /// change to under a hypothetical collapse u → v. Returns `None` only
    /// if u and v aren't both adjacent in the current `neighbors` map
    /// (which `is_legal_collapse_candidate` gates against anyway).
    fn compute_collapse_delta(&self, u: VIdx, v: VIdx) -> Option<CollapseDelta> {
        let bm = &self.boundary_matrix;
        let n_u = &self.neighbors[&u];
        let n_v = &self.neighbors[&v];

        // Shared neighbors of u and v (excluding u, v themselves).
        let mut shared: Vec<VIdx> = Vec::new();
        for &w in n_u {
            if w != v && w != u && n_v.contains(&w) {
                shared.push(w);
            }
        }

        // Faces incident to edge (u, v) — one per shared neighbor that
        // currently sits as an active face.
        let mut faces_removed = Vec::new();
        for &w in &shared {
            let f = sort_face(u, v, w);
            if self.active_faces.contains(&f) {
                faces_removed.push(f);
            }
        }

        // Faces (u, n1, n2) with n1 ≠ v and n2 ≠ v: either rename to
        // (v, n1, n2), or merge with an existing (v, n1, n2) face.
        let mut faces_merged = Vec::new();
        let mut faces_renamed = Vec::new();
        let neighbors_u: Vec<VIdx> = n_u.iter().copied().collect();
        for i in 0..neighbors_u.len() {
            for j in (i + 1)..neighbors_u.len() {
                let n1 = neighbors_u[i];
                let n2 = neighbors_u[j];
                if n1 == v || n2 == v {
                    continue;
                }
                let face_old = sort_face(u, n1, n2);
                if !self.active_faces.contains(&face_old) {
                    continue;
                }
                let face_new = sort_face(v, n1, n2);
                if self.active_faces.contains(&face_new) {
                    faces_merged.push((face_old, face_new));
                } else {
                    faces_renamed.push((face_old, face_new));
                }
            }
        }

        // Per-shared-neighbor column merge: keep the (v, w) column, drop
        // the (u, w) column.
        let mut edge_col_merges = Vec::with_capacity(shared.len());
        for &w in &shared {
            let col_keep = bm.edge_to_col[&sort_edge(v, w)];
            let col_drop = bm.edge_to_col[&sort_edge(u, w)];
            edge_col_merges.push((col_keep, col_drop));
        }

        // Edges (u, w) for non-shared w: same column, new key.
        let mut edge_renames = Vec::new();
        for &w in n_u {
            if w == v || w == u {
                continue;
            }
            if !n_v.contains(&w) {
                edge_renames.push((sort_edge(u, w), sort_edge(v, w)));
            }
        }

        let delta_v = 1usize;
        let delta_e = 1 + shared.len(); // (u, v) + one duplicate per shared
        let delta_f = faces_removed.len() + faces_merged.len();

        Some(CollapseDelta {
            faces_removed,
            faces_merged,
            faces_renamed,
            edge_col_merges,
            edge_renames,
            removed_uv_edge: sort_edge(u, v),
            delta_v,
            delta_e,
            delta_f,
        })
    }

    /// Applies a [`CollapseDelta`] in place to the given boundary matrix,
    /// its face/edge maps, and the tracked handle cycles. Used both for
    /// the snapshot in [`Self::simulate_post_collapse_stats`] and for the
    /// live commit in [`Self::collapse_edge`]. The caller is responsible
    /// for refreshing [`BoundaryMatrix::aug_pivots`] afterwards.
    fn apply_collapse_delta(bm: &mut BoundaryMatrix, delta: &CollapseDelta) {
        // 0. Look up the (u, v) column BEFORE we mutate edge_to_col, so we
        //    can scrub it out of every tracked 1-chain below.
        let removed_uv_col = bm.edge_to_col.get(&delta.removed_uv_edge).copied();

        // 1a. Evolve each tracked handle cycle by the edge-collapse chain
        //     map. On 1-chains, the chain map for an edge collapse u → v
        //     is: (a) merge the columns of every shared-neighbour edge
        //     pair (same merge_columns op we do on matrix rows), and
        //     (b) drop the (u, v) column itself.
        for cycle in &mut bm.handle_cycles {
            for &(keep, drop) in &delta.edge_col_merges {
                merge_columns_in_chain(cycle, keep, drop);
            }
            if let Some(col) = removed_uv_col {
                if let Ok(pos) = cycle.binary_search(&col) {
                    cycle.remove(pos);
                }
            }
        }

        // 1b. Column merges on the matrix: shared-neighbour edge columns
        //     collapse into one.
        for &(keep, drop) in &delta.edge_col_merges {
            bm.matrix.merge_columns(keep, drop);
        }

        // 2. Clear rows of removed faces (incident to the (u, v) edge).
        for face in &delta.faces_removed {
            if let Some(&row) = bm.face_to_row.get(face) {
                bm.matrix.clear_row(row);
            }
            bm.face_to_row.remove(face);
        }

        // 3. Clear rows of merged faces; keep the partner row in place.
        for (old, _new_kept) in &delta.faces_merged {
            if let Some(&row) = bm.face_to_row.get(old) {
                bm.matrix.clear_row(row);
            }
            bm.face_to_row.remove(old);
        }

        // 4. Rename face entries: same row, new key.
        for (old, new) in &delta.faces_renamed {
            if let Some(row) = bm.face_to_row.remove(old) {
                bm.face_to_row.insert(*new, row);
            }
        }

        // 5. Rename edge entries: same column, new key. (Pre-collapse the
        //    new key cannot already be present — that would mean w is a
        //    shared neighbor, contradicting the rename branch.)
        for (old, new) in &delta.edge_renames {
            if let Some(col) = bm.edge_to_col.remove(old) {
                bm.edge_to_col.insert(*new, col);
            }
        }

        // 6. Drop edge_to_col entries pointing at columns that got merged
        //    away (the (u, w) keys for shared neighbors). The (v, w) keys
        //    already point to the kept column.
        if !delta.edge_col_merges.is_empty() {
            let dropped: HashSet<u32> = delta
                .edge_col_merges
                .iter()
                .map(|&(_, d)| d)
                .collect();
            bm.edge_to_col.retain(|_, col| !dropped.contains(col));
        }

        // 7. Drop the (u, v) edge entry. Its column is now the zero column
        //    because every face incident to it was cleared in step 2.
        bm.edge_to_col.remove(&delta.removed_uv_edge);
    }

    /// Returns whether directed collapse u -> v is currently legal.
    fn is_legal_collapse_candidate(&self, u: VIdx, v: VIdx) -> bool {
        if self.is_dead.contains(&u) || self.is_dead.contains(&v) {
            return false;
        }

        if !self
            .neighbors
            .get(&u)
            .is_some_and(|neighbor_set| neighbor_set.contains(&v))
        {
            return false;
        }

        if !self.edge_has_faces(u, v) {
            return false;
        }

        // Homology-aware link condition: for each shared neighbour `w`
        // of `u` and `v`, the 3-cycle `γ_w` (the corner triangle in the
        // 1-skeleton) is what the collapse would kill in `H₁`. Reject
        // iff any such `γ_w` has tunnel content. Strictly manifold
        // collapses are *not* required: non-manifold collapses and
        // bubble creation are both fine as long as no essential cycle
        // of the underlying solid dies.
        if !self.check_link_condition(u, v) {
            return false;
        }

        true
    }

    /// Counts all currently legal directed collapse candidates.
    fn count_legal_collapse_candidates(&self) -> usize {
        let mut count = 0;

        for (&u, neighbor_set) in &self.neighbors {
            if self.is_dead.contains(&u) {
                continue;
            }

            for &v in neighbor_set {
                if self.is_legal_collapse_candidate(u, v) {
                    count += 1;
                }
            }
        }

        count
    }

    /// Logs a full diagnostic of the stuck state: aggregate counts, the
    /// connected-component structure of the sub-complex still carrying
    /// faces, and a per-directed-edge β₁ probe for every remaining face.
    ///
    /// Intended to fire once when surgery exits with `active_faces`
    /// non-empty. The point is to make it obvious whether we're stuck on a
    /// single irreducible "torus core" (a topological obstruction) or on
    /// scattered pathologies (a likely bug).
    fn dump_stuck_state_diagnostic(&self) {
        let v_now = self.positions.len() - self.is_dead.len();
        let f_now = self.active_faces.len();
        let e_now = self.boundary_matrix.edge_to_col.len();
        let chi = v_now as i64 - e_now as i64 + f_now as i64;
        let n_handle_cycles = self.boundary_matrix.handle_cycles.len();
        warn!(
            "Stuck-state aggregate: V={}, E={}, F={}, χ={}, target_g={}, live_handle_cycles={}.",
            v_now, e_now, f_now, chi, self.target_g, n_handle_cycles
        );

        // Vertices that still belong to at least one active face form the
        // "live sub-complex". Compute its connected components by BFS over
        // the adjacency restricted to faces.
        let mut face_verts: HashSet<VIdx> = HashSet::new();
        for &[a, b, c] in &self.active_faces {
            face_verts.insert(a);
            face_verts.insert(b);
            face_verts.insert(c);
        }
        let mut adj: HashMap<VIdx, HashSet<VIdx>> = HashMap::new();
        for &[a, b, c] in &self.active_faces {
            adj.entry(a).or_default().extend([b, c]);
            adj.entry(b).or_default().extend([a, c]);
            adj.entry(c).or_default().extend([a, b]);
        }
        let mut visited: HashSet<VIdx> = HashSet::new();
        let mut components: Vec<(usize, usize)> = Vec::new(); // (verts, faces)
        for &start in &face_verts {
            if visited.contains(&start) {
                continue;
            }
            let mut comp_verts: HashSet<VIdx> = HashSet::new();
            let mut queue: VecDeque<VIdx> = VecDeque::new();
            queue.push_back(start);
            visited.insert(start);
            comp_verts.insert(start);
            while let Some(v) = queue.pop_front() {
                if let Some(ns) = adj.get(&v) {
                    for &n in ns {
                        if visited.insert(n) {
                            comp_verts.insert(n);
                            queue.push_back(n);
                        }
                    }
                }
            }
            let comp_face_count = self
                .active_faces
                .iter()
                .filter(|&&[a, b, c]| {
                    comp_verts.contains(&a)
                        && comp_verts.contains(&b)
                        && comp_verts.contains(&c)
                })
                .count();
            components.push((comp_verts.len(), comp_face_count));
        }
        warn!(
            "Stuck sub-complex has {} connected component(s): {:?} (V, F per component).",
            components.len(),
            components
        );

        // Handle-cycle dump: for each tracked handle generator, report
        // - total edges,
        // - how many of those edges currently appear in the boundary of
        //   some active face (= face-incident, i.e. the edge could become
        //   a collapse candidate),
        // - the actual vertex pairs of the cycle's edges, so the user can
        //   correlate with the geometric view.
        let bm = &self.boundary_matrix;
        let mut face_incident_cols: HashSet<u32> = HashSet::new();
        for face in &self.active_faces {
            if let Some(&row_idx) = bm.face_to_row.get(face) {
                for &col in &bm.matrix.rows()[row_idx] {
                    face_incident_cols.insert(col);
                }
            }
        }
        let col_to_edge: HashMap<u32, [VIdx; 2]> =
            bm.edge_to_col.iter().map(|(&k, &v)| (v, k)).collect();

        for (i, cycle) in bm.handle_cycles.iter().enumerate() {
            let total = cycle.len();
            let face_incident = cycle
                .iter()
                .filter(|col| face_incident_cols.contains(col))
                .count();
            let orphaned = total - face_incident;
            warn!(
                "Handle cycle {}: {} edges total, {} face-incident (collapse-candidatable), {} orphaned (no incident face).",
                i, total, face_incident, orphaned
            );
            // Per-edge dump: vertex pair + whether it's face-incident.
            for &col in cycle {
                let endpoints = col_to_edge.get(&col).copied();
                let endpoints_str = match endpoints {
                    Some([a, b]) => format!("{:?}-{:?}", a, b),
                    None => "<edge no longer in edge_to_col>".to_string(),
                };
                let where_ = if face_incident_cols.contains(&col) { "face-incident" } else { "orphaned" };
                warn!("    col {} = {} [{}]", col, endpoints_str, where_);
            }
        }

        // Per-face dump. With ~10–20 stuck faces this is ~60–120 lines —
        // verbose, but the whole point of this hook is to let us read every
        // rejection.
        warn!(
            "Per-face per-directed-edge diagnostics ({} faces × 6 edges):",
            f_now
        );
        for (i, &face) in self.active_faces.iter().enumerate() {
            let [a, b, c] = face;
            warn!("  face {} {:?}:", i, face);
            for &(u, v) in &[(a, b), (b, a), (b, c), (c, b), (a, c), (c, a)] {
                warn!("    {}", self.edge_diagnostic_summary(u, v));
            }
        }
    }

    /// Returns a one-line diagnostic summary for directed edge u -> v.
    fn edge_diagnostic_summary(&self, u: VIdx, v: VIdx) -> String {
        let u_dead = self.is_dead.contains(&u);
        let v_dead = self.is_dead.contains(&v);
        let adjacent = self
            .neighbors
            .get(&u)
            .is_some_and(|neighbor_set| neighbor_set.contains(&v));
        let has_faces = adjacent && !u_dead && !v_dead && self.edge_has_faces(u, v);
        let stats = if has_faces {
            self.simulate_post_collapse_stats(u, v)
        } else {
            None
        };
        let link_ok = has_faces && self.check_link_condition(u, v);

        let stats_str = match stats {
            Some((b, k, b2)) => format!(
                "new_β₁={}, new_dim_K={}, new_β₂={} (target g={})",
                b, k, b2, self.target_g,
            ),
            None => "post-collapse stats unavailable".to_string(),
        };
        format!(
            "{:?}->{:?}[dead u/v={}/{}, adj={}, faces={}, link_ok={}, {}]",
            u, v, u_dead, v_dead, adjacent, has_faces, link_ok, stats_str
        )
    }

    /// Performs edge collapse: u merges into v.
    /// Attempts a single **free-face elementary collapse** if one is
    /// available. A face is "free" when at least one of its three edges
    /// is incident only to that face (no other active face has it). The
    /// collapse removes the face and that free edge, which is a textbook
    /// CW elementary collapse: a deformation retraction, so β₁, dim K,
    /// and β₂ are all preserved exactly. The legality criterion can't
    /// reject this operation, so we don't check it.
    ///
    /// Returns `true` if a collapse was performed. Caller is expected to
    /// loop until this returns `false`, after which no free faces remain.
    fn try_free_face_collapse(&mut self) -> bool {
        // Count how many active faces each edge is incident to.
        let mut edge_face_count: HashMap<[VIdx; 2], usize> = HashMap::new();
        for face in &self.active_faces {
            let [a, b, c] = *face;
            *edge_face_count.entry(sort_edge(a, b)).or_insert(0) += 1;
            *edge_face_count.entry(sort_edge(b, c)).or_insert(0) += 1;
            *edge_face_count.entry(sort_edge(a, c)).or_insert(0) += 1;
        }

        // Look for a face that owns at least one count-1 edge.
        let faces_snapshot: Vec<[VIdx; 3]> = self.active_faces.iter().copied().collect();
        for face in faces_snapshot {
            let [a, b, c] = face;
            for &(x, y) in &[(a, b), (b, c), (a, c)] {
                let edge = sort_edge(x, y);
                if edge_face_count.get(&edge).copied().unwrap_or(0) == 1 {
                    self.do_free_face_collapse(face, edge);
                    return true;
                }
            }
        }
        false
    }

    /// Performs a free-face elementary collapse: remove `face` and the
    /// `free_edge` (which must currently be incident only to `face`).
    /// Updates the boundary matrix, handle cycles, surface 2-chain, and
    /// the live face/edge/neighbor maps.
    fn do_free_face_collapse(&mut self, face: [VIdx; 3], free_edge: [VIdx; 2]) {
        let bm = &mut self.boundary_matrix;
        let face_row = bm.face_to_row.get(&face).copied();
        let free_col = bm.edge_to_col.get(&free_edge).copied();

        // The face's two NON-free edges, in canonical sorted form.
        let [a, b, c] = face;
        let mut other_cols: [Option<u32>; 2] = [None, None];
        let mut next = 0;
        for &(x, y) in &[(a, b), (b, c), (a, c)] {
            let e = sort_edge(x, y);
            if e == free_edge {
                continue;
            }
            other_cols[next] = bm.edge_to_col.get(&e).copied();
            next += 1;
        }

        // Evolve handle cycles via the free-face chain map: e_free ↦
        // e_2 + e_3 on 1-chains. For each cycle that contains the free
        // edge's column, remove that column and toggle the other two.
        if let (Some(fc), Some(c2), Some(c3)) = (free_col, other_cols[0], other_cols[1]) {
            for cycle in &mut bm.handle_cycles {
                if let Ok(pos) = cycle.binary_search(&fc) {
                    cycle.remove(pos);
                    toggle_col_in_chain(cycle, c2);
                    toggle_col_in_chain(cycle, c3);
                }
            }
        }

        // Clear the face's matrix row.
        if let Some(row) = face_row {
            bm.matrix.clear_row(row);
        }
        bm.face_to_row.remove(&face);
        bm.edge_to_col.remove(&free_edge);

        // Mirror the active-faces set and the 1-skeleton adjacency.
        self.active_faces.remove(&face);
        let [u, v] = free_edge;
        if let Some(set) = self.neighbors.get_mut(&u) {
            set.remove(&v);
        }
        if let Some(set) = self.neighbors.get_mut(&v) {
            set.remove(&u);
        }

        // The matrix and handle cycles were both mutated; the cached
        // pivot RREF is stale until refreshed.
        self.boundary_matrix.rebuild_pivots();
    }

    fn collapse_edge(&mut self, u: VIdx, v: VIdx) {
        // Compute the boundary-matrix delta against the pre-collapse state.
        // This is `None` for genus-0 inputs where we don't track the matrix.
        let delta = self.compute_collapse_delta(u, v);

        // Merge quadrics
        let q_u = self.quadrics[&u];
        *self.quadrics.get_mut(&v).unwrap() += q_u;

        // Mark u as dead
        self.is_dead.insert(u);

        // Merge original vertex mappings
        let mut originals_u = self.vertex_to_original.remove(&u).unwrap_or_default();
        self.vertex_to_original
            .get_mut(&v)
            .unwrap()
            .append(&mut originals_u);

        // Get u's neighbors before modifying
        let neighbors_u: Vec<_> = self.neighbors[&u].iter().copied().collect();

        // Update faces
        let mut faces_to_remove = Vec::new();
        let mut faces_to_add = Vec::new();

        for i in 0..neighbors_u.len() {
            for j in (i + 1)..neighbors_u.len() {
                let n1 = neighbors_u[i];
                let n2 = neighbors_u[j];
                let face_key = sort_face(u, n1, n2);

                if self.active_faces.contains(&face_key) {
                    faces_to_remove.push(face_key);

                    // If face doesn't involve v, it morphs into (v, n1, n2)
                    if n1 != v && n2 != v {
                        faces_to_add.push(sort_face(v, n1, n2));
                    }
                }
            }
        }

        for f in faces_to_remove {
            self.active_faces.remove(&f);
        }

        for f in faces_to_add {
            self.active_faces.insert(f);
        }

        // Update neighbor sets
        // Remove u from all its neighbors' sets
        for &n in &neighbors_u {
            if let Some(n_set) = self.neighbors.get_mut(&n) {
                n_set.remove(&u);
            }
        }

        // Merge u's neighbors into v's neighbors
        for &n in &neighbors_u {
            if n == v {
                continue;
            }

            // Add n to v's neighbor set
            self.neighbors.get_mut(&v).unwrap().insert(n);

            // Add v to n's neighbor set
            self.neighbors.get_mut(&n).unwrap().insert(v);
        }

        // Clear u's neighbor set
        self.neighbors.get_mut(&u).unwrap().clear();

        // Commit the boundary-matrix delta to the live state and refresh
        // the cached pivot RREF used by `check_link_condition`. Handle
        // cycles are evolved inside `apply_collapse_delta`.
        if let Some(delta) = delta {
            Self::apply_collapse_delta(&mut self.boundary_matrix, &delta);
            self.boundary_matrix.rebuild_pivots();
        }
    }


    /// Builds the final CurveSkeleton graph from the surgery result.
    ///
    /// `compute_boundary_loops`: when true, edges carry the proper face-walk
    /// boundary loop. When false (used for partial / failed surgery results),
    /// edges carry an empty BoundaryLoop placeholder so the graph can be
    /// rendered without panicking on patches that don't share a real
    /// boundary in the original mesh.
    fn to_curve_skeleton(
        &self,
        original_mesh: &Mesh<INPUT>,
        compute_boundary_loops: bool,
    ) -> CurveSkeleton {
        let mut graph = CurveSkeleton::default();
        let mut node_indices = HashMap::new();

        // Add living nodes
        for (&v, &pos) in &self.positions {
            if self.is_dead.contains(&v) {
                continue;
            }

            // Denormalize position to original model space
            let original_pos = pos / self.scale + self.center;

            // Get the list of original vertex keys this node represents
            let originals = self.vertex_to_original.get(&v).cloned().unwrap_or_default();

            let idx = graph.add_node(SkeletonNode {
                position: original_pos,
                patch_vertices: originals,
            });
            node_indices.insert(v, idx);
        }

        // Add edges. When `compute_boundary_loops` is false (surgery failed
        // and we're producing a skeleton purely for inspection), insert
        // empty BoundaryLoops instead of running the face-walk — partial
        // skeletons can have adjacent skeleton nodes whose patches share no
        // boundary face in the original mesh, which would panic
        // `BoundaryLoop::new`.
        for (&u, neighbor_set) in &self.neighbors {
            if self.is_dead.contains(&u) {
                continue;
            }

            if let Some(&u_node) = node_indices.get(&u) {
                for &v in neighbor_set {
                    // Only add each edge once (when u < v)
                    if u < v && !self.is_dead.contains(&v) {
                        if let Some(&v_node) = node_indices.get(&v) {
                            let loop_data = if compute_boundary_loops {
                                let patch_u = graph
                                    .node_weight(u_node)
                                    .unwrap()
                                    .patch_vertices
                                    .clone();
                                let patch_v = graph
                                    .node_weight(v_node)
                                    .unwrap()
                                    .patch_vertices
                                    .clone();
                                BoundaryLoop::new(&patch_u, &patch_v, original_mesh)
                            } else {
                                BoundaryLoop {
                                    edge_midpoints: Vec::new(),
                                }
                            };
                            graph.add_edge(u_node, v_node, loop_data);
                        }
                    }
                }
            }
        }

        graph
    }

    /// Returns world-space triangle positions for every 2-face still in the
    /// simplicial complex when surgery stopped. Used by the failed-surgery
    /// overlay to draw the literal remnant faces in red. Positions are
    /// taken from the contracted, normalized vertex positions and
    /// denormalized to original world space — matching the placement of
    /// skeleton nodes in `to_curve_skeleton`.
    fn remaining_face_triangles(&self) -> Vec<[Vector3D; 3]> {
        self.active_faces
            .iter()
            .map(|&[a, b, c]| {
                [
                    self.positions[&a] / self.scale + self.center,
                    self.positions[&b] / self.scale + self.center,
                    self.positions[&c] / self.scale + self.center,
                ]
            })
            .collect()
    }
}

/// Diagnostic bundle returned by [`extract_skeleton`] when surgery did NOT
/// complete (some faces remained). Carries both the partial curve skeleton
/// (with empty boundary loops, since the patch partition is mangled) and
/// the world-space triangle positions of every face still in the stuck
/// state — so the GUI can render the remnant in red.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FailedSurgeryDiagnostic {
    /// Partial skeleton: a node per surviving vertex, an edge per surviving
    /// 1-cell. Edges carry empty [`BoundaryLoop`] placeholders.
    pub skeleton: CurveSkeleton,
    /// Each entry is a triangle's three world-space corner positions
    /// (post-contraction, denormalized).
    pub remaining_face_positions: Vec<[Vector3D; 3]>,
}

/// Extracts a 1D curve skeleton from a contracted mesh via connectivity surgery.
///
/// # Arguments
/// * `contracted_mesh` - The mesh after contraction (thin but still 2D).
/// * `original_mesh` - The original input mesh (for embedding refinement).
///
/// # Returns
/// `(skeleton, failed_surgery_diagnostic)`:
/// - On success: a fully-collapsed curve skeleton and `None`.
/// - On failure (preprocessing reject, or no candidates left while faces
///   remain): an empty `CurveSkeleton` and `Some(partial_skeleton)` carrying
///   the mangled state for inspection. Returning empty as the primary lets
///   downstream pipeline stages run unguarded — they trivially no-op on an
///   empty graph — while the diagnostic can be rendered as a separate
///   overlay.
pub fn extract_skeleton(
    contracted_mesh: &Mesh<CONTRACTION>,
    original_mesh: &Mesh<INPUT>,
) -> (CurveSkeleton, Option<FailedSurgeryDiagnostic>) {
    let Some(mut ctx) = SurgeryContext::new(contracted_mesh, original_mesh) else {
        // Preprocessing already logged the reason.
        return (CurveSkeleton::default(), None);
    };
    let mut heap = BinaryHeap::new();

    // Initial heap population
    let vert_ids: Vec<_> = ctx.neighbors.keys().copied().collect();
    for u in vert_ids {
        if ctx.is_dead.contains(&u) {
            continue;
        }
        let neighbors: Vec<_> = ctx.neighbors[&u].iter().copied().collect();
        for v in neighbors {
            // Only add edges that have incident faces
            if ctx.edge_has_faces(u, v) {
                let cost = ctx.compute_collapse_cost(u, v);
                heap.push(CollapseCandidate { u, v, cost });
            }
        }
    }

    let mut collapses = 0;

    // Greedy collapse loop
    while !ctx.active_faces.is_empty() {

        let Some(candidate) = heap.pop() else {
            let legal_candidates = ctx.count_legal_collapse_candidates();
            let remaining_faces = ctx.active_faces.len();
            warn!(
                "No more collapse candidates but {} faces remain. Legal directed candidates now: {}.",
                remaining_faces,
                legal_candidates
            );

            if legal_candidates > 0 {
                warn!(
                    "Heap is empty despite legal collapse candidates; this indicates candidate starvation from stale-entry dropping."
                );
            }

            ctx.dump_stuck_state_diagnostic();
            break;
        };

        if !ctx.is_legal_collapse_candidate(candidate.u, candidate.v) {
            continue;
        }

        // Refresh stale entries (cost changed since insertion) instead of dropping them.
        let real_cost = ctx.compute_collapse_cost(candidate.u, candidate.v);
        const COST_EPSILON: f64 = 1e-8;
        if (candidate.cost - real_cost).abs() > COST_EPSILON {
            heap.push(CollapseCandidate {
                u: candidate.u,
                v: candidate.v,
                cost: real_cost,
            });
            continue;
        }

        // Perform the collapse
        ctx.collapse_edge(candidate.u, candidate.v);
        collapses += 1;

        // Re-evaluate edges connected to v
        let neighbors: Vec<_> = ctx.neighbors[&candidate.v].iter().copied().collect();
        for neighbor in neighbors {
            // Evaluate v -> neighbor
            if ctx.edge_has_faces(candidate.v, neighbor) {
                let cost = ctx.compute_collapse_cost(candidate.v, neighbor);
                heap.push(CollapseCandidate {
                    u: candidate.v,
                    v: neighbor,
                    cost,
                });
            }

            // Evaluate neighbor -> v
            if ctx.edge_has_faces(neighbor, candidate.v) {
                let cost = ctx.compute_collapse_cost(neighbor, candidate.v);
                heap.push(CollapseCandidate {
                    u: neighbor,
                    v: candidate.v,
                    cost,
                });
            }
        }
    }

    // Free-face elementary-collapse cleanup pass. Whenever the main
    // surgery loop leaves face-incident leftovers, free-face collapses can
    // clean up disk-like remnants (preserve homotopy exactly: no β₁,
    // dim K, or β₂ change). Doesn't help with closed-sub-complex
    // remnants (e.g. residual torus cores) — those have no free edges.
    let mut free_face_collapses = 0;
    while ctx.try_free_face_collapse() {
        collapses += 1;
        free_face_collapses += 1;
    }

    let succeeded = ctx.active_faces.is_empty();
    if succeeded {
        info!(
            "Connectivity surgery complete. Collapsed {} edges ({} free-face). Remaining vertices: {}",
            collapses,
            free_face_collapses,
            ctx.positions.len() - ctx.is_dead.len()
        );
        let mut skeleton = ctx.to_curve_skeleton(original_mesh, true);
        skeleton.refine_embeddings(original_mesh);
        (skeleton, None)
    } else {
        warn!(
            "Connectivity surgery did NOT complete: {} faces remain after {} collapses. Returning empty skeleton to the pipeline and the partial state as a separate diagnostic.",
            ctx.active_faces.len(),
            collapses
        );
        // Build the partial skeleton with empty BoundaryLoops and no
        // embedding refinement — both steps assume a clean partition into
        // well-formed patches and would panic on the mangled state. Also
        // bundle the world-space triangle positions of every remaining
        // face so the GUI can render the stuck-state remnant.
        let partial_skeleton = ctx.to_curve_skeleton(original_mesh, false);
        let remaining_face_positions = ctx.remaining_face_triangles();
        let diagnostic = FailedSurgeryDiagnostic {
            skeleton: partial_skeleton,
            remaining_face_positions,
        };
        (CurveSkeleton::default(), Some(diagnostic))
    }
}

/// Verifies the input is a closed connected 2-manifold, derives the genus,
/// and builds the GF(2) boundary matrix ∂₂. Returns `(g, BoundaryMatrix)`
/// — the `handle_cycles` field of the matrix is left empty here; callers
/// fill it in afterwards via [`attach_handle_cycles`] when genus > 0.
///
/// On failure (non-manifold edge, disconnected, non-integer genus, sanity
/// check mismatch), logs an error and returns `None`.
fn preprocess_topology(
    active_faces: &HashSet<[VIdx; 3]>,
    neighbors: &HashMap<VIdx, HashSet<VIdx>>,
    is_dead: &HashSet<VIdx>,
    n_verts_total: usize,
) -> Option<(usize, BoundaryMatrix)> {
    // Closed-manifold check: every undirected edge bounds exactly two faces.
    // Building the edge -> face count map also gives us E.
    let mut edge_face_count: HashMap<[VIdx; 2], usize> = HashMap::new();
    for &face in active_faces {
        let [a, b, c] = face;
        *edge_face_count.entry(sort_edge(a, b)).or_insert(0) += 1;
        *edge_face_count.entry(sort_edge(b, c)).or_insert(0) += 1;
        *edge_face_count.entry(sort_edge(a, c)).or_insert(0) += 1;
    }
    for (&edge, &count) in &edge_face_count {
        if count != 2 {
            error!(
                "Connectivity surgery aborted: input mesh is not a closed 2-manifold (edge {:?} is incident to {} faces, expected 2).",
                edge, count
            );
            return None;
        }
    }

    // Connectedness check: BFS over `neighbors` from any alive vertex must
    // reach every alive vertex.
    let alive_count = n_verts_total - is_dead.len();
    if alive_count == 0 {
        error!("Connectivity surgery aborted: input mesh has no vertices.");
        return None;
    }
    let start = *neighbors
        .keys()
        .find(|v| !is_dead.contains(v))
        .expect("non-empty mesh");
    let mut visited: HashSet<VIdx> = HashSet::with_capacity(alive_count);
    let mut queue: VecDeque<VIdx> = VecDeque::new();
    visited.insert(start);
    queue.push_back(start);
    while let Some(v) = queue.pop_front() {
        if let Some(ns) = neighbors.get(&v) {
            for &n in ns {
                if !is_dead.contains(&n) && visited.insert(n) {
                    queue.push_back(n);
                }
            }
        }
    }
    if visited.len() != alive_count {
        error!(
            "Connectivity surgery aborted: input mesh is not connected ({} of {} vertices reachable from start).",
            visited.len(),
            alive_count
        );
        return None;
    }

    // Euler characteristic and genus.
    let v_count = alive_count;
    let e_count = edge_face_count.len();
    let f_count = active_faces.len();
    let chi: i64 = v_count as i64 - e_count as i64 + f_count as i64;
    let two_g: i64 = 2 - chi;
    if two_g < 0 || two_g % 2 != 0 {
        error!(
            "Connectivity surgery aborted: derived genus is not a non-negative integer (V={}, E={}, F={}, χ={}, 2g={}).",
            v_count, e_count, f_count, chi, two_g
        );
        return None;
    }
    let g = (two_g / 2) as usize;

    info!(
        "Connectivity surgery preprocessing: V={}, E={}, F={}, χ={}, genus={}.",
        v_count, e_count, f_count, chi, g
    );

    // Build ∂₂: assign columns to edges, rows to faces.
    let mut edge_to_col: HashMap<[VIdx; 2], u32> = HashMap::with_capacity(e_count);
    for &edge in edge_face_count.keys() {
        let next = edge_to_col.len() as u32;
        edge_to_col.insert(edge, next);
    }

    let mut rows: Vec<Vec<u32>> = Vec::with_capacity(f_count);
    let mut face_to_row: HashMap<[VIdx; 3], usize> = HashMap::with_capacity(f_count);
    for &face in active_faces {
        let [a, b, c] = face;
        let mut row = [
            edge_to_col[&sort_edge(a, b)],
            edge_to_col[&sort_edge(b, c)],
            edge_to_col[&sort_edge(a, c)],
        ];
        row.sort();
        let row_idx = rows.len();
        rows.push(row.to_vec());
        face_to_row.insert(face, row_idx);
    }

    let mut matrix = F2Matrix::from_rows(rows);
    matrix.reduce();
    let rank = matrix.rank();

    // Sanity: β₁ = E − V + 1 − rank(∂₂) should equal 2g for a closed
    // connected orientable 2-manifold.
    let beta_1 = e_count as i64 - v_count as i64 + 1 - rank as i64;
    if beta_1 != two_g {
        error!(
            "Connectivity surgery aborted: β₁ sanity check failed (got {}, expected 2g = {}). Input topology is not what was assumed.",
            beta_1, two_g
        );
        return None;
    }

    let mut bm = BoundaryMatrix {
        matrix,
        face_to_row,
        edge_to_col,
        handle_cycles: Vec::new(),
        aug_pivots: F2Matrix::from_rows(Vec::new()),
        aug_pivot_map: HashMap::new(),
    };
    // Genus-0 path: no handles will be attached later, so this is the
    // final pivot set. For genus > 0, `attach_handle_cycles` will rebuild
    // it again after writing the handle generators.
    bm.rebuild_pivots();
    Some((g, bm))
}

/// Tetrahedralizes `original_mesh`, computes the handle subspace
/// `K = ker(H₁(S; ℤ/2) → H₁(V; ℤ/2))`, and stores a basis of `K` as
/// `handle_cycles` on `bm`, translated into surgery's contraction edge
/// basis (column indices in `bm.matrix`).
///
/// The translation works because the contracted mesh shares vertex raw
/// values with the input mesh (`SurgeryContext::new` initialises
/// `vertex_to_original` via `VertKey::<INPUT>::new(v.raw())`), and the
/// vendored tritet preserves every input surface triangle, so each surface
/// edge is also a contraction-mesh edge with a column entry in
/// `bm.edge_to_col`.
///
/// Returns `None` (after logging) if the input is malformed enough to
/// break this invariant. `target_g` is supplied for the post-condition
/// sanity check (we should produce exactly `g` handle cycles).
fn attach_handle_cycles(
    bm: &mut BoundaryMatrix,
    original_mesh: &Mesh<INPUT>,
    target_g: usize,
) -> Option<()> {
    let tm = match tetrahedralize(original_mesh) {
        Ok(t) => t,
        Err(e) => {
            error!("Connectivity surgery aborted: tetrahedralization failed: {}", e);
            return None;
        }
    };
    let tb = build_tet_boundary(&tm);
    let hs = compute_handle_subspace(original_mesh, &tb);

    if hs.cycles.len() != target_g {
        error!(
            "Connectivity surgery aborted: handle-subspace dimension mismatch (got {}, expected genus {}). Tetrahedralization likely violated an invariant.",
            hs.cycles.len(),
            target_g
        );
        return None;
    }

    // Build a translation table from surface-edge index → contraction-mesh
    // column index in bm.matrix. Surface edges are pairs of u32 positions
    // in original_mesh.vert_ids(); contraction VertKeys share raw values
    // with the input mesh.
    let input_vert_ids = original_mesh.vert_ids();
    let surface_to_col: Vec<u32> = hs
        .surface_edges
        .iter()
        .map(|&[a, b]| {
            let a_input = input_vert_ids[a as usize];
            let b_input = input_vert_ids[b as usize];
            let a_contr = VIdx::new(a_input.raw());
            let b_contr = VIdx::new(b_input.raw());
            *bm.edge_to_col.get(&sort_edge(a_contr, b_contr)).unwrap_or_else(|| {
                panic!(
                    "handle-cycle translation: input surface edge ({:?}, {:?}) missing from contraction edge_to_col",
                    a_input, b_input
                )
            })
        })
        .collect();

    let translate = |cycle: Vec<u32>| -> Vec<u32> {
        let mut translated: Vec<u32> = cycle
            .into_iter()
            .map(|surf_idx| surface_to_col[surf_idx as usize])
            .collect();
        translated.sort();
        translated
    };

    bm.handle_cycles = hs.cycles.into_iter().map(translate).collect();
    // Augmented pivot set is now `B₁(X) + K`, the full target span for
    // the legality check.
    bm.rebuild_pivots();

    info!(
        "Attached {} handle-cycle generators to the boundary matrix.",
        bm.handle_cycles.len()
    );

    Some(())
}

/// Computes the edge quadric matrix (Eq 4 in paper).
fn compute_edge_quadric(p: Vector3D, a: Vector3D) -> Matrix4<f64> {
    let b = a.cross(&p);

    // Build K matrix rows
    let r0 = Vector4::new(0.0, -a.z, a.y, -b.x);
    let r1 = Vector4::new(a.z, 0.0, -a.x, -b.y);
    let r2 = Vector4::new(-a.y, a.x, 0.0, -b.z);
    let r3 = Vector4::new(0.0, 0.0, 0.0, 0.0);

    let k = Matrix4::from_rows(&[
        r0.transpose(),
        r1.transpose(),
        r2.transpose(),
        r3.transpose(),
    ]);

    k.transpose() * k
}

/// Sorts three vertex keys into a canonical order for face comparison.
#[inline]
fn sort_face(a: VIdx, b: VIdx, c: VIdx) -> [VIdx; 3] {
    let mut f = [a, b, c];
    f.sort();
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use mehsh::prelude::{Tag, VERT};
    use mehsh::utils::ids::IdMap;

    const RINGS: usize = 20;
    const SLOTS: usize = 6;
    const N_SIDE: usize = RINGS * SLOTS;
    const BOT_APEX: usize = N_SIDE;
    const TOP_APEX: usize = N_SIDE + 1;
    const N_VERTS: usize = N_SIDE + 2;
    // 19 strips * 6 slots * 2 tris + 2 caps * 6 = 240
    const N_FACES: usize = (RINGS - 1) * SLOTS * 2 + 2 * SLOTS;

    /// Build a closed triangulated cylinder mesh as described by `RINGS`/`SLOTS`,
    /// closed at top and bottom by triangle fans to two apex vertices. Returns the
    /// mesh along with the IdMap from original integer vertex index → mesh `VertKey`.
    fn build_cylinder<M: Tag>() -> (Mesh<M>, IdMap<VERT, M>) {
        let radius: f64 = 1.0;
        let height_step: f64 = 0.5;

        let mut positions = Vec::with_capacity(N_VERTS);
        for r in 0..RINGS {
            for s in 0..SLOTS {
                let theta = 2.0 * std::f64::consts::PI * (s as f64) / (SLOTS as f64);
                let x = radius * theta.cos();
                let y = radius * theta.sin();
                let z = (r as f64) * height_step;
                positions.push(Vector3D::new(x, y, z));
            }
        }
        positions.push(Vector3D::new(0.0, 0.0, -height_step));
        positions.push(Vector3D::new(0.0, 0.0, (RINGS as f64) * height_step));

        let v = |r: usize, s: usize| r * SLOTS + (s % SLOTS);
        let mut faces: Vec<Vec<usize>> = Vec::new();
        for r in 0..(RINGS - 1) {
            for s in 0..SLOTS {
                let s1 = (s + 1) % SLOTS;
                faces.push(vec![v(r, s), v(r + 1, s), v(r, s1)]);
                faces.push(vec![v(r, s1), v(r + 1, s), v(r + 1, s1)]);
            }
        }
        for s in 0..SLOTS {
            faces.push(vec![BOT_APEX, v(0, s), v(0, (s + 1) % SLOTS)]);
        }
        let last = RINGS - 1;
        for s in 0..SLOTS {
            faces.push(vec![TOP_APEX, v(last, (s + 1) % SLOTS), v(last, s)]);
        }

        let (mesh, vmap, _) =
            Mesh::<M>::from(&faces, &positions).expect("cylinder mesh build failed");
        (mesh, vmap)
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum EdgeKind {
        Vertical,
        Ring,
        Diagonal,
        Cap,
    }

    fn classify(a_idx: usize, b_idx: usize) -> EdgeKind {
        let is_apex = |idx: usize| idx == BOT_APEX || idx == TOP_APEX;
        if is_apex(a_idx) || is_apex(b_idx) {
            return EdgeKind::Cap;
        }
        let (ra, sa) = (a_idx / SLOTS, a_idx % SLOTS);
        let (rb, sb) = (b_idx / SLOTS, b_idx % SLOTS);
        if ra == rb {
            EdgeKind::Ring
        } else if sa == sb {
            EdgeKind::Vertical
        } else {
            EdgeKind::Diagonal
        }
    }

    /// Count undirected edges in `ctx.neighbors` (skipping dead vertices).
    fn edge_count(ctx: &SurgeryContext) -> usize {
        let mut sum = 0usize;
        for (&u, nset) in &ctx.neighbors {
            if ctx.is_dead.contains(&u) {
                continue;
            }
            sum += nset.iter().filter(|n| !ctx.is_dead.contains(n)).count();
        }
        sum / 2
    }

    /// Reason a popped candidate was rejected.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum RejectReason {
        Dead,
        NotAdjacent,
        NoFaces,
        LinkFailed,
        StaleCost,
    }

    /// Try to classify why an `is_legal_collapse_candidate` returned false (or, if it
    /// passed, the cost is stale). Returns Some if a rejection reason applies.
    fn rejection_reason(ctx: &SurgeryContext, c: &CollapseCandidate) -> Option<RejectReason> {
        if ctx.is_dead.contains(&c.u) || ctx.is_dead.contains(&c.v) {
            return Some(RejectReason::Dead);
        }
        if !ctx
            .neighbors
            .get(&c.u)
            .is_some_and(|s| s.contains(&c.v))
        {
            return Some(RejectReason::NotAdjacent);
        }
        if !ctx.edge_has_faces(c.u, c.v) {
            return Some(RejectReason::NoFaces);
        }
        if !ctx.check_link_condition(c.u, c.v) {
            return Some(RejectReason::LinkFailed);
        }
        None
    }

    /// The reason the surgery loop terminated.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum ExitReason {
        FacesEmpty,
        HeapEmpty,
    }

    #[derive(Debug)]
    struct RunSummary {
        alive_verts: usize,
        active_faces: usize,
        edges: usize,
        collapses: usize,
        exit: ExitReason,
        last_kind: Option<EdgeKind>,
        /// Number of accepted collapses for which the strict (Dey-Edelsbrunner) edge-link
        /// condition would have rejected: i.e. there exist common neighbours w1, w2 with both
        /// (u, w1, w2) and (v, w1, w2) active faces (= edge (w1,w2) ∈ lk(u) ∩ lk(v) \ lk(uv)).
        strict_violations_accepted: usize,
    }

    /// Returns Some((w1, w2)) if the picked collapse u→v violates the strict 2-manifold
    /// edge-link condition: there exist common neighbours w1≠w2 of u and v such that
    /// BOTH (u, w1, w2) and (v, w1, w2) are currently active faces.
    fn strict_violation(ctx: &SurgeryContext, u: VIdx, v: VIdx) -> Option<(VIdx, VIdx)> {
        let common: Vec<_> = ctx.neighbors[&u]
            .iter()
            .copied()
            .filter(|w| *w != v && ctx.neighbors[&v].contains(w))
            .collect();
        for i in 0..common.len() {
            for j in (i + 1)..common.len() {
                let w1 = common[i];
                let w2 = common[j];
                if ctx.active_faces.contains(&sort_face(u, w1, w2))
                    && ctx.active_faces.contains(&sort_face(v, w1, w2))
                {
                    return Some((w1, w2));
                }
            }
        }
        None
    }

    /// Runs the surgery loop on a fresh context built from `mesh`.
    /// If `link_condition_enabled` is false, the link-condition guard inside
    /// `is_legal_collapse_candidate` is bypassed (we still require alive + adjacent + has-faces).
    /// `vmap` is used to translate `VIdx` back into original integer indices for classification.
    fn run_surgery(
        mesh: &Mesh<CONTRACTION>,
        vmap: &IdMap<VERT, CONTRACTION>,
        link_condition_enabled: bool,
        verbose: bool,
    ) -> RunSummary {
        // SurgeryContext::new now needs an INPUT counterpart for the
        // tetrahedralization-based handle-subspace computation. Build one
        // with the same geometry; vertex raw values match across tags.
        let (input_mesh, _vmap_input): (Mesh<INPUT>, _) = build_cylinder();
        let mut ctx = SurgeryContext::new(mesh, &input_mesh).expect("preprocessing failed in test");

        // Resolve original-index for a VIdx (relies on vmap being the build_cylinder output).
        let idx_of = |k: VIdx| -> usize { *vmap.id(&k).expect("vmap missing key") };

        // Seed the heap with all directed edges that currently have faces.
        let mut heap: BinaryHeap<CollapseCandidate> = BinaryHeap::new();
        let vert_ids: Vec<_> = ctx.neighbors.keys().copied().collect();
        for u in vert_ids {
            let nbrs: Vec<_> = ctx.neighbors[&u].iter().copied().collect();
            for v in nbrs {
                if ctx.edge_has_faces(u, v) {
                    let cost = ctx.compute_collapse_cost(u, v);
                    heap.push(CollapseCandidate { u, v, cost });
                }
            }
        }

        eprintln!(
            "[link={}] seed: V={}, E={}, F={}, heap={}",
            link_condition_enabled,
            ctx.positions.len() - ctx.is_dead.len(),
            edge_count(&ctx),
            ctx.active_faces.len(),
            heap.len()
        );

        let mut collapses = 0usize;
        let mut last_kind: Option<EdgeKind> = None;
        let mut strict_violations_accepted = 0usize;
        const COST_EPSILON: f64 = 1e-8;

        let exit = loop {
            if ctx.active_faces.is_empty() {
                break ExitReason::FacesEmpty;
            }

            // Per-iteration we may pop many stale/illegal candidates before doing one collapse.
            let mut rejected_dead = 0usize;
            let mut rejected_not_adjacent = 0usize;
            let mut rejected_no_faces = 0usize;
            let mut rejected_link = 0usize;
            let mut rejected_stale = 0usize;
            let mut min_cost_seen = f64::INFINITY;

            let chosen = loop {
                let Some(c) = heap.pop() else {
                    break None;
                };
                min_cost_seen = min_cost_seen.min(c.cost);

                // Combined legality check (with link-condition optionally bypassed)
                let basic_ok = !ctx.is_dead.contains(&c.u)
                    && !ctx.is_dead.contains(&c.v)
                    && ctx
                        .neighbors
                        .get(&c.u)
                        .is_some_and(|s| s.contains(&c.v))
                    && ctx.edge_has_faces(c.u, c.v);

                let link_ok = if link_condition_enabled {
                    basic_ok && ctx.check_link_condition(c.u, c.v)
                } else {
                    basic_ok
                };

                if !link_ok {
                    match rejection_reason(&ctx, &c) {
                        Some(RejectReason::Dead) => rejected_dead += 1,
                        Some(RejectReason::NotAdjacent) => rejected_not_adjacent += 1,
                        Some(RejectReason::NoFaces) => rejected_no_faces += 1,
                        Some(RejectReason::LinkFailed) => rejected_link += 1,
                        _ => {}
                    }
                    continue;
                }

                let real_cost = ctx.compute_collapse_cost(c.u, c.v);
                if (c.cost - real_cost).abs() > COST_EPSILON {
                    heap.push(CollapseCandidate {
                        u: c.u,
                        v: c.v,
                        cost: real_cost,
                    });
                    rejected_stale += 1;
                    continue;
                }
                break Some(c);
            };

            let Some(c) = chosen else {
                if verbose {
                    eprintln!(
                        "[link={}] iter exhausted heap: rejected dead={}, !adj={}, !face={}, link={}, stale={}",
                        link_condition_enabled,
                        rejected_dead,
                        rejected_not_adjacent,
                        rejected_no_faces,
                        rejected_link,
                        rejected_stale,
                    );
                }
                break ExitReason::HeapEmpty;
            };

            let a_idx = idx_of(c.u);
            let b_idx = idx_of(c.v);
            let kind = classify(a_idx, b_idx);
            last_kind = Some(kind);

            let sv = strict_violation(&ctx, c.u, c.v);
            if sv.is_some() {
                strict_violations_accepted += 1;
            }

            if verbose {
                let sv_tag = match sv {
                    Some((w1, w2)) => format!(" STRICT-VIOLATION via ({},{})", idx_of(w1), idx_of(w2)),
                    None => String::new(),
                };
                eprintln!(
                    "[link={}] it={:3}: V={:3} E={:3} F={:3} | pick {:3}->{:3} ({:?}) cost={:.4e} | rej dead={} !adj={} !face={} link={} stale={}{}",
                    link_condition_enabled,
                    collapses,
                    ctx.positions.len() - ctx.is_dead.len(),
                    edge_count(&ctx),
                    ctx.active_faces.len(),
                    a_idx,
                    b_idx,
                    kind,
                    c.cost,
                    rejected_dead,
                    rejected_not_adjacent,
                    rejected_no_faces,
                    rejected_link,
                    rejected_stale,
                    sv_tag,
                );
            }

            ctx.collapse_edge(c.u, c.v);
            collapses += 1;

            // Re-seed edges incident to the survivor
            let nbrs: Vec<_> = ctx.neighbors[&c.v].iter().copied().collect();
            for n in nbrs {
                if ctx.edge_has_faces(c.v, n) {
                    let cost = ctx.compute_collapse_cost(c.v, n);
                    heap.push(CollapseCandidate { u: c.v, v: n, cost });
                }
                if ctx.edge_has_faces(n, c.v) {
                    let cost = ctx.compute_collapse_cost(n, c.v);
                    heap.push(CollapseCandidate { u: n, v: c.v, cost });
                }
            }
        };

        let summary = RunSummary {
            alive_verts: ctx.positions.len() - ctx.is_dead.len(),
            active_faces: ctx.active_faces.len(),
            edges: edge_count(&ctx),
            collapses,
            exit,
            last_kind,
            strict_violations_accepted,
        };
        eprintln!(
            "[link={}] EXIT {:?}: collapses={}, V={}, E={}, F={}, last_edge_kind={:?}, strict_violations_accepted={}",
            link_condition_enabled,
            summary.exit,
            summary.collapses,
            summary.alive_verts,
            summary.edges,
            summary.active_faces,
            summary.last_kind,
            summary.strict_violations_accepted,
        );
        summary
    }

    /// Step 3: Direct probe of `check_link_condition` for a middle-ring edge.
    /// Ring 10 is the middle. The edge from slot 0 to slot 1 on ring 10 has a third
    /// ring-10 vertex (slot 5? slot 2?) as a common neighbor only if the triangulation
    /// connects it across — for our quad triangulation, common neighbors of an edge on a
    /// single ring are the two diagonal-vertex/vertical-vertex pairs from the strips on
    /// either side. The ring-10 vertices themselves are NOT mutual neighbors except via
    /// the strip diagonals, so this probe instead targets a *ring* edge directly to
    /// confirm the link-condition behaviour on edges within a single ring.
    #[test]
    fn link_condition_probe_middle_ring_edge() {
        let (mesh, vmap): (Mesh<CONTRACTION>, _) = build_cylinder();
        let (input_mesh, _): (Mesh<INPUT>, _) = build_cylinder();
        let ctx = SurgeryContext::new(&mesh, &input_mesh).expect("preprocessing failed in test");
        let idx_of = |k: VIdx| -> usize { *vmap.id(&k).expect("vmap missing key") };

        let r = 10usize;
        let s0 = 0usize;
        let s1 = 1usize;
        let i = *vmap.key(r * SLOTS + s0).expect("missing ring 10 slot 0");
        let j = *vmap.key(r * SLOTS + s1).expect("missing ring 10 slot 1");
        eprintln!(
            "PROBE: middle-ring edge i={:?} (idx {}) -- j={:?} (idx {})",
            i, idx_of(i), j, idx_of(j)
        );

        let n_i: HashSet<_> = ctx.neighbors[&i].iter().copied().collect();
        let n_j: HashSet<_> = ctx.neighbors[&j].iter().copied().collect();
        let common: Vec<_> = n_i.intersection(&n_j).copied().filter(|k| *k != i && *k != j).collect();
        eprintln!(
            "  N(i)={:?}",
            n_i.iter().map(|k| idx_of(*k)).collect::<Vec<_>>()
        );
        eprintln!(
            "  N(j)={:?}",
            n_j.iter().map(|k| idx_of(*k)).collect::<Vec<_>>()
        );
        eprintln!(
            "  common neighbors = {:?}",
            common.iter().map(|k| idx_of(*k)).collect::<Vec<_>>()
        );
        for &k in &common {
            let exists = ctx.active_faces.contains(&sort_face(i, j, k));
            eprintln!("    k={:>3}: (i,j,k) face exists? {}", idx_of(k), exists);
        }
        let verdict = ctx.check_link_condition(i, j);
        eprintln!("  check_link_condition(i, j) = {}", verdict);

        // Also: for any *pair* of common neighbors w1, w2, if both (i,w1,w2) and (j,w1,w2)
        // are faces, then w1-w2 ∈ lk(i) ∩ lk(j) but not in lk(ij). The classical 2-manifold
        // link condition would reject. The current implementation does NOT check this.
        let mut strict_violations: Vec<(usize, usize)> = Vec::new();
        for a in 0..common.len() {
            for b in (a + 1)..common.len() {
                let w1 = common[a];
                let w2 = common[b];
                if ctx.active_faces.contains(&sort_face(i, w1, w2))
                    && ctx.active_faces.contains(&sort_face(j, w1, w2))
                {
                    strict_violations.push((idx_of(w1), idx_of(w2)));
                }
            }
        }
        eprintln!(
            "  strict (Dey/Edelsbrunner) edge-link violations: {:?}",
            strict_violations
        );
    }

    /// Step 5: A/B experiment — run the surgery twice on the same cylinder, with the
    /// link-condition check enabled vs. disabled. If the implementation is functioning,
    /// (b) should leave V ≫ 2 while (a) collapses to V = 2.
    #[test]
    fn cylinder_a_b_link_condition() {
        let (mesh_a, vmap_a): (Mesh<CONTRACTION>, _) = build_cylinder();
        let (mesh_b, vmap_b): (Mesh<CONTRACTION>, _) = build_cylinder();

        eprintln!("--- Run (a): link condition DISABLED ---");
        let a = run_surgery(&mesh_a, &vmap_a, false, false);
        eprintln!("--- Run (b): link condition ENABLED  ---");
        let b = run_surgery(&mesh_b, &vmap_b, true, false);

        eprintln!("A/B summary:");
        eprintln!("  (a) link disabled: V={}, E={}, F={}, collapses={}", a.alive_verts, a.edges, a.active_faces, a.collapses);
        eprintln!("  (b) link enabled : V={}, E={}, F={}, collapses={}", b.alive_verts, b.edges, b.active_faces, b.collapses);
    }

    /// Step 2 + 4: Per-iteration instrumented run. Logs (V,E,F), the selected edge with
    /// classification, rejection counts, and the final exit reason.
    #[test]
    fn cylinder_surgery_per_iteration_trace() {
        let (mesh, vmap): (Mesh<CONTRACTION>, _) = build_cylinder();
        eprintln!(
            "Input mesh: verts={}, edges(half)={}, faces={}",
            mesh.nr_verts(),
            mesh.nr_edges(),
            mesh.nr_faces()
        );
        assert_eq!(mesh.nr_verts(), N_VERTS);
        assert_eq!(mesh.nr_faces(), N_FACES);

        let summary = run_surgery(&mesh, &vmap, true, true);

        eprintln!(
            "FINAL: V={}, E={}, F={}, exit={:?}, last_edge_kind={:?}",
            summary.alive_verts,
            summary.edges,
            summary.active_faces,
            summary.exit,
            summary.last_kind,
        );
    }

}
