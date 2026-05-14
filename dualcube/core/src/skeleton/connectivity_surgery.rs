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
/// face triple and columns by undirected edge. Maintained alongside the
/// surgery state when the input has positive genus, so the legality check
/// can probe what β₁ would become after a hypothetical collapse.
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

    /// β₁ of the input simplicial complex (= 2g for a closed orientable
    /// 2-manifold of genus g). The legality check rejects any collapse that
    /// would change β₁ away from this target.
    target_beta_1: usize,

    /// Boundary matrix ∂₂ tracked across collapses. `None` when the input has
    /// genus 0, in which case the legality check is a no-op (always accept).
    boundary_matrix: Option<BoundaryMatrix>,
}

impl SurgeryContext {
    /// Creates a new SurgeryContext from a contracted mesh.
    ///
    /// Returns `None` (after logging an error) when the input mesh fails the
    /// preprocessing checks: not a closed connected 2-manifold, or the
    /// derived genus isn't a non-negative integer. Callers should treat
    /// `None` as "produce an empty skeleton".
    fn new(mesh: &Mesh<CONTRACTION>) -> Option<Self> {
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
        // connected 2-manifold, derive genus, and (for g > 0) build the
        // boundary matrix ∂₂ that the homology-preserving legality check
        // works against.
        // At construction time no vertex has been collapsed yet.
        let is_dead: HashSet<VIdx> = HashSet::new();
        let (target_beta_1, boundary_matrix) =
            preprocess_topology(&active_faces, &neighbors, &is_dead, vert_ids.len())?;

        Some(Self {
            positions,
            center,
            scale,
            neighbors,
            active_faces,
            quadrics,
            is_dead,
            vertex_to_original,
            target_beta_1,
            boundary_matrix,
        })
    }

    /// Computes the collapse cost for edge u -> v (Eq 8 in paper).
    fn compute_collapse_cost(&self, u: VIdx, v: VIdx) -> f64 {
        const WA: f64 = 1.0; // Shape weight
        const WB: f64 = 0.1; // Sampling weight

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

        WA * shape_cost + WB * sampling_cost
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

    /// Homology-preserving legality check.
    ///
    /// For genus-0 inputs (no boundary matrix is maintained) this is a
    /// no-op: every collapse is accepted. Otherwise: probe what β₁ of the
    /// simplicial complex would become after a hypothetical collapse u → v
    /// and reject if it would change. This is the only invariant we care
    /// about — it's strictly weaker than the classical Dey–Edelsbrunner
    /// edge-link condition, so it admits collapses that the strict check
    /// would refuse, while still preventing topology change.
    fn check_link_condition(&self, u: VIdx, v: VIdx) -> bool {
        let Some(bm) = self.boundary_matrix.as_ref() else {
            return true;
        };
        let Some(delta) = self.compute_collapse_delta(u, v) else {
            return true;
        };

        // Snapshot the boundary matrix, apply the would-be changes, and
        // re-reduce to get the post-collapse rank.
        let mut snap = bm.clone();
        Self::apply_collapse_delta(&mut snap, &delta);
        snap.matrix.reduce();
        let new_rank = snap.matrix.rank();

        // Post-collapse counts. For a connected complex, β₁ = E − V + 1 −
        // rank(∂₂). Edge collapse on a connected complex preserves
        // connectedness (the merged vertex is still connected to everything
        // either endpoint reached), so we can keep β₀ = 1 fixed.
        let v_now = (self.positions.len() - self.is_dead.len()) as i64;
        let e_now = bm.edge_to_col.len() as i64;
        let new_v = v_now - delta.delta_v as i64;
        let new_e = e_now - delta.delta_e as i64;
        let new_beta_1 = new_e - new_v + 1 - new_rank as i64;
        new_beta_1 == self.target_beta_1 as i64
    }

    /// Computes what the (face_to_row, edge_to_col, matrix) state would
    /// change to under a hypothetical collapse u → v. Returns `None` when
    /// no boundary matrix is maintained (genus 0).
    fn compute_collapse_delta(&self, u: VIdx, v: VIdx) -> Option<CollapseDelta> {
        let bm = self.boundary_matrix.as_ref()?;
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

    /// Applies a [`CollapseDelta`] in place to the given boundary matrix and
    /// its face/edge maps. Used both for the snapshot in
    /// [`Self::check_link_condition`] and for the live commit in
    /// [`Self::collapse_edge`].
    fn apply_collapse_delta(bm: &mut BoundaryMatrix, delta: &CollapseDelta) {
        // 1. Column merges: shared-neighbor edge columns collapse into one.
        //    Done first so subsequent row clears reference the merged columns
        //    if needed (they don't, but ordering is harmless).
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

    /// Returns a one-line diagnostic summary for directed edge u -> v.
    fn edge_diagnostic_summary(&self, u: VIdx, v: VIdx) -> String {
        let u_dead = self.is_dead.contains(&u);
        let v_dead = self.is_dead.contains(&v);
        let adjacent = self
            .neighbors
            .get(&u)
            .is_some_and(|neighbor_set| neighbor_set.contains(&v));
        let has_faces = adjacent && !u_dead && !v_dead && self.edge_has_faces(u, v);
        let link_ok = has_faces && self.check_link_condition(u, v);

        format!(
            "{:?}->{:?}[u_dead={}, v_dead={}, adjacent={}, has_faces={}, link_ok={}]",
            u, v, u_dead, v_dead, adjacent, has_faces, link_ok
        )
    }

    /// Performs edge collapse: u merges into v.
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

        // Commit the boundary-matrix delta to the live state.
        if let (Some(delta), Some(bm)) = (delta, self.boundary_matrix.as_mut()) {
            Self::apply_collapse_delta(bm, &delta);
        }
    }


    /// Builds the final CurveSkeleton graph from the surgery result.
    fn to_curve_skeleton(&self, original_mesh: &Mesh<INPUT>) -> CurveSkeleton {
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

        // Add edges
        for (&u, neighbor_set) in &self.neighbors {
            if self.is_dead.contains(&u) {
                continue;
            }

            if let Some(&u_node) = node_indices.get(&u) {
                for &v in neighbor_set {
                    // Only add each edge once (when u < v)
                    if u < v && !self.is_dead.contains(&v) {
                        if let Some(&v_node) = node_indices.get(&v) {
                            // compute boundary loop from the two node patches via face-walk
                            let patch_u = graph.node_weight(u_node).unwrap().patch_vertices.clone();
                            let patch_v = graph.node_weight(v_node).unwrap().patch_vertices.clone();
                            graph.add_edge(
                                u_node,
                                v_node,
                                BoundaryLoop::new(&patch_u, &patch_v, original_mesh),
                            );
                        }
                    }
                }
            }
        }

        graph
    }
}

/// Extracts a 1D curve skeleton from a contracted mesh via connectivity surgery.
///
/// # Arguments
/// * `contracted_mesh` - The mesh after contraction (thin but still 2D).
/// * `original_mesh` - The original input mesh (for embedding refinement).
///
/// # Returns
/// A `CurveSkeleton` graph representing the 1D skeleton.
pub fn extract_skeleton(
    contracted_mesh: &Mesh<CONTRACTION>,
    original_mesh: &Mesh<INPUT>,
) -> CurveSkeleton {
    let Some(mut ctx) = SurgeryContext::new(contracted_mesh) else {
        // Preprocessing already logged the reason. Return an empty skeleton
        // so the rest of the pipeline doesn't crash on a missing CurveSkeleton.
        return CurveSkeleton::default();
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

            if let Some(face) = ctx.active_faces.iter().copied().next() {
                let [a, b, c] = face;
                warn!(
                    "Sample remaining face {:?} diagnostics: {}, {}, {}, {}, {}, {}",
                    face,
                    ctx.edge_diagnostic_summary(a, b),
                    ctx.edge_diagnostic_summary(b, a),
                    ctx.edge_diagnostic_summary(b, c),
                    ctx.edge_diagnostic_summary(c, b),
                    ctx.edge_diagnostic_summary(c, a),
                    ctx.edge_diagnostic_summary(a, c)
                );
            }
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

    info!(
        "Connectivity surgery complete. Collapsed {} edges. Remaining vertices: {}",
        collapses,
        ctx.positions.len() - ctx.is_dead.len()
    );

    // Build the skeleton
    let mut skeleton = ctx.to_curve_skeleton(original_mesh);

    // Refine embedding using original mesh positions
    skeleton.refine_embeddings(original_mesh);

    skeleton
}

/// Verifies the input is a closed connected 2-manifold and (when genus > 0)
/// builds the GF(2) boundary matrix ∂₂ that will be used for the
/// homology-preserving legality check. Returns `(target_beta_1, matrix)`
/// where `matrix` is `None` for genus 0 (legality check becomes a no-op).
///
/// On failure (non-manifold edge, disconnected, non-integer genus, sanity
/// check mismatch), logs an error and returns `None`.
fn preprocess_topology(
    active_faces: &HashSet<[VIdx; 3]>,
    neighbors: &HashMap<VIdx, HashSet<VIdx>>,
    is_dead: &HashSet<VIdx>,
    n_verts_total: usize,
) -> Option<(usize, Option<BoundaryMatrix>)> {
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
    let target_beta_1 = 2 * g;

    info!(
        "Connectivity surgery preprocessing: V={}, E={}, F={}, χ={}, genus={}.",
        v_count, e_count, f_count, chi, g
    );

    if g == 0 {
        return Some((target_beta_1, None));
    }

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

    Some((
        target_beta_1,
        Some(BoundaryMatrix {
            matrix,
            face_to_row,
            edge_to_col,
        }),
    ))
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
        let mut ctx = SurgeryContext::new(mesh).expect("preprocessing failed in test");

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
        let ctx = SurgeryContext::new(&mesh).expect("preprocessing failed in test");
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
