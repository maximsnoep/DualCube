//! Wraps TetGen (via the `tritet` crate) to tetrahedralize the solid
//! bounded by a closed triangle mesh.
//!
//! This is the volumetric-modelling step of the homology-preserving
//! surgery pipeline (see `HOMOLOGY_PRESERVING_SURGERY.md`). The output
//! [`TetMesh`] is the discrete model of the solid `V` from which we will
//! compute the handle subspace `K = ker(H₁(S) → H₁(V))`.

use log::{info, warn};
use mehsh::prelude::{HasPosition, HasVertices, Mesh, Vector3D, VertKey};
use tritet::Tetgen;

use crate::{prelude::INPUT, skeleton::geometry::moller_trumbore};

/// A simplicial 3-complex: vertices and tetrahedra produced by TetGen.
///
/// Vertex indices `0..n_input_vertices` correspond, in order, to the
/// surface vertices passed in. TetGen places these first by convention;
/// any Steiner points it inserts get appended at indices `n_input_vertices..`.
/// This means the inclusion `S ↪ T` on vertices is just the identity map
/// on the first `n_input_vertices` indices.
#[derive(Debug, Clone)]
pub struct TetMesh {
    /// All vertex positions. The first `n_input_vertices` entries are the
    /// surface vertices, in the order passed in to [`tetrahedralize`].
    pub vertices: Vec<Vector3D>,
    /// Each tetrahedron, as four vertex indices into `vertices`.
    pub tets: Vec<[u32; 4]>,
    /// How many of the first entries of `vertices` came from the input
    /// surface mesh (the rest are Steiner points TetGen inserted).
    pub n_input_vertices: usize,
}

impl TetMesh {
    pub fn n_vertices(&self) -> usize {
        self.vertices.len()
    }

    pub fn n_tets(&self) -> usize {
        self.tets.len()
    }
}

/// Errors from tetrahedralizing an input mesh.
#[derive(Debug)]
pub enum TetError {
    /// A face had a vertex count other than 3.
    NonTriangularFace { face_idx: usize, vertex_count: usize },
    /// A surface vertex was missing from the position map (should be impossible
    /// if the mesh is well-formed; included as a safety net).
    MissingVertexPosition,
    /// Forwarded from the underlying TetGen wrapper.
    Tetgen(&'static str),
}

impl std::fmt::Display for TetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TetError::NonTriangularFace { face_idx, vertex_count } => write!(
                f,
                "tetrahedralize: face {} has {} vertices, expected 3 (TetGen requires a triangulated boundary)",
                face_idx, vertex_count
            ),
            TetError::MissingVertexPosition => write!(f, "tetrahedralize: surface vertex missing from position map"),
            TetError::Tetgen(msg) => write!(f, "tetrahedralize: TetGen error: {}", msg),
        }
    }
}

impl std::error::Error for TetError {}

/// Tetrahedralizes the solid bounded by the closed triangle mesh `mesh`.
///
/// Returns a [`TetMesh`] of the interior. The input mesh must be a closed,
/// triangulated, manifold surface — the same precondition we already check
/// in `connectivity_surgery::preprocess_topology`. TetGen does not validate
/// closedness here; if the surface is open, the result is undefined.
pub fn tetrahedralize(mesh: &Mesh<INPUT>) -> Result<TetMesh, TetError> {
    // Build a stable [0, n) index for every surface vertex. The order we
    // feed these to TetGen is the order they will appear at indices
    // `0..n_input_vertices` of the output.
    let vert_ids: Vec<VertKey<INPUT>> = mesh.vert_ids();
    let n_input_vertices = vert_ids.len();
    let surface_index_of: std::collections::HashMap<VertKey<INPUT>, usize> = vert_ids
        .iter()
        .enumerate()
        .map(|(i, &v)| (v, i))
        .collect();

    // Collect surface triangles as triples of [0, n_input_vertices) indices.
    let face_ids = mesh.face_ids();
    let mut triangles: Vec<[usize; 3]> = Vec::with_capacity(face_ids.len());
    for (face_idx, fid) in face_ids.iter().enumerate() {
        let verts: Vec<_> = mesh.vertices(*fid).collect();
        if verts.len() != 3 {
            return Err(TetError::NonTriangularFace { face_idx, vertex_count: verts.len() });
        }
        triangles.push([
            *surface_index_of.get(&verts[0]).ok_or(TetError::MissingVertexPosition)?,
            *surface_index_of.get(&verts[1]).ok_or(TetError::MissingVertexPosition)?,
            *surface_index_of.get(&verts[2]).ok_or(TetError::MissingVertexPosition)?,
        ]);
    }

    // Configure TetGen: one facet per surface triangle, one region (the
    // interior), no holes. The region marker is what triggers TetGen's PLC
    // (constrained Delaunay) mode — without it the call degenerates to
    // plain Delaunay tetrahedralization of the input points and the
    // surface facets are not preserved.
    let facet_npoint: Vec<usize> = vec![3; triangles.len()];
    let mut tet = Tetgen::new(n_input_vertices, Some(facet_npoint), Some(1), None)
        .map_err(TetError::Tetgen)?;

    // Feed surface vertices.
    for (i, &v) in vert_ids.iter().enumerate() {
        let p = mesh.position(v);
        tet.set_point(i, 0, p.x, p.y, p.z).map_err(TetError::Tetgen)?;
    }

    // Feed surface triangles.
    for (f, tri) in triangles.iter().enumerate() {
        for m in 0..3 {
            tet.set_facet_point(f, m, tri[m]).map_err(TetError::Tetgen)?;
        }
    }

    // Mark the interior region by giving TetGen a point inside it.
    //
    // The vertex centroid (used previously) lands inside any star-shaped
    // solid but routinely falls *outside* for non-star-shaped genus > 0
    // surfaces — e.g. a pretzel-shaped genus-2 mesh whose centroid lands
    // in a tunnel hole. When that happens TetGen fills the complementary
    // region instead and every downstream computation that assumed
    // `T = interior of S` is silently wrong.
    //
    // Instead pick a non-degenerate surface triangle, compute its centroid
    // and its *inward* normal (sign of vertex-order orientation determined
    // from the signed volume of the closed surface), ray-cast inward until
    // we hit the next surface triangle, and place the marker at half that
    // distance. The marker is then guaranteed to be inside the solid for
    // any closed orientable manifold input — including thin features,
    // because the half-distance step can't reach the opposing wall.
    let marker = interior_region_marker(mesh, &triangles, &vert_ids)?;
    info!(
        "Tetrahedralize: region marker at ({:.6}, {:.6}, {:.6})",
        marker.x, marker.y, marker.z
    );
    tet.set_region(0, 1, marker.x, marker.y, marker.z, None)
        .map_err(TetError::Tetgen)?;

    // Run TetGen on the PLC. Defaults: no quadratic elements, no global size
    // bounds, no minimum-angle constraint. We want the cheapest valid
    // tetrahedralization since topology (not element quality) is all we need.
    tet.generate_mesh(false, false, None, None)
        .map_err(TetError::Tetgen)?;

    // Read back vertices.
    let n_out = tet.out_npoint();
    let mut vertices: Vec<Vector3D> = Vec::with_capacity(n_out);
    for i in 0..n_out {
        vertices.push(Vector3D::new(
            tet.out_point(i, 0),
            tet.out_point(i, 1),
            tet.out_point(i, 2),
        ));
    }

    // Read back tetrahedra.
    let n_tet = tet.out_ncell();
    debug_assert_eq!(tet.out_cell_npoint(), 4, "expected 4 vertices per tet");
    let mut tets: Vec<[u32; 4]> = Vec::with_capacity(n_tet);
    for i in 0..n_tet {
        tets.push([
            tet.out_cell_point(i, 0) as u32,
            tet.out_cell_point(i, 1) as u32,
            tet.out_cell_point(i, 2) as u32,
            tet.out_cell_point(i, 3) as u32,
        ]);
    }

    let steiner = n_out.saturating_sub(n_input_vertices);
    info!(
        "TetGen produced {} tetrahedra from {} surface vertices ({} Steiner points added, {} surface triangles).",
        n_tet, n_input_vertices, steiner, triangles.len()
    );

    // Sanity-check that TetGen kept the input surface vertices at the
    // first `n_input_vertices` indices in the *correct order*. If it
    // didn't, every downstream computation that looks up T-edges by
    // input-vertex pair (`tet_boundary.edge_col(a, b)`) will silently
    // address the wrong edge — and the handle-subspace test becomes
    // garbage.
    let mut drifted = 0usize;
    let mut max_drift: f64 = 0.0;
    for (i, &v) in vert_ids.iter().enumerate() {
        let expected = mesh.position(v);
        let got = vertices[i];
        let dx = (got.x - expected.x).abs();
        let dy = (got.y - expected.y).abs();
        let dz = (got.z - expected.z).abs();
        let drift = dx + dy + dz;
        let scale = expected.x.abs() + expected.y.abs() + expected.z.abs() + 1.0;
        if drift > 1e-9 * scale {
            drifted += 1;
            if drift > max_drift {
                max_drift = drift;
            }
        }
    }
    if drifted > 0 {
        warn!(
            "Tetrahedralize: {} of {} surface vertices drifted from their input positions (max drift {}). TetGen may have reordered/merged vertices; edge-column lookups by input-vertex pair would address the wrong edge.",
            drifted, n_input_vertices, max_drift
        );
    }

    Ok(TetMesh {
        vertices,
        tets,
        n_input_vertices,
    })
}

/// Picks a point guaranteed to lie strictly inside the closed orientable
/// solid bounded by `mesh`. Used as TetGen's region marker so it fills
/// the correct side of the surface.
///
/// Algorithm:
/// 1. Compute the signed volume of the surface (origin-invariant for any
///    closed surface). Its sign tells us whether the stored vertex order
///    of each triangle produces an outward-pointing or inward-pointing
///    normal under `(b−a) × (c−a)`.
/// 2. Pick the first non-degenerate triangle, take its centroid and its
///    inward normal.
/// 3. Ray-cast inward and find the smallest positive `t` at which the
///    ray crosses any other surface triangle (Möller–Trumbore over the
///    full triangle list — `O(F)` per cast, fast enough for one shot).
/// 4. Return `centroid + (t/2) · inward`. Half the distance can't reach
///    the opposing wall, so the marker is inside the solid even for
///    thin features.
fn interior_region_marker(
    mesh: &Mesh<INPUT>,
    triangles: &[[usize; 3]],
    vert_ids: &[VertKey<INPUT>],
) -> Result<Vector3D, TetError> {
    // Signed volume of the closed surface (6×, since (a×b)·c is 6× the
    // tetrahedron volume from origin). Sign is invariant of origin
    // choice for a closed surface and tells us the vertex-order
    // convention.
    let mut six_volume: f64 = 0.0;
    for tri in triangles {
        let pa = mesh.position(vert_ids[tri[0]]);
        let pb = mesh.position(vert_ids[tri[1]]);
        let pc = mesh.position(vert_ids[tri[2]]);
        six_volume += pa.cross(&pb).dot(&pc);
    }
    if six_volume.abs() < 1e-12 {
        return Err(TetError::Tetgen(
            "surface mesh has zero signed volume — closed orientable manifold expected",
        ));
    }
    let outward_sign = if six_volume > 0.0 { 1.0 } else { -1.0 };

    // Pick the first non-degenerate triangle. For any reasonable mesh
    // this is index 0.
    let mut src: Option<(usize, Vector3D, Vector3D)> = None;
    for (i, tri) in triangles.iter().enumerate() {
        let pa = mesh.position(vert_ids[tri[0]]);
        let pb = mesh.position(vert_ids[tri[1]]);
        let pc = mesh.position(vert_ids[tri[2]]);
        let normal = (pb - pa).cross(&(pc - pa));
        let two_area = normal.norm();
        if two_area < 1e-20 {
            continue;
        }
        let centroid = (pa + pb + pc) / 3.0;
        let inward = normal * (-outward_sign / two_area);
        src = Some((i, centroid, inward));
        break;
    }
    let (src_idx, centroid, inward) = src
        .ok_or(TetError::Tetgen("all surface triangles are degenerate"))?;

    // Ray-cast inward; find the smallest positive intersection parameter.
    let mut nearest_t = f64::INFINITY;
    for (j, tri) in triangles.iter().enumerate() {
        if j == src_idx {
            continue;
        }
        let qa = mesh.position(vert_ids[tri[0]]);
        let qb = mesh.position(vert_ids[tri[1]]);
        let qc = mesh.position(vert_ids[tri[2]]);
        if let Some(t) = ray_triangle_intersect(centroid, inward, qa, qb, qc) {
            // Strict positivity guards against grazing the source face
            // through a shared edge or vertex.
            if t > 1e-9 && t < nearest_t {
                nearest_t = t;
            }
        }
    }
    if !nearest_t.is_finite() {
        return Err(TetError::Tetgen(
            "inward ray from source triangle hit no other surface triangle — surface is not closed?",
        ));
    }

    Ok(centroid + inward * (nearest_t * 0.5))
}

/// Forward ray-triangle intersection over `f64`. Returns the positive
/// parameter `t` such that `origin + t · direction` lies on the triangle
/// `(a, b, c)`, or `None` if the ray misses, hits the triangle behind the
/// origin, or is parallel.
fn ray_triangle_intersect(
    origin: Vector3D,
    direction: Vector3D,
    a: Vector3D,
    b: Vector3D,
    c: Vector3D,
) -> Option<f64> {
    moller_trumbore(origin, direction, a, b, c, 1e-12).filter(|&t| t > 0.0)
}

#[cfg(test)]
#[path = "test/test_tetrahedralize.rs"]
mod test_tetrahedralize;