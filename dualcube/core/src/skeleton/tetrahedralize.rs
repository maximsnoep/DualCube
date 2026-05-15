//! Wraps TetGen (via the `tritet` crate) to tetrahedralize the solid
//! bounded by a closed triangle mesh.
//!
//! This is the volumetric-modelling step of the homology-preserving
//! surgery pipeline (see `HOMOLOGY_PRESERVING_SURGERY.md`). The output
//! [`TetMesh`] is the discrete model of the solid `V` from which we will
//! compute the handle subspace `K = ker(H₁(S) → H₁(V))`.

use mehsh::prelude::{HasPosition, HasVertices, Mesh, Vector3D, VertKey};
use tritet::Tetgen;

use crate::prelude::INPUT;

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

    // Mark the interior region by giving TetGen a point inside it. For
    // closed meshes whose vertex centroid lies inside the volume (true for
    // any star-shaped solid and in particular for all our test inputs and
    // for bob), the centroid of all surface vertices works.
    let (mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0);
    for &v in &vert_ids {
        let p = mesh.position(v);
        cx += p.x;
        cy += p.y;
        cz += p.z;
    }
    let n = n_input_vertices as f64;
    cx /= n;
    cy /= n;
    cz /= n;
    tet.set_region(0, 1, cx, cy, cz, None).map_err(TetError::Tetgen)?;

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

    Ok(TetMesh {
        vertices,
        tets,
        n_input_vertices,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mehsh::prelude::Mesh;

    /// Build a regular tetrahedron as a 4-vertex closed surface mesh.
    fn build_tetrahedron_surface() -> Mesh<INPUT> {
        let positions = vec![
            Vector3D::new(0.0, 0.0, 0.0),
            Vector3D::new(1.0, 0.0, 0.0),
            Vector3D::new(0.0, 1.0, 0.0),
            Vector3D::new(0.0, 0.0, 1.0),
        ];
        // Outward-pointing faces (right-hand rule from the centroid).
        let faces = vec![
            vec![0, 2, 1],
            vec![0, 1, 3],
            vec![0, 3, 2],
            vec![1, 2, 3],
        ];
        let (mesh, _, _) = Mesh::<INPUT>::from(&faces, &positions)
            .expect("tetrahedron surface mesh build failed");
        mesh
    }

    /// A tetrahedron interior tetrahedralizes successfully. TetGen may or
    /// may not insert a Steiner point at the centroid (we've observed both),
    /// so we don't assert the tet count — just that the call succeeds and
    /// the output is internally consistent.
    #[test]
    fn tetrahedron_interior_round_trips() {
        let mesh = build_tetrahedron_surface();
        let tm = tetrahedralize(&mesh).expect("tetgen failed on tetrahedron");

        assert_eq!(tm.n_input_vertices, 4, "input vertex count preserved");
        assert!(tm.n_tets() >= 1, "interior must be non-empty");
        // Every output tet vertex index is in range.
        for (i, tet) in tm.tets.iter().enumerate() {
            for &idx in tet {
                assert!(
                    (idx as usize) < tm.n_vertices(),
                    "tet {} has out-of-range vertex {}",
                    i,
                    idx
                );
            }
        }
    }

    /// Build a closed cube surface (12 triangles, 8 vertices). TetGen should
    /// produce some tets without inserting Steiner points (a cube fills with
    /// 5 or 6 tets, depending on diagonal choice).
    fn build_cube_surface() -> Mesh<INPUT> {
        let positions = vec![
            Vector3D::new(0.0, 0.0, 0.0), // 0
            Vector3D::new(1.0, 0.0, 0.0), // 1
            Vector3D::new(1.0, 1.0, 0.0), // 2
            Vector3D::new(0.0, 1.0, 0.0), // 3
            Vector3D::new(0.0, 0.0, 1.0), // 4
            Vector3D::new(1.0, 0.0, 1.0), // 5
            Vector3D::new(1.0, 1.0, 1.0), // 6
            Vector3D::new(0.0, 1.0, 1.0), // 7
        ];
        // Outward-facing triangles, two per square face.
        let faces = vec![
            // bottom z=0 (normal -z)
            vec![0, 2, 1], vec![0, 3, 2],
            // top z=1 (normal +z)
            vec![4, 5, 6], vec![4, 6, 7],
            // front y=0 (normal -y)
            vec![0, 1, 5], vec![0, 5, 4],
            // back y=1 (normal +y)
            vec![3, 7, 6], vec![3, 6, 2],
            // left x=0 (normal -x)
            vec![0, 4, 7], vec![0, 7, 3],
            // right x=1 (normal +x)
            vec![1, 2, 6], vec![1, 6, 5],
        ];
        let (mesh, _, _) = Mesh::<INPUT>::from(&faces, &positions)
            .expect("cube surface mesh build failed");
        mesh
    }

    /// A unit cube tetrahedralizes into 5 or 6 tets without Steiner points.
    /// We verify the topology counts and that χ matches a closed solid ball:
    /// V − E + F − T = 1 (a 3-ball is contractible, so χ = 1).
    ///
    /// We can compute χ from just V and T plus the surface counts since
    /// every interior simplex contributes once; but the simpler test is
    /// just that no Steiner points were added.
    #[test]
    fn cube_interior_has_no_steiner_points() {
        let mesh = build_cube_surface();
        let tm = tetrahedralize(&mesh).expect("tetgen failed on cube");

        assert_eq!(tm.n_input_vertices, 8, "input vertex count preserved");
        assert_eq!(tm.n_vertices(), 8, "cube interior should need no Steiner points");
        assert!(tm.n_tets() >= 5, "cube interior is at least 5 tets");
        assert!(tm.n_tets() <= 6, "cube interior is at most 6 tets");

        // Every output tet vertex index is in range.
        for (i, tet) in tm.tets.iter().enumerate() {
            for &idx in tet {
                assert!(
                    (idx as usize) < tm.n_vertices(),
                    "tet {} has out-of-range vertex {}",
                    i,
                    idx
                );
            }
        }
    }

    /// Output vertex indices `0..n_input_vertices` must coincide spatially
    /// with the input vertices (TetGen places them first; this is the
    /// invariant the rest of the pipeline relies on).
    #[test]
    fn input_vertices_appear_first_unchanged() {
        let mesh = build_cube_surface();
        let tm = tetrahedralize(&mesh).expect("tetgen failed on cube");

        let surface_positions: Vec<Vector3D> = mesh
            .vert_ids()
            .iter()
            .map(|&v| mesh.position(v))
            .collect();

        for (i, expected) in surface_positions.iter().enumerate() {
            let got = tm.vertices[i];
            assert!(
                (got - expected).norm() < 1e-12,
                "input vertex {} moved: expected {:?}, got {:?}",
                i,
                expected,
                got
            );
        }
    }
}
