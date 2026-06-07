#[cfg(test)]
mod tests {
    use crate::{prelude::INPUT, skeleton::connectivity_surgery::tetrahedralize::tetrahedralize};
    use mehsh::{mesh::connectivity::HasPosition, prelude::Mesh, utils::primitives::Vector3D};

    /// Build a regular tetrahedron as a 4-vertex closed surface mesh.
    fn build_tetrahedron_surface() -> Mesh<INPUT> {
        let positions = vec![
            Vector3D::new(0.0, 0.0, 0.0),
            Vector3D::new(1.0, 0.0, 0.0),
            Vector3D::new(0.0, 1.0, 0.0),
            Vector3D::new(0.0, 0.0, 1.0),
        ];
        // Outward-pointing faces (right-hand rule from the centroid).
        let faces = vec![vec![0, 2, 1], vec![0, 1, 3], vec![0, 3, 2], vec![1, 2, 3]];
        let (mesh, _, _) =
            Mesh::<INPUT>::from(&faces, &positions).expect("tetrahedron surface mesh build failed");
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
            vec![0, 2, 1],
            vec![0, 3, 2],
            // top z=1 (normal +z)
            vec![4, 5, 6],
            vec![4, 6, 7],
            // front y=0 (normal -y)
            vec![0, 1, 5],
            vec![0, 5, 4],
            // back y=1 (normal +y)
            vec![3, 7, 6],
            vec![3, 6, 2],
            // left x=0 (normal -x)
            vec![0, 4, 7],
            vec![0, 7, 3],
            // right x=1 (normal +x)
            vec![1, 2, 6],
            vec![1, 6, 5],
        ];
        let (mesh, _, _) =
            Mesh::<INPUT>::from(&faces, &positions).expect("cube surface mesh build failed");
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
        assert_eq!(
            tm.n_vertices(),
            8,
            "cube interior should need no Steiner points"
        );
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

        let surface_positions: Vec<Vector3D> =
            mesh.vert_ids().iter().map(|&v| mesh.position(v)).collect();

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
