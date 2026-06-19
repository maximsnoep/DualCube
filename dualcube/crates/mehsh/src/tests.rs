use crate::prelude::*;
#[cfg(any(feature = "obj", feature = "stl"))]
use std::path::PathBuf;
define_tag!(TestMesh);

#[test]
fn from_manual() {
    let faces = vec![vec![0, 2, 1], vec![0, 1, 3], vec![1, 2, 3], vec![0, 3, 2]];
    let douconel = Mesh::<TestMesh>::from(&faces, &[Vector3D::new(0., 0., 0.); 4]);
    assert!(douconel.is_ok(), "{douconel:?}");
    if let Ok((douconel, _, _)) = douconel {
        assert!(douconel.nr_verts() == 4);
        assert!(douconel.nr_edges() == 6 * 2);
        assert!(douconel.nr_faces() == 4);

        for face_id in douconel.faces.ids() {
            assert!(douconel.vertices(face_id).count() == 3);
        }
    }
}

#[cfg(feature = "stl")]
#[test]
fn from_blub_stl() {
    let douconel = Mesh::<TestMesh>::from_stl(&PathBuf::from("assets/blub001k.stl"));
    assert!(douconel.is_ok(), "{douconel:?}");
    if let Ok((douconel, _, _)) = douconel {
        assert!(douconel.nr_verts() == 945);
        assert!(douconel.nr_edges() == 2829 * 2);
        assert!(douconel.nr_faces() == 1886);

        for face_id in douconel.faces.ids() {
            assert!(douconel.vertices(face_id).count() == 3);
        }
    }
}

#[cfg(feature = "obj")]
#[test]
fn from_blub_obj() {
    let douconel = Mesh::<TestMesh>::from_obj(&PathBuf::from("assets/blub001k.obj"));
    assert!(douconel.is_ok(), "{douconel:?}");
    if let Ok((douconel, _, _)) = douconel {
        assert!(douconel.nr_verts() == 945);
        assert!(douconel.nr_edges() == 2829 * 2);
        assert!(douconel.nr_faces() == 1886);

        for face_id in douconel.faces.ids() {
            assert!(douconel.vertices(face_id).count() == 3);
        }
    }
}

#[cfg(feature = "stl")]
#[test]
fn from_nefertiti_stl() {
    let douconel = Mesh::<TestMesh>::from_stl(&PathBuf::from("assets/nefertiti099k.stl"));
    assert!(douconel.is_ok(), "{douconel:?}");
    if let Ok((douconel, _, _)) = douconel {
        assert!(douconel.nr_verts() == 49971);
        assert!(douconel.nr_edges() == 149_907 * 2);
        assert!(douconel.nr_faces() == 99938);

        for face_id in douconel.faces.ids() {
            assert!(douconel.vertices(face_id).count() == 3);
        }
    }
}

#[cfg(feature = "obj")]
#[test]
fn from_hexahedron_obj() {
    let douconel = Mesh::<TestMesh>::from_obj(&PathBuf::from("assets/hexahedron.obj"));
    assert!(douconel.is_ok(), "{douconel:?}");
    if let Ok((douconel, _, _)) = douconel {
        assert!(douconel.nr_verts() == 8);
        assert!(douconel.nr_edges() == 4 * 6);
        assert!(douconel.nr_faces() == 6);

        for face_id in douconel.faces.ids() {
            assert!(douconel.vertices(face_id).count() == 4);
        }
    }
}

#[cfg(feature = "obj")]
#[test]
fn from_tetrahedron_obj() {
    let douconel = Mesh::<TestMesh>::from_obj(&PathBuf::from("assets/tetrahedron.obj"));
    assert!(douconel.is_ok(), "{douconel:?}");
    if let Ok((douconel, _, _)) = douconel {
        assert!(douconel.nr_verts() == 4);
        assert!(douconel.nr_edges() == 3 * 4);
        assert!(douconel.nr_faces() == 4);

        for face_id in douconel.faces.ids() {
            assert!(douconel.vertices(face_id).count() == 3);
        }
    }
}

#[test]
fn triangulate_hexahedron() {
    let (mesh, _, _) =
        Mesh::<TestMesh>::from(&hexahedron_faces(), &hexahedron_positions()).unwrap();
    assert!(mesh.is_triangular().is_err());

    let (triangulated, face_sources) = mesh.triangulate().unwrap();
    assert!(triangulated.is_triangular().is_ok());
    assert!(triangulated.nr_verts() == 8);
    assert!(triangulated.nr_edges() == 18 * 2);
    assert!(triangulated.nr_faces() == 12);
    assert!(face_sources.len() == triangulated.nr_faces());
}

#[test]
fn cut_and_cap_preserves_original_vertex_ids() {
    let (mesh, vertex_map, _) =
        Mesh::<TestMesh>::from(&hexahedron_faces(), &hexahedron_positions()).unwrap();

    let v0 = *vertex_map.key(0).unwrap();
    let v1 = *vertex_map.key(1).unwrap();
    let v2 = *vertex_map.key(2).unwrap();
    let v3 = *vertex_map.key(3).unwrap();
    let v4 = *vertex_map.key(4).unwrap();
    let v5 = *vertex_map.key(5).unwrap();
    let v6 = *vertex_map.key(6).unwrap();
    let v7 = *vertex_map.key(7).unwrap();

    let cut_loop = vec![v4, v5, v6, v7, v4];
    let cap_triangles = vec![[v4, v5, v6], [v4, v6, v7]];

    let (mesh_a, mesh_b) = mesh.cut_and_cap(&cut_loop, &cap_triangles).unwrap();
    let (top, bottom) = if mesh_a.nr_verts() == 4 {
        (mesh_a, mesh_b)
    } else {
        (mesh_b, mesh_a)
    };

    assert_eq!(top.nr_verts(), 4);
    assert_eq!(top.nr_faces(), 3);
    assert_eq!(bottom.nr_verts(), 8);
    assert_eq!(bottom.nr_faces(), 7);

    for vertex in [v4, v5, v6, v7] {
        assert!(top.verts.contains(vertex));
        assert!(bottom.verts.contains(vertex));
        assert_eq!(top.position(vertex), mesh.position(vertex));
        assert_eq!(bottom.position(vertex), mesh.position(vertex));
    }

    for vertex in [v0, v1, v2, v3] {
        assert!(!top.verts.contains(vertex));
        assert!(bottom.verts.contains(vertex));
        assert_eq!(bottom.position(vertex), mesh.position(vertex));
    }
}

#[test]
fn cut_and_cap_from_positions_stitches_boundary_and_adds_interior_vertices() {
    let (mesh, vertex_map, _) =
        Mesh::<TestMesh>::from(&hexahedron_faces(), &hexahedron_positions()).unwrap();

    let v4 = *vertex_map.key(4).unwrap();
    let v5 = *vertex_map.key(5).unwrap();
    let v6 = *vertex_map.key(6).unwrap();
    let v7 = *vertex_map.key(7).unwrap();
    let center = Vector3D::new(0.5, 1.0, 0.5);

    let mut cap_positions = vec![Vector3D::zeros(); 12];
    cap_positions[2] = mesh.position(v4);
    cap_positions[5] = mesh.position(v5);
    cap_positions[7] = mesh.position(v6);
    cap_positions[11] = mesh.position(v7);
    cap_positions[9] = center;

    let cut_loop = vec![v4, v5, v6, v7];
    let cap_triangles = vec![[2, 5, 9], [5, 7, 9], [7, 11, 9], [11, 2, 9]];

    let output = mesh
        .cut_and_cap_from_positions(&cut_loop, &cap_positions, &cap_triangles)
        .unwrap();

    assert_eq!(*output.cap_vertices_a.key(2).unwrap(), v4);
    assert_eq!(*output.cap_vertices_a.key(5).unwrap(), v5);
    assert_eq!(*output.cap_vertices_a.key(7).unwrap(), v6);
    assert_eq!(*output.cap_vertices_a.key(11).unwrap(), v7);
    assert_eq!(*output.cap_vertices_b.key(2).unwrap(), v4);
    assert_eq!(*output.cap_vertices_b.key(5).unwrap(), v5);
    assert_eq!(*output.cap_vertices_b.key(7).unwrap(), v6);
    assert_eq!(*output.cap_vertices_b.key(11).unwrap(), v7);

    let center_a = *output.cap_vertices_a.key(9).unwrap();
    let center_b = *output.cap_vertices_b.key(9).unwrap();
    assert!(!mesh.verts.contains(center_a));
    assert!(!mesh.verts.contains(center_b));
    assert_eq!(output.mesh_a.position(center_a), center);
    assert_eq!(output.mesh_b.position(center_b), center);

    assert!(output.mesh_a.verts.contains(v4));
    assert!(output.mesh_b.verts.contains(v4));
}

#[test]
fn triangulate_pentagonal_pyramid() {
    let positions = vec![
        Vector3D::new(1.0, 0.0, 0.0),
        Vector3D::new(0.309_016_994, 0.951_056_516, 0.0),
        Vector3D::new(-0.809_016_994, 0.587_785_252, 0.0),
        Vector3D::new(-0.809_016_994, -0.587_785_252, 0.0),
        Vector3D::new(0.309_016_994, -0.951_056_516, 0.0),
        Vector3D::new(0.0, 0.0, 1.0),
    ];
    let faces = vec![
        vec![0, 1, 2, 3, 4],
        vec![1, 0, 5],
        vec![2, 1, 5],
        vec![3, 2, 5],
        vec![4, 3, 5],
        vec![0, 4, 5],
    ];

    let (mesh, _, _) = Mesh::<TestMesh>::from(&faces, &positions).unwrap();
    let (triangulated, face_sources) = mesh.triangulate().unwrap();

    assert!(triangulated.is_triangular().is_ok());
    assert!(triangulated.nr_verts() == 6);
    assert!(triangulated.nr_edges() == 12 * 2);
    assert!(triangulated.nr_faces() == 8);
    assert!(face_sources.len() == triangulated.nr_faces());
}

#[test]
fn triangulate_concave_prism() {
    let (mesh, _, _) =
        Mesh::<TestMesh>::from(&concave_prism_faces(), &concave_prism_positions()).unwrap();
    let (triangulated, _) = mesh.triangulate().unwrap();

    assert!(triangulated.is_triangular().is_ok());
    assert!(triangulated.nr_verts() == 12);
    assert!(triangulated.nr_edges() == 30 * 2);
    assert!(triangulated.nr_faces() == 20);
}

fn hexahedron_faces() -> Vec<Vec<usize>> {
    vec![
        vec![0, 1, 2, 3],
        vec![7, 6, 5, 4],
        vec![0, 4, 5, 1],
        vec![1, 5, 6, 2],
        vec![2, 6, 7, 3],
        vec![3, 7, 4, 0],
    ]
}

fn hexahedron_positions() -> Vec<Vector3D> {
    vec![
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(1.0, 0.0, 0.0),
        Vector3D::new(1.0, 0.0, 1.0),
        Vector3D::new(0.0, 0.0, 1.0),
        Vector3D::new(0.0, 1.0, 0.0),
        Vector3D::new(1.0, 1.0, 0.0),
        Vector3D::new(1.0, 1.0, 1.0),
        Vector3D::new(0.0, 1.0, 1.0),
    ]
}

fn concave_prism_faces() -> Vec<Vec<usize>> {
    vec![
        vec![5, 4, 3, 2, 1, 0],
        vec![6, 7, 8, 9, 10, 11],
        vec![0, 1, 7, 6],
        vec![1, 2, 8, 7],
        vec![2, 3, 9, 8],
        vec![3, 4, 10, 9],
        vec![4, 5, 11, 10],
        vec![5, 0, 6, 11],
    ]
}

fn concave_prism_positions() -> Vec<Vector3D> {
    vec![
        Vector3D::new(0.0, 0.0, 0.0),
        Vector3D::new(2.0, 0.0, 0.0),
        Vector3D::new(2.0, 1.0, 0.0),
        Vector3D::new(1.0, 1.0, 0.0),
        Vector3D::new(1.0, 2.0, 0.0),
        Vector3D::new(0.0, 2.0, 0.0),
        Vector3D::new(0.0, 0.0, 1.0),
        Vector3D::new(2.0, 0.0, 1.0),
        Vector3D::new(2.0, 1.0, 1.0),
        Vector3D::new(1.0, 1.0, 1.0),
        Vector3D::new(1.0, 2.0, 1.0),
        Vector3D::new(0.0, 2.0, 1.0),
    ]
}
