use crate::prelude::*;
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
