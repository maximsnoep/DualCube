use crate::dual::{Dual, LoopRegionID, Orientation};
use crate::layout::Layout;
use crate::polycube::Polycube;
use crate::EmbeddedMesh;
use bimap::BiHashMap;
use douconel::douconel::{Douconel, Empty};
use douconel::douconel_embedded::{EmbeddedVertex, HasPosition};
use faer::sparse::{SparseColMat, Triplet};
use faer::{mat, Mat};
use faer_gmres::gmres;
use hutspot::geom::{Vector2D, Vector3D};
use itertools::Itertools;
use sprs::{CsMat, CsVec, TriMat};
use sprs_ldl::Ldl;
use std::collections::{HashMap, HashSet};
use std::hash::Hash;

slotmap::new_key_type! {
    pub struct QuadVertID;
    pub struct QuadEdgeID;
    pub struct QuadFaceID;
}

type QuadMesh = Douconel<QuadVertID, EmbeddedVertex, QuadEdgeID, Empty, QuadFaceID, Empty>;

#[derive(Clone, Debug)]
pub struct Quad {
    pub triangle_mesh_polycube: EmbeddedMesh,
    pub structure: QuadMesh,
}

impl Quad {
    fn arc_length_parameterization(verts: &[Vector3D]) -> Vec<f64> {
        // Arc-length parameterization of list of points to [0, 1] interval
        let distances = [0.0].into_iter().chain(verts.windows(2).map(|w| (w[1] - w[0]).norm())).collect_vec();
        let total_length = distances.iter().sum::<f64>();

        distances
            .into_iter()
            .scan(0.0, |acc, d| {
                *acc += d;
                Some(*acc / total_length)
            })
            .collect_vec()
    }

    pub fn from_layout(layout: &Layout, polycube: &Polycube) -> Self {
        let mut triangle_mesh_polycube = layout.granulated_mesh.clone();

        // For every patch in the layout (corresponding to a face in the polycube), we map this patch to a unit square
        // 1. Map the boundary of the patch to the boundary of a unit square via arc-length parameterization
        // 2. Map the interior of the patch to the interior of the unit square via mean-value coordinates (MVC)
        let polycube = &polycube.structure;
        let patch_ids = polycube.face_ids();
        for &patch_id in &patch_ids {
            // A map for each vertex in the patch to its corresponding 2D coordinate in the unit square
            let mut map_to_2d = HashMap::new();

            let patch_edges = polycube.edges(patch_id);

            // Edge1 is mapped to unit edge (0,1) -> (1,1)
            let edge1 = patch_edges[0];
            let boundary1 = layout.edge_to_path.get(&edge1).unwrap();
            let parameterization1 = Self::arc_length_parameterization(&boundary1.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary1.iter().enumerate() {
                let mapped_pos = Vector2D::new(parameterization1[i], 1.0);
                map_to_2d.insert(v, mapped_pos);
            }

            // Edge2 is mapped to unit edge (1,1) -> (1,0)
            let edge2 = patch_edges[1];
            let boundary2 = layout.edge_to_path.get(&edge2).unwrap();
            let parameterization2 = Self::arc_length_parameterization(&boundary2.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary2.iter().enumerate() {
                let mapped_pos = Vector2D::new(1.0, 1.0 - parameterization2[i]);
                map_to_2d.insert(v, mapped_pos);
            }

            // Edge3 is mapped to unit edge (1,0) -> (0,0)
            let edge3 = patch_edges[2];
            let boundary3 = layout.edge_to_path.get(&edge3).unwrap();
            let parameterization3 = Self::arc_length_parameterization(&boundary3.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary3.iter().enumerate() {
                let mapped_pos = Vector2D::new(1.0 - parameterization3[i], 0.0);
                map_to_2d.insert(v, mapped_pos);
            }

            // Edge4 is mapped to unit edge (0,0) -> (0,1)
            let edge4 = patch_edges[3];
            let boundary4 = layout.edge_to_path.get(&edge4).unwrap();
            let parameterization4 = Self::arc_length_parameterization(&boundary4.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary4.iter().enumerate() {
                let mapped_pos = Vector2D::new(0.0, parameterization4[i]);
                map_to_2d.insert(v, mapped_pos);
            }

            // d3p1 = (x1, y1, z1)
            // d3p2 = (x2, y2, z2)
            // d3p3 = (x3, y3, z3)
            // d3p4 = (x4, y4, z4)
            // Figure out which coordinate is constant.
            // Then map
            // d2p1 = (u1, v1)
            // d2p2 = (u2, v1)
            // d2p3 = (u2, v2)
            // d2p4 = (u1, v2)
            let p1 = polycube.endpoints(edge1).0;
            let d3p1 = polycube.position(p1);
            let p2 = polycube.endpoints(edge2).0;
            let d3p2 = polycube.position(p2);
            let p3 = polycube.endpoints(edge3).0;
            let d3p3 = polycube.position(p3);
            let p4 = polycube.endpoints(edge4).0;
            let d3p4 = polycube.position(p4);

            let coordinates = if d3p1.x == d3p2.x && d3p1.x == d3p3.x && d3p1.x == d3p4.x {
                (1, 2, 0)
            } else if d3p1.y == d3p2.y && d3p1.y == d3p3.y && d3p1.y == d3p4.y {
                (0, 2, 1)
            } else if d3p1.z == d3p2.z && d3p1.z == d3p3.z && d3p1.z == d3p4.z {
                (0, 1, 2)
            } else {
                panic!("The face is not axis-aligned!")
            };

            let d2p1 = Vector2D::new(d3p1[coordinates.0], d3p1[coordinates.1]);
            let d2p2 = Vector2D::new(d3p2[coordinates.0], d3p2[coordinates.1]);
            let d2p3 = Vector2D::new(d3p3[coordinates.0], d3p3[coordinates.1]);
            let d2p4 = Vector2D::new(d3p4[coordinates.0], d3p4[coordinates.1]);

            // Interpolate the vertices on the edges (paths) between p1 and p2, p2 and p3, p3 and p4, p4 and p1
            // Parameterize (using arc-length) the vertices
            // From p1 to p2
            let interpolation1 = Self::arc_length_parameterization(&boundary1.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary1.iter().enumerate() {
                let mapped_pos = d2p1 * (1.0 - interpolation1[i]) + d2p2 * interpolation1[i];
                map_to_2d.insert(v, mapped_pos);
            }
            // From p2 to p3
            let interpolation2 = Self::arc_length_parameterization(&boundary2.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary2.iter().enumerate() {
                let mapped_pos = d2p2 * (1.0 - interpolation2[i]) + d2p3 * interpolation2[i];
                map_to_2d.insert(v, mapped_pos);
            }
            // From p3 to p4
            let interpolation3 = Self::arc_length_parameterization(&boundary3.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary3.iter().enumerate() {
                let mapped_pos = d2p3 * (1.0 - interpolation3[i]) + d2p4 * interpolation3[i];
                map_to_2d.insert(v, mapped_pos);
            }
            // From p4 to p1
            let interpolation4 = Self::arc_length_parameterization(&boundary4.iter().map(|&v| layout.granulated_mesh.position(v)).collect_vec());
            for (i, &v) in boundary4.iter().enumerate() {
                let mapped_pos = d2p4 * (1.0 - interpolation4[i]) + d2p1 * interpolation4[i];
                map_to_2d.insert(v, mapped_pos);
            }

            // Now we have the boundary of the patch mapped to the unit square
            // We need to map the interior of the patch to the interior of the unit square
            // We can use mean-value coordinates (MVC) to do this

            let all_verts = layout
                .face_to_patch
                .get(&patch_id)
                .unwrap()
                .faces
                .iter()
                .flat_map(|&face_id| layout.granulated_mesh.corners(face_id))
                .collect::<HashSet<_>>();

            let interior_verts = all_verts.iter().filter(|&&v| !map_to_2d.contains_key(&v)).copied().collect_vec();

            let mut vert_to_id = BiHashMap::new();
            for (i, &v) in interior_verts.iter().enumerate() {
                vert_to_id.insert(v, i);
            }

            let n = interior_verts.len();
            let mut triplets = Vec::new();
            let mut bu = vec![0.0; n];
            let mut bv = vec![0.0; n];
            for i in 0..n {
                triplets.push((i, i, 1.0));
            }

            for &v0 in &interior_verts {
                let row = vert_to_id.get_by_left(&v0).unwrap().to_owned();
                // For vi (all neighbors of v0), we calculate weight wi, where wi = tan(alpha_{i-1} / 2) + tan(alpha_i / 2) / || vi - v0 ||

                let neighbors = layout.granulated_mesh.vneighbors(v0);
                let k = neighbors.len();

                let w = (0..k)
                    .map(|i| {
                        let vip1 = neighbors[(i + 1) % k];
                        let eip1 = layout.granulated_mesh.edge_between_verts(v0, vip1).unwrap().0;
                        let vi = neighbors[i];
                        let ei = layout.granulated_mesh.edge_between_verts(v0, vi).unwrap().0;
                        let vim1 = neighbors[(i + k - 1) % k];
                        let eim1 = layout.granulated_mesh.edge_between_verts(v0, vim1).unwrap().0;

                        let alpha_im1 = layout.granulated_mesh.angle(eim1, ei);
                        let alpha_i = layout.granulated_mesh.angle(ei, eip1);
                        let len_ei = (layout.granulated_mesh.position(v0) - layout.granulated_mesh.position(vi)).norm();

                        ((alpha_im1 / 2.0).tan() + (alpha_i / 2.0).tan()) / len_ei
                    })
                    .collect_vec();

                let sum_w = w.iter().sum::<f64>();

                let weights = w.iter().map(|&wi| wi / sum_w).collect_vec();

                let sum_weights: f64 = weights.iter().sum();
                if (sum_weights - 1.0).abs() > 1e-6 {
                    println!("Warning: weights sum to {} instead of 1.0", sum_weights);
                }

                for i in 0..k {
                    let vi = neighbors[i];
                    let is_boundary = map_to_2d.contains_key(&vi);

                    if is_boundary {
                        let mapped_pos = map_to_2d.get(&vi).unwrap().to_owned();
                        bu[row] += weights[i] * mapped_pos.x;
                        bv[row] += weights[i] * mapped_pos.y;
                    } else {
                        let col = vert_to_id.get_by_left(&vi).unwrap().to_owned();
                        triplets.push((row, col, -weights[i]));
                    }
                }
            }

            let b_u = Mat::from_fn(n, 1, |i, _| bu[i]);
            let b_v = Mat::from_fn(n, 1, |i, _| bv[i]);

            assert!(bu.iter().all(|v| v.is_finite()), "bu contains NaN or Inf");
            assert!(bv.iter().all(|v| v.is_finite()), "bv contains NaN or Inf");

            let mut x_u = Mat::from_fn(n, 1, |_, _| 0.0);
            let mut x_v = Mat::from_fn(n, 1, |_, _| 0.0);

            let faer_triplets = triplets.into_iter().map(|(i, j, v)| Triplet::new(i, j, v)).collect::<Vec<_>>();

            for triplet in &faer_triplets {
                assert!(triplet.val.is_finite(), "Triplet contains NaN or Inf: {:?}", triplet);
            }

            let a = SparseColMat::<usize, f64>::try_new_from_triplets(n, n, &faer_triplets).unwrap();

            if !interior_verts.is_empty() {
                let (err_u, iter_u) = gmres(a.as_ref(), b_u.as_ref(), x_u.as_mut(), 100, 1e-8, None).unwrap();
                let (err_v, iter_v) = gmres(a.as_ref(), b_v.as_ref(), x_v.as_mut(), 100, 1e-8, None).unwrap();
            }

            for v in all_verts {
                let is_boundary = map_to_2d.contains_key(&v);
                let position = if is_boundary {
                    let mapped_pos = map_to_2d.get(&v).unwrap().to_owned();
                    match coordinates {
                        (1, 2, 0) => Vector3D::new(d3p1.x, mapped_pos.x, mapped_pos.y),
                        (0, 2, 1) => Vector3D::new(mapped_pos.x, d3p1.y, mapped_pos.y),
                        (0, 1, 2) => Vector3D::new(mapped_pos.x, mapped_pos.y, d3p1.z),
                        _ => panic!("Invalid coordinates"),
                    }
                } else {
                    let mapped_pos = Vector2D::new(
                        x_u[(vert_to_id.get_by_left(&v).unwrap().to_owned(), 0)],
                        x_v[(vert_to_id.get_by_left(&v).unwrap().to_owned(), 0)],
                    );
                    match coordinates {
                        (1, 2, 0) => Vector3D::new(d3p1.x, mapped_pos.x, mapped_pos.y),
                        (0, 2, 1) => Vector3D::new(mapped_pos.x, d3p1.y, mapped_pos.y),
                        (0, 1, 2) => Vector3D::new(mapped_pos.x, mapped_pos.y, d3p1.z),
                        _ => panic!("Invalid coordinates"),
                    }
                };

                triangle_mesh_polycube.verts[v].set_position(position);
            }
        }

        Quad {
            triangle_mesh_polycube,
            structure: QuadMesh::default(),
        }
    }
}
