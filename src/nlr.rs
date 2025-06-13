use crate::{
    polycube::{PolycubeEdgeID, PolycubeFaceID, PolycubeVertID},
    solutions::Solution,
    to_principal_direction, PrincipalDirection,
};
use bimap::BiHashMap;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use log::info;
use mehsh::prelude::*;
use ordered_float::OrderedFloat;
use std::io::Write;
use std::path::Path;

impl Solution {
    pub fn export_to_nlr(&self, path: &Path) -> std::io::Result<()> {
        let path_topol = path.with_extension("topol");
        let path_geom = path.with_extension("geom");
        let path_cdim = path.with_extension("cdim");
        let path_bcdat = path.with_extension("bcdat");

        if let (Ok(dual), Ok(layout), Some(polycube)) = (&self.dual, &self.layout, &self.polycube) {
            let signature = " -- automatically generated via the DualLoops algorithm";

            // Also add farfield description (box around the model) with size mult*a x mult*b x mult*c (if whole model is a x b x c)
            let mult = 10.;

            // Find the most front vertex (reference vertex), which is the vertex with the smallest x-coordinate
            let refv = polycube
                .structure
                .vert_ids()
                .into_iter()
                .min_by_key(|&vert| OrderedFloat(polycube.structure.position(vert).x))
                .unwrap();

            let (min_x, max_x, min_y, max_y, min_z, max_z) = (
                dual.mesh_ref
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(dual.mesh_ref.position(v).x))
                    .min()
                    .unwrap()
                    .0,
                dual.mesh_ref
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(dual.mesh_ref.position(v).x))
                    .max()
                    .unwrap()
                    .0,
                dual.mesh_ref
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(dual.mesh_ref.position(v).y))
                    .min()
                    .unwrap()
                    .0,
                dual.mesh_ref
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(dual.mesh_ref.position(v).y))
                    .max()
                    .unwrap()
                    .0,
                dual.mesh_ref
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(dual.mesh_ref.position(v).z))
                    .min()
                    .unwrap()
                    .0,
                dual.mesh_ref
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(dual.mesh_ref.position(v).z))
                    .max()
                    .unwrap()
                    .0,
            );

            let delta_x = Vector3D::new(max_x - min_x, 0., 0.);
            let mid_x = (min_x + max_x) / 2.;
            let delta_y = Vector3D::new(0., max_y - min_y, 0.);
            let mid_y = (min_y + max_y) / 2.;
            let delta_z = Vector3D::new(0., 0., max_z - min_z);
            let mid_z = (min_z + max_z) / 2.;

            let (p_min_x, p_max_x, p_min_y, p_max_y, p_min_z, p_max_z) = (
                polycube
                    .structure
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(polycube.structure.position(v).x))
                    .min()
                    .unwrap()
                    .0,
                polycube
                    .structure
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(polycube.structure.position(v).x))
                    .max()
                    .unwrap()
                    .0,
                polycube
                    .structure
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(polycube.structure.position(v).y))
                    .min()
                    .unwrap()
                    .0,
                polycube
                    .structure
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(polycube.structure.position(v).y))
                    .max()
                    .unwrap()
                    .0,
                polycube
                    .structure
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(polycube.structure.position(v).z))
                    .min()
                    .unwrap()
                    .0,
                polycube
                    .structure
                    .vert_ids()
                    .into_iter()
                    .map(|v| OrderedFloat(polycube.structure.position(v).z))
                    .max()
                    .unwrap()
                    .0,
            );

            let p_delta_x = p_max_x - p_min_x;
            let p_mid_x = (p_min_x + p_max_x) / 2.;
            let p_delta_y = p_max_y - p_min_y;
            let p_mid_y = (p_min_y + p_max_y) / 2.;
            let p_delta_z = p_max_z - p_min_z;
            let p_mid_z = (p_min_z + p_max_z) / 2.;

            let v1 = Vector3D::new(mid_x, mid_y, mid_z) - mult * delta_x - mult * delta_y - mult * delta_z;
            let v2 = v1 + 2. * mult * delta_z;
            let v3 = v2 + 2. * mult * delta_y;
            let v4 = v3 - 2. * mult * delta_z;

            let v5 = v1 + 2. * mult * delta_x;
            let v6 = v5 + 2. * mult * delta_z;
            let v7 = v6 + 2. * mult * delta_y;
            let v8 = v7 - 2. * mult * delta_z;

            assert!(polycube.structure.verts.len() < 10000);
            let vert_to_id: BiHashMap<PolycubeVertID, usize> = polycube.structure.vert_ids().iter().enumerate().map(|(i, &id)| (id, 10001 + i)).collect();

            assert!(polycube.structure.edges.len() < 10000);
            let edge_to_id: BiHashMap<PolycubeEdgeID, usize> = polycube
                .structure
                .edge_ids()
                .iter()
                .filter(|&&edge_id| edge_id < polycube.structure.twin(edge_id))
                .enumerate()
                .map(|(i, &id)| (id, 20001 + i))
                .collect();

            assert!(polycube.structure.faces.len() < 10000);
            let face_to_id: BiHashMap<PolycubeFaceID, usize> = polycube.structure.face_ids().iter().enumerate().map(|(i, &id)| (id, 30001 + i)).collect();

            // ------------------------
            // --- WRITE TOPOL FILE ---
            //
            //
            //
            info!("Writing TOPOL file to {path_topol:?}");
            let mut file_topol = std::fs::File::create(path_topol)?;

            // Write info (should apparently be 5 lines)
            write!(file_topol, "'topol file <> {signature}'\n'2nd line'\n'3rd line'\n'4th line'\n'5th line'")?;

            // Write blocks and compound blocks (we do not define any)
            write!(file_topol, "\n NUMBER OF BLOCKS:\n       0\n NUMBER OF COMPOUND BLOCKS:\n       0")?;

            // Write faces
            write!(
                file_topol,
                "\n NUMBER OF ELEMENTARY FACES:\n       {}\n        FACE       EDGE1       EDGE2       EDGE3       EDGE4       IDENT\n",
                polycube.structure.face_ids().len() + 6
            )?;
            write!(
                file_topol,
                "{}",
                polycube
                    .structure
                    .face_ids()
                    .iter()
                    .map(|face_id| {
                        let face_int = face_to_id.get_by_left(face_id).unwrap();
                        let edges = polycube.structure.edges(*face_id);
                        let edge_int1 = edge_to_id
                            .get_by_left(&edges[0])
                            .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edges[0])))
                            .unwrap();
                        let edge_int2 = edge_to_id
                            .get_by_left(&edges[2])
                            .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edges[2])))
                            .unwrap();
                        let edge_int3 = edge_to_id
                            .get_by_left(&edges[1])
                            .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edges[1])))
                            .unwrap();
                        let edge_int4 = edge_to_id
                            .get_by_left(&edges[3])
                            .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edges[3])))
                            .unwrap();
                        format!("       {face_int}       {edge_int1}       {edge_int2}       {edge_int3}       {edge_int4}       'FACE'")
                    })
                    .collect::<Vec<_>>()
                    .join("\n")
            )?;
            // Add the bounding box faces
            write!(file_topol, "\n       39001       29012       29034       29023       29041       'FACE'")?;
            write!(file_topol, "\n       39002       29056       29078       29067       29085       'FACE'")?;
            write!(file_topol, "\n       39003       29015       29048       29041       29085       'FACE'")?;
            write!(file_topol, "\n       39004       29026       29037       29023       29067       'FACE'")?;
            write!(file_topol, "\n       39005       29012       29056       29015       29026       'FACE'")?;
            write!(file_topol, "\n       39006       29048       29037       29078       29034       'FACE'")?;

            write!(file_topol, "\n NUMBER OF COMPOUND FACES:\n       0\n")?;

            // Write edges
            write!(
                file_topol,
                " NUMBER OF ELEMENTARY EDGES:\n       {}\n        EDGE       VERT1       VERT2       IDENT\n",
                (polycube.structure.edge_ids().len() / 2) + 12
            )?;
            write!(
                file_topol,
                "{}",
                polycube
                    .structure
                    .edge_ids()
                    .iter()
                    .filter_map(|edge_id| {
                        edge_to_id.get_by_left(edge_id).map(|edge_int| {
                            let verts = polycube.structure.vertices(*edge_id);
                            let vert_int1 = vert_to_id.get_by_left(&verts[0]).unwrap();
                            let vert_int2 = vert_to_id.get_by_left(&verts[1]).unwrap();
                            format!("       {edge_int}       {vert_int1}       {vert_int2}       'EDGE'")
                        })
                    })
                    .collect::<Vec<_>>()
                    .join("\n")
            )?;
            // Add the bounding box edges
            write!(file_topol, "\n       29012       19001       19002       'EDGE'")?;
            write!(file_topol, "\n       29023       19002       19003       'EDGE'")?;
            write!(file_topol, "\n       29034       19003       19004       'EDGE'")?;
            write!(file_topol, "\n       29041       19004       19001       'EDGE'")?;

            write!(file_topol, "\n       29056       19005       19006       'EDGE'")?;
            write!(file_topol, "\n       29067       19006       19007       'EDGE'")?;
            write!(file_topol, "\n       29078       19007       19008       'EDGE'")?;
            write!(file_topol, "\n       29085       19008       19005       'EDGE'")?;

            write!(file_topol, "\n       29015       19001       19005       'EDGE'")?;
            write!(file_topol, "\n       29026       19002       19006       'EDGE'")?;
            write!(file_topol, "\n       29037       19003       19007       'EDGE'")?;
            write!(file_topol, "\n       29048       19004       19008       'EDGE'")?;

            write!(file_topol, "\n NUMBER OF COMPOUND EDGES:\n       0")?;

            info!("Finished writing TOPOL file");

            // -----------------------
            // --- WRITE GEOM FILE ---
            //
            //
            //
            info!("Writing GEOM file to {path_geom:?}");
            let mut file_geom = std::fs::File::create(path_geom)?;
            write!(file_geom, "'geom file <> {signature}'")?;

            // Write all verts
            write!(
                file_geom,
                "\n NUMBER OF VERTICES:\n       {}\n        VERT       X Y Z                     IDENT\n",
                polycube.structure.vert_ids().len() + 8
            )?;
            write!(
                file_geom,
                "{}",
                polycube
                    .structure
                    .vert_ids()
                    .iter()
                    .map(|vert_id| {
                        let edge_id = polycube.structure.edges(*vert_id)[0];
                        let path = layout.edge_to_path.get(&edge_id).unwrap();
                        let first_vertex = path[0];
                        let vert_int = vert_to_id.get_by_left(vert_id).unwrap();
                        let pos = layout.granulated_mesh.position(first_vertex);
                        format!(
                            "       {}       {}  {}  {}       'VERTEX'",
                            vert_int,
                            ryu::Buffer::new().format(pos.x),
                            ryu::Buffer::new().format(pos.y),
                            ryu::Buffer::new().format(pos.z)
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("\n")
            )?;
            write!(
                file_geom,
                "\n       19001       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v1.x),
                ryu::Buffer::new().format(v1.y),
                ryu::Buffer::new().format(v1.z)
            )?;
            write!(
                file_geom,
                "\n       19002       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v2.x),
                ryu::Buffer::new().format(v2.y),
                ryu::Buffer::new().format(v2.z)
            )?;
            write!(
                file_geom,
                "\n       19003       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v3.x),
                ryu::Buffer::new().format(v3.y),
                ryu::Buffer::new().format(v3.z)
            )?;
            write!(
                file_geom,
                "\n       19004       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v4.x),
                ryu::Buffer::new().format(v4.y),
                ryu::Buffer::new().format(v4.z)
            )?;
            write!(
                file_geom,
                "\n       19005       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v5.x),
                ryu::Buffer::new().format(v5.y),
                ryu::Buffer::new().format(v5.z)
            )?;
            write!(
                file_geom,
                "\n       19006       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v6.x),
                ryu::Buffer::new().format(v6.y),
                ryu::Buffer::new().format(v6.z)
            )?;
            write!(
                file_geom,
                "\n       19007       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v7.x),
                ryu::Buffer::new().format(v7.y),
                ryu::Buffer::new().format(v7.z)
            )?;
            write!(
                file_geom,
                "\n       19008       {}  {}  {}       'VERTEX'",
                ryu::Buffer::new().format(v8.x),
                ryu::Buffer::new().format(v8.y),
                ryu::Buffer::new().format(v8.z)
            )?;

            // Write all edges
            write!(
                file_geom,
                "\n NUMBER OF EDGES:\n       {}\n        EDGE       CONTROL POINTS\n",
                polycube.structure.edge_ids().len() / 2
            )?;
            write!(
                file_geom,
                "{}",
                polycube
                    .structure
                    .edge_ids()
                    .iter()
                    .filter_map(|edge_id| {
                        edge_to_id.get_by_left(edge_id).map(|edge_int| {
                            let path = layout.edge_to_path.get(edge_id).unwrap();
                            format!("       {}       {}\n", edge_int, path.len())
                                + &path
                                    .iter()
                                    .map(|&point| {
                                        let pos = layout.granulated_mesh.position(point);
                                        format!(
                                            "  {}  {}  {}",
                                            ryu::Buffer::new().format(pos.x),
                                            ryu::Buffer::new().format(pos.y),
                                            ryu::Buffer::new().format(pos.z)
                                        )
                                    })
                                    .collect::<Vec<_>>()
                                    .join("\n")
                        })
                    })
                    .collect::<Vec<_>>()
                    .join("\n")
            )?;

            write!(file_geom, "\n NUMBER OF FACES:\n       0")?;
            info!("Finished writing GEOM file");

            // -----------------------
            // --- WRITE CDIM FILE ---
            //
            //
            //
            info!("Writing CDIM file to {path_cdim:?}");
            let mut file_cdim = std::fs::File::create(path_cdim)?;
            write!(file_cdim, "'cdim file <> {signature}'")?;

            // Write all edge lengths
            let loops = self.loops.keys();
            let edge_per_loop = loops
                .map(|loop_id| {
                    dual.loop_structure
                        .edge_ids()
                        .into_iter()
                        .find(|&segment_id| dual.segment_to_loop(segment_id) == loop_id)
                        .unwrap()
                })
                .map(|segment_id| dual.loop_structure.faces(segment_id))
                .map(|face| [face[0], face[1]])
                .map(|[region1, region2]| {
                    (
                        polycube.region_to_vertex.get_by_left(&region1).unwrap().to_owned(),
                        polycube.region_to_vertex.get_by_left(&region2).unwrap().to_owned(),
                    )
                })
                .map(|(vertex1, vertex2)| polycube.structure.edge_between_verts(vertex1, vertex2).unwrap().0);
            write!(
                file_cdim,
                "\n NUMBER OF USER SPECIFIED EDGE DIMENSIONS:\n       {}\n        EDGE       DIM\n",
                edge_per_loop.clone().count() + 3
            )?;
            write!(
                file_cdim,
                "{}",
                edge_per_loop
                    .clone()
                    .map(|edge_id| {
                        let edge_int = edge_to_id
                            .get_by_left(&edge_id)
                            .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edge_id)))
                            .unwrap();
                        let length = polycube.structure.size(edge_id) as usize;
                        format!("       {edge_int}       {length}")
                    })
                    .collect::<Vec<_>>()
                    .join("\n")
            )?;
            let (cartesian_x, cartesian_y, cartesian_z) = ((mult * p_delta_x) as usize, (mult * p_delta_y) as usize, (mult * p_delta_z) as usize);
            write!(file_cdim, "\n       29015       {}", cartesian_x * 2)?;
            write!(file_cdim, "\n       29041       {}", cartesian_x * 2)?;
            write!(file_cdim, "\n       29012       {}", cartesian_x * 2)?;

            // Write grid levels
            write!(file_cdim, "\n GRID LEVEL OF BASIC GRID AND COMPUTATIONAL GRID:\n       1 1")?;

            // Write refinement
            write!(file_cdim, "\n NUMBER OF BLOCKS WITH LOCAL GRID REFINEMENT:\n       0")?;

            // Write edges in x (i) direction
            let x_edges = edge_per_loop
                .clone()
                .filter(|&edge_id| to_principal_direction(polycube.structure.vector(edge_id)).0 == PrincipalDirection::X)
                .map(|edge_id| {
                    edge_to_id
                        .get_by_left(&edge_id)
                        .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edge_id)))
                        .unwrap()
                })
                .collect_vec();
            write!(
                file_cdim,
                "\n NUMBER OF EDGES IN I-DIRECTION IN CARTESIAN SPACE:\n       {}\n  EDGES:\n  ",
                x_edges.len() + 1
            )?;
            write!(
                file_cdim,
                "{}",
                x_edges.iter().map(|edge_id| format!("  {edge_id}")).collect::<Vec<_>>().join("  ")
            )?;
            write!(file_cdim, "  29015")?;

            // Write edges in y (j) direction
            let y_edges = edge_per_loop
                .clone()
                .filter(|&edge_id| to_principal_direction(polycube.structure.vector(edge_id)).0 == PrincipalDirection::Y)
                .map(|edge_id| {
                    edge_to_id
                        .get_by_left(&edge_id)
                        .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edge_id)))
                        .unwrap()
                })
                .collect_vec();
            write!(
                file_cdim,
                "\n NUMBER OF EDGES IN J-DIRECTION IN CARTESIAN SPACE:\n       {}\n  EDGES:\n  ",
                y_edges.len() + 1
            )?;
            write!(
                file_cdim,
                "{}",
                y_edges.iter().map(|edge_id| format!("  {edge_id}")).collect::<Vec<_>>().join("  ")
            )?;
            write!(file_cdim, "  29041")?;

            // Write edges in z (k) direction
            let z_edges = edge_per_loop
                .clone()
                .filter(|&edge_id| to_principal_direction(polycube.structure.vector(edge_id)).0 == PrincipalDirection::Z)
                .map(|edge_id| {
                    edge_to_id
                        .get_by_left(&edge_id)
                        .or_else(|| edge_to_id.get_by_left(&polycube.structure.twin(edge_id)))
                        .unwrap()
                })
                .collect_vec();
            write!(
                file_cdim,
                "\n NUMBER OF EDGES IN K-DIRECTION IN CARTESIAN SPACE:\n       {}\n  EDGES:\n  ",
                z_edges.len() + 1
            )?;
            write!(
                file_cdim,
                "{}",
                z_edges.iter().map(|edge_id| format!("  {edge_id}")).collect::<Vec<_>>().join("  ")
            )?;
            write!(file_cdim, "  29012")?;

            // Write reference (origin) vertex
            write!(file_cdim, "\n NUMBER OF VERTICES WITH CARTESIAN COORDINATES:\n       2\n        VERT i j k\n")?;
            write!(file_cdim, "       {} 0 0 0\n", vert_to_id.get_by_left(&refv).unwrap())?;
            write!(file_cdim, "       19001 -{cartesian_x} -{cartesian_x} -{cartesian_x}\n")?;

            // Write symmetry and orientation
            write!(file_cdim, "SYMMETRY\n       0\nORIENTATION\n       0")?;
            info!("Finished writing CDIM file");

            // ------------------------
            // --- WRITE BCDAT FILE ---
            //
            //
            //
            info!("Writing BCDAT file to {path_bcdat:?}");
            let mut file_bcdat = std::fs::File::create(path_bcdat)?;
            write!(
                file_bcdat,
                "'cdim file <> {signature}'\n\n        15  'NaS solid wall'        {}\n",
                polycube.structure.faces.len()
            )?;
            write!(
                file_bcdat,
                "{}",
                polycube
                    .structure
                    .face_ids()
                    .into_iter()
                    .map(|face_id| format!("  {}", face_to_id.get_by_left(&face_id).unwrap()))
                    .collect::<Vec<_>>()
                    .join("  ")
            )?;
            info!("Finished writing BCDAT file");
        }

        Ok(())
    }
}
