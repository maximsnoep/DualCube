use crate::dual::{Dual, LoopRegionID};
use crate::layout::Layout;
use crate::PrincipalDirection;
use bimap::BiHashMap;
use itertools::Itertools;
use mehsh::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

mehsh::prelude::define_tag!(POLYCUBE);
pub type PolycubeVertID = mehsh::prelude::VertKey<POLYCUBE>;
pub type PolycubeEdgeID = mehsh::prelude::EdgeKey<POLYCUBE>;
pub type PolycubeFaceID = mehsh::prelude::FaceKey<POLYCUBE>;
type PolycubeMesh = mehsh::prelude::Mesh<POLYCUBE>;

#[derive(Clone, Debug)]
pub struct Polycube {
    pub structure: PolycubeMesh,

    // Mapping from dual to primal
    pub region_to_vertex: BiHashMap<LoopRegionID, PolycubeVertID>,
}

impl Polycube {
    pub fn from_dual(dual: &Dual) -> Self {
        let primal_vertices = dual.loop_structure.face_ids();
        let primal_faces = dual.loop_structure.vert_ids();

        let mut region_to_vertex = BiHashMap::new();

        // Each face to an int
        let vert_to_int: HashMap<LoopRegionID, usize> = primal_vertices.clone().into_iter().enumerate().map(|(i, f)| (f, i)).collect();

        // Create the dual (primal)
        // By creating the primal faces
        let faces = primal_faces
            .iter()
            .map(|&dual_vert_id| dual.loop_structure.faces(dual_vert_id).into_iter().rev().collect_vec())
            .collect_vec();
        let int_faces = faces.iter().map(|face| face.iter().map(|vert| vert_to_int[vert]).collect_vec()).collect_vec();

        let (primal, vert_map, _) = PolycubeMesh::from(&int_faces, &vec![Vector3D::new(0., 0., 0.); primal_vertices.len()]).unwrap();

        for vert_id in &primal.vert_ids() {
            let region_id = primal_vertices[vert_map.id(vert_id).unwrap().to_owned()];
            region_to_vertex.insert(region_id, vert_id.to_owned());
        }

        let mut polycube = Self {
            structure: primal,
            region_to_vertex,
        };

        polycube.resize(dual, None);

        polycube
    }

    pub fn resize(&mut self, dual: &Dual, layout: Option<&Layout>) {
        let mut vert_to_coord = HashMap::new();
        for vert_id in self.structure.vert_ids() {
            vert_to_coord.insert(vert_id, [0., 0., 0.]);
        }

        let mut levels = [Vec::new(), Vec::new(), Vec::new()];

        // Fix the positions of the vertices that are in the same level
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            for (level, zones) in dual.level_graphs.levels[direction as usize].iter().enumerate() {
                let verts_in_level = zones
                    .iter()
                    .flat_map(|&zone_id| {
                        dual.level_graphs.zones[zone_id]
                            .regions
                            .iter()
                            .map(|&region_id| self.region_to_vertex.get_by_left(&region_id).unwrap().to_owned())
                    })
                    .collect_vec();

                let value = layout.map_or(level as f64, |lay| {
                    let verts_in_mesh = verts_in_level
                        .iter()
                        .map(|vert| lay.granulated_mesh.position(lay.vert_to_corner.get_by_left(vert).unwrap().to_owned())[direction as usize])
                        .collect_vec();
                    hutspot::math::calculate_average_f64(verts_in_mesh.into_iter())
                });

                levels[direction as usize].push((value, verts_in_level.clone()));
            }
        }

        // scale the coordinates s.t. smallest edge is 1, and all other edges are multiples of 1 (integer lengths)
        let mut min_distance = f64::MAX;
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let direction_levels = &levels[direction as usize];
            for (level1, level2) in direction_levels.iter().tuple_windows() {
                let distance = level2.0 - level1.0;
                if distance < min_distance {
                    min_distance = distance;
                }
            }
        }
        assert!(!(min_distance == 0.), "The distance between two levels is 0. This should not happen.");
        let scale = 1. / min_distance;
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            for (level, verts_in_level) in levels[direction as usize].iter() {
                let value = level * scale;
                // round to nearest integer
                let value = value.round();
                for vert in verts_in_level {
                    vert_to_coord.get_mut(&vert).unwrap()[direction as usize] = value;
                }
            }
        }

        // Assign the positions to the vertices
        for vert_id in self.structure.vert_ids() {
            let [x, y, z] = vert_to_coord[&vert_id];
            self.structure.set_position(vert_id, Vector3D::new(x, y, z));
        }
    }
}
