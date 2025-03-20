use crate::dual::{Dual, LoopRegionID, LoopSegmentID, Orientation};
use crate::layout::Layout;
use crate::PrincipalDirection;
use bevy::utils::hashbrown::HashSet;
use bimap::BiHashMap;
use douconel::douconel::{Douconel, Empty};
use douconel::douconel_embedded::{EmbeddedVertex, HasPosition};
use hutspot::geom::Vector3D;
use itertools::Itertools;
use std::collections::HashMap;

slotmap::new_key_type! {
    pub struct PolycubeVertID;
    pub struct PolycubeEdgeID;
    pub struct PolycubeFaceID;
}

type PolycubeMesh = Douconel<PolycubeVertID, EmbeddedVertex, PolycubeEdgeID, Empty, PolycubeFaceID, (PrincipalDirection, Orientation)>;

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
            .map(|&dual_vert_id| dual.loop_structure.star(dual_vert_id).into_iter().rev().collect_vec())
            .collect_vec();
        let int_faces = faces.iter().map(|face| face.iter().map(|vert| vert_to_int[vert]).collect_vec()).collect_vec();

        let (primal, vert_map, _) = PolycubeMesh::from_embedded_faces(&int_faces, &vec![Vector3D::new(0., 0., 0.); primal_vertices.len()]).unwrap();

        for vert_id in &primal.vert_ids() {
            let region_id = primal_vertices[vert_map.get_by_right(vert_id).unwrap().to_owned()];
            region_to_vertex.insert(region_id, vert_id.to_owned());
        }

        let mut polycube = Self {
            structure: primal,
            region_to_vertex,
        };

        polycube.resize(dual, None, None);

        polycube
    }

    pub fn find_intersectionfree_embedding(&mut self, dual: &Dual, layout: Option<&Layout>) {
        // Compute initial embedding without extra intersection constraints
        self.resize(dual, layout, None);
    }

    pub fn resize(&mut self, dual: &Dual, layout: Option<&Layout>, constraints: Option<HashSet<(PolycubeFaceID, PolycubeFaceID)>>) {
        let mut vert_to_coord = HashMap::new();
        for vert_id in self.structure.vert_ids() {
            vert_to_coord.insert(vert_id, [0., 0., 0.]);
        }

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

                let value = if let Some(lay) = layout {
                    let verts_in_mesh = verts_in_level
                        .iter()
                        .map(|vert| lay.granulated_mesh.position(lay.vert_to_corner.get_by_left(vert).unwrap().to_owned())[direction as usize])
                        .collect_vec();
                    hutspot::math::calculate_average_f64(verts_in_mesh.into_iter())
                } else {
                    level as f64
                };

                println!("Level {} in direction {:?} should be {}", level, direction, value);
                for vert in verts_in_level {
                    vert_to_coord.get_mut(&vert).unwrap()[direction as usize] = value;
                }
            }
        }

        // Assign the positions to the vertices
        for vert_id in self.structure.vert_ids() {
            let [x, y, z] = vert_to_coord[&vert_id];
            let position = Vector3D::new(x, y, z);
            // println!("Vertex {vert_id:?} at {position:?}");
            self.structure.verts[vert_id].set_position(position);
        }
    }

    pub fn edge_to_segment(&self, edge_id: PolycubeEdgeID, dual: &Dual) -> LoopSegmentID {
        let (vertex_start, vertex_end) = self.structure.endpoints(edge_id);
        let (intersection_start, intersection_end) = (
            self.region_to_vertex.get_by_right(&vertex_start).unwrap(),
            self.region_to_vertex.get_by_right(&vertex_end).unwrap(),
        );

        dual.loop_structure.edge_between_faces(*intersection_start, *intersection_end).unwrap().0
    }

    fn label(&self, face_id: PolycubeFaceID, dual: &Dual) -> PrincipalDirection {
        let segments = self
            .structure
            .edges(face_id)
            .into_iter()
            .map(|e| {
                let endpoints = self.structure.endpoints(e);
                (
                    self.region_to_vertex.get_by_right(&endpoints.0).unwrap().to_owned(),
                    self.region_to_vertex.get_by_right(&endpoints.1).unwrap().to_owned(),
                )
            })
            .map(|(r1, r2)| dual.loop_structure.edge_between_faces(r1, r2).unwrap().0)
            .map(|s| dual.segment_to_direction(s))
            .collect_vec();
        [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z]
            .iter()
            .find(|&direction| !segments.contains(direction))
            .unwrap()
            .to_owned()
    }
}
