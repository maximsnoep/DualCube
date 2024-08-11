use crate::dual::{Dual, IntersectionID, PrincipalDirection, RegionID};
use bimap::BiHashMap;
use douconel::douconel::{Douconel, Empty};
use douconel::douconel_embedded::EmbeddedVertex;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use std::collections::HashMap;

slotmap::new_key_type! {
    pub struct PolycubeVertID;
    pub struct PolycubeEdgeID;
    pub struct PolycubeFaceID;
}

#[derive(Clone, Debug, Default)]
pub struct Polycube {
    pub structure: Douconel<PolycubeVertID, EmbeddedVertex, PolycubeEdgeID, Empty, PolycubeFaceID, Empty>,

    // Mapping.
    pub intersection_to_face: BiHashMap<IntersectionID, PolycubeFaceID>,
    pub region_to_vertex: BiHashMap<RegionID, PolycubeVertID>,
}

impl Polycube {
    pub fn from_dual(dual: &Dual) -> Self {
        let primal_vertices = dual.loop_structure.face_ids();
        let primal_faces = dual.loop_structure.vert_ids();

        let mut intersection_to_face = BiHashMap::new();
        let mut region_to_vertex = BiHashMap::new();

        // Each face to an int
        let vert_to_int: HashMap<RegionID, usize> = primal_vertices.clone().into_iter().enumerate().map(|(i, f)| (f, i)).collect();

        // Create the dual (primal)
        // By creating the primal faces
        let faces = primal_faces
            .clone()
            .into_iter()
            .map(|dual_vert_id| dual.loop_structure.star(dual_vert_id).into_iter().rev().collect_vec())
            .collect_vec();
        let int_faces = faces.iter().map(|face| face.iter().map(|vert| vert_to_int[vert]).collect_vec()).collect_vec();

        let vertex_positions = primal_vertices
            .clone()
            .into_iter()
            .map(|subsurface_id| {
                // Find the three zones that this subsurface is part of (X/Y/Z)
                let coordinates = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|d| {
                    dual.zones
                        .iter()
                        .find(|(_, z)| z.regions.contains(&subsurface_id) && z.direction == d)
                        .unwrap()
                        .1
                        .coordinate
                        .unwrap()
                });
                Vector3D::new(coordinates[0], coordinates[1], coordinates[2])
            })
            .collect_vec();

        let (primal, vert_map, face_map) =
            Douconel::<PolycubeVertID, EmbeddedVertex, PolycubeEdgeID, Empty, PolycubeFaceID, Empty>::from_embedded_faces(&int_faces, &vertex_positions)
                .unwrap();

        for face_id in &primal.face_ids() {
            let intersection_id = primal_faces[face_map.get_by_right(face_id).unwrap().to_owned()];
            intersection_to_face.insert(intersection_id, face_id.to_owned());
        }

        for vert_id in &primal.vert_ids() {
            let region_id = primal_vertices[vert_map.get_by_right(vert_id).unwrap().to_owned()];
            region_to_vertex.insert(region_id, vert_id.to_owned());
        }

        Self {
            structure: primal,
            intersection_to_face,
            region_to_vertex,
        }
    }
}
