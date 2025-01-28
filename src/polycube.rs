use crate::dual::{Dual, RegionID, ZoneID};
use crate::layout::Layout;
use crate::PrincipalDirection;
use bimap::BiHashMap;
use douconel::douconel::{Douconel, Empty};
use douconel::douconel_embedded::{EmbeddedVertex, HasPosition};
use hutspot::geom::Vector3D;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use std::collections::HashMap;

slotmap::new_key_type! {
    pub struct PolycubeVertID;
    pub struct PolycubeEdgeID;
    pub struct PolycubeFaceID;
}

#[derive(Clone, Debug)]
pub struct Polycube {
    pub structure: Douconel<PolycubeVertID, EmbeddedVertex, PolycubeEdgeID, Empty, PolycubeFaceID, Empty>,

    // Mapping from dual to primal
    pub region_to_vertex: BiHashMap<RegionID, PolycubeVertID>,
}

impl Polycube {
    pub fn from_dual(dual: &Dual) -> Self {
        let primal_vertices = dual.loop_structure.face_ids();
        let primal_faces = dual.loop_structure.vert_ids();

        let mut region_to_vertex = BiHashMap::new();

        // Each face to an int
        let vert_to_int: HashMap<RegionID, usize> = primal_vertices.clone().into_iter().enumerate().map(|(i, f)| (f, i)).collect();

        // Create the dual (primal)
        // By creating the primal faces
        let faces = primal_faces
            .iter()
            .map(|&dual_vert_id| dual.loop_structure.star(dual_vert_id).into_iter().rev().collect_vec())
            .collect_vec();
        let int_faces = faces.iter().map(|face| face.iter().map(|vert| vert_to_int[vert]).collect_vec()).collect_vec();

        let (primal, vert_map, _) = Douconel::<PolycubeVertID, EmbeddedVertex, PolycubeEdgeID, Empty, PolycubeFaceID, Empty>::from_embedded_faces(
            &int_faces,
            &vec![Vector3D::new(0., 0., 0.); primal_vertices.len()],
        )
        .unwrap();

        for vert_id in &primal.vert_ids() {
            let region_id = primal_vertices[vert_map.get_by_right(vert_id).unwrap().to_owned()];
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
        let mut zone_to_target = HashMap::new();
        let mut smallest = 0.1;

        if let Some(layout) = layout {
            for zone_id in dual.zones.keys() {
                let direction = dual.zones[zone_id].direction;
                let regions = &dual.zones[zone_id].regions;
                let average_coord = hutspot::math::calculate_average_f64(
                    regions
                        .iter()
                        .filter_map(|&r| self.region_to_vertex.get_by_left(&r))
                        .filter_map(|&v| layout.vert_to_corner.get_by_left(&v))
                        .map(|&c| layout.granulated_mesh.position(c)[direction as usize]),
                );

                zone_to_target.insert(zone_id, average_coord * 1000.);
            }

            let min = zone_to_target.values().min_by_key(|&&a| OrderedFloat(a)).unwrap().to_owned();
            let max = zone_to_target.values().max_by_key(|&&a| OrderedFloat(a)).unwrap().to_owned();
            let max_dist = max - min;
            smallest = max_dist * 0.001;
        }

        let mut zone_to_dependencies: HashMap<_, Vec<f64>> = HashMap::new();
        let mut zone_to_coordinate = HashMap::new();
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let mut root_is_set = false;
            // Get topological sort of the zones
            let level_graph = &dual.level_graphs[direction as usize];
            let mut topological_sort =
                hutspot::graph::topological_sort::<ZoneID>(&level_graph.keys().copied().collect_vec(), |z| level_graph[&z].clone()).unwrap();
            for zone in topological_sort.clone() {
                let mut coordinate = zone_to_target.get(&zone).copied().unwrap_or(0.0);
                // Get the dependencies of the zone
                if let Some(dependencies) = zone_to_dependencies.get(&zone) {
                    // Assign value as the maximum of the dependencies + 1
                    coordinate = (dependencies.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() + smallest).max(coordinate);
                } else {
                    if root_is_set {
                        continue;
                    }
                    root_is_set = true;
                }
                // Assign this coordinate as a dependency to all the children
                for child in &level_graph[&zone] {
                    zone_to_dependencies.entry(child).or_insert(vec![]).push(coordinate);
                }
                // Assign the coordinate to the zone
                zone_to_coordinate.insert(zone, coordinate);
            }
            topological_sort.reverse();
            for zone in topological_sort {
                if zone_to_coordinate.contains_key(&zone) {
                    continue;
                }
                // Get the dependencies of the zone
                let dependencies = level_graph[&zone].iter().map(|z| zone_to_coordinate[z]).collect_vec();

                // Assign the coordinate to the zone
                if let Some(coordinate) = zone_to_target.get(&zone).copied() {
                    zone_to_coordinate.insert(
                        zone,
                        (dependencies.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() - smallest).min(coordinate),
                    );
                } else {
                    zone_to_coordinate.insert(zone, dependencies.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() - smallest);
                }
            }
        }

        for vertex_id in self.structure.vert_ids() {
            let region_id = self.region_to_vertex.get_by_right(&vertex_id).unwrap().to_owned();
            let coordinates = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|d| {
                let zone = dual.region_to_zone(region_id, d);
                zone_to_coordinate[&zone]
            });
            self.structure.verts[vertex_id].set_position(Vector3D::new(coordinates[0], coordinates[1], coordinates[2]));
        }
    }
}
