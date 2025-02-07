use crate::dual::{Dual, Orientation, RegionID, ZoneID};
use crate::layout::Layout;
use crate::PrincipalDirection;
use bevy::log::{debug, info};
use bevy::utils::hashbrown::HashSet;
use bimap::BiHashMap;
use douconel::douconel::{Douconel, Empty};
use douconel::douconel_embedded::{EmbeddedVertex, HasPosition};
use hutspot::geom::Vector3D;
use itertools::Itertools;
use microlp::{ComparisonOp, OptimizationDirection, Problem};
use nalgebra::{DMatrix, DVector};
use nalgebra_sparse::factorization::{CholeskyError, CscCholesky};
use ordered_float::OrderedFloat;
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

        // Find all intersections
        let mut intersections = HashSet::new();

        loop {
            let mut added_new_intersection = false;

            let faces = self.structure.face_ids();
            for i in 0..faces.len() {
                for j in i + 1..faces.len() {
                    let face_id = faces[i];
                    let other_face_id = faces[j];

                    if self.faces_intersect(face_id, other_face_id, dual) {
                        if face_id < other_face_id {
                            intersections.insert((face_id, other_face_id));
                        } else {
                            intersections.insert((other_face_id, face_id));
                        }
                        added_new_intersection = true;
                        break;
                    }
                }
            }

            // println!("adding {} intersections", intersections.len());

            if !added_new_intersection {
                break;
            }

            // Recompute embedding with intersection constraints
            self.resize(dual, layout, Some(intersections.clone()));
        }

        // println!("Found intersection-free embedding!");
    }

    pub fn resize(&mut self, dual: &Dual, layout: Option<&Layout>, constraints: Option<HashSet<(PolycubeFaceID, PolycubeFaceID)>>) {
        // We choose to minimize the overall volume (height + width + depth) of the polycube.
        let mut problem = Problem::new(OptimizationDirection::Minimize);

        // Define variables for all vertices (x, y, and z)
        let mut vert_to_var = HashMap::new();
        for vert_id in self.structure.vert_ids() {
            let x = problem.add_integer_var(0.0, (0, 100));
            let y = problem.add_integer_var(0.0, (0, 100));
            let z = problem.add_integer_var(0.0, (0, 100));
            vert_to_var.insert(vert_id, (x, y, z));
        }

        // Fix the positions of the vertices that are in the same zone
        for zone_id in dual.zones.keys() {
            let verts_in_zone = dual.zones[zone_id]
                .regions
                .iter()
                .map(|&region_id| self.region_to_vertex.get_by_left(&region_id).unwrap().to_owned())
                .collect_vec();
            let direction = dual.zones[zone_id].direction;

            let anchor_vert = verts_in_zone[0];
            for vert_id in verts_in_zone.iter().skip(1) {
                let (x, y, z) = vert_to_var[vert_id];
                let (x_anchor, y_anchor, z_anchor) = vert_to_var[&anchor_vert];
                match direction {
                    PrincipalDirection::X => {
                        problem.add_constraint([(x, 1.0), (x_anchor, -1.0)], ComparisonOp::Eq, 0.0);
                    }
                    PrincipalDirection::Y => {
                        problem.add_constraint([(y, 1.0), (y_anchor, -1.0)], ComparisonOp::Eq, 0.0);
                    }
                    PrincipalDirection::Z => {
                        problem.add_constraint([(z, 1.0), (z_anchor, -1.0)], ComparisonOp::Eq, 0.0);
                    }
                }
            }
        }

        // Subsequent zones in the level graph should be at least 1 unit away from each other
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let level_graph = &dual.level_graphs[direction as usize];
            for &zone_id in level_graph.keys() {
                for &child in &level_graph[&zone_id] {
                    let verts_in_zone = dual.zones[zone_id]
                        .regions
                        .iter()
                        .map(|&region_id| self.region_to_vertex.get_by_left(&region_id).unwrap().to_owned())
                        .collect_vec();
                    let verts_in_child = dual.zones[child]
                        .regions
                        .iter()
                        .map(|&region_id| self.region_to_vertex.get_by_left(&region_id).unwrap().to_owned())
                        .collect_vec();

                    let (x, y, z) = vert_to_var[&verts_in_zone[0]];
                    let (x_child, y_child, z_child) = vert_to_var[&verts_in_child[0]];

                    match direction {
                        PrincipalDirection::X => {
                            problem.add_constraint([(x, 1.0), (x_child, -1.0)], ComparisonOp::Le, -1.0);
                        }
                        PrincipalDirection::Y => {
                            problem.add_constraint([(y, 1.0), (y_child, -1.0)], ComparisonOp::Le, -1.0);
                        }
                        PrincipalDirection::Z => {
                            problem.add_constraint([(z, 1.0), (z_child, -1.0)], ComparisonOp::Le, -1.0);
                        }
                    }
                }
            }
        }

        // For every pair of faces add constraints s.t. they do not intersect (by adding some seperation criteria)
        // Do not add this constraint for any pair of faces that is directly adjacent to eachother.
        if let Some(faces) = constraints {
            for (f_i, f_j) in faces {
                let [(x_i_min, x_i_max), (y_i_min, y_i_max), (z_i_min, z_i_max)] = self.face_box(f_i, dual);
                let [(x_j_min, x_j_max), (y_j_min, y_j_max), (z_j_min, z_j_max)] = self.face_box(f_j, dual);
                let x_i_min = vert_to_var[&x_i_min].0;
                let x_i_max = vert_to_var[&x_i_max].0;
                let y_i_min = vert_to_var[&y_i_min].1;
                let y_i_max = vert_to_var[&y_i_max].1;
                let z_i_min = vert_to_var[&z_i_min].2;
                let z_i_max = vert_to_var[&z_i_max].2;
                let x_j_min = vert_to_var[&x_j_min].0;
                let x_j_max = vert_to_var[&x_j_max].0;
                let y_j_min = vert_to_var[&y_j_min].1;
                let y_j_max = vert_to_var[&y_j_max].1;
                let z_j_min = vert_to_var[&z_j_min].2;
                let z_j_max = vert_to_var[&z_j_max].2;

                // Add the constraints
                // Big-M: choose a number large enough so that when a constraint is “turned off” it is inactive.
                let M = 500.0;
                // Gap required between faces.
                let delta = 1.0;

                // Create binary variables for each separation disjunct.
                let b_x_left = problem.add_binary_var(0.0); // f_i is to the left of f_j (x-axis).
                let b_x_right = problem.add_binary_var(0.0); // f_i is to the right of f_j (x-axis).
                let b_y_below = problem.add_binary_var(0.0); // f_i is below f_j (y-axis).
                let b_y_above = problem.add_binary_var(0.0); // f_i is above f_j (y-axis).
                let b_z_front = problem.add_binary_var(0.0); // f_i is in front of f_j (z-axis).
                let b_z_back = problem.add_binary_var(0.0); // f_i is behind f_j (z-axis).

                // 1. b_x_left = 1 => x_i_max + delta < x_j_min,
                //    simulated as: x_i_max - x_j_min + M * b_x_left <= M - delta - epsilon
                problem.add_constraint([(x_i_max, 1.0), (x_j_min, -1.0), (b_x_left, M)], ComparisonOp::Le, M - delta);

                // 2. b_x_right = 1 => x_j_max + delta < x_i_min,
                //    simulated as: x_j_max - x_i_min + M * b_x_right <= M - delta - epsilon
                problem.add_constraint([(x_j_max, 1.0), (x_i_min, -1.0), (b_x_right, M)], ComparisonOp::Le, M - delta);

                // 3. b_y_below = 1 => y_i_max + delta < y_j_min,
                problem.add_constraint([(y_i_max, 1.0), (y_j_min, -1.0), (b_y_below, M)], ComparisonOp::Le, M - delta);

                // 4. b_y_above = 1 => y_j_max + delta < y_i_min,
                problem.add_constraint([(y_j_max, 1.0), (y_i_min, -1.0), (b_y_above, M)], ComparisonOp::Le, M - delta);

                // 5. b_z_front = 1 => z_i_max + delta < z_j_min,
                problem.add_constraint([(z_i_max, 1.0), (z_j_min, -1.0), (b_z_front, M)], ComparisonOp::Le, M - delta);

                // 6. b_z_back = 1 => z_j_max + delta < z_i_min,
                problem.add_constraint([(z_j_max, 1.0), (z_i_min, -1.0), (b_z_back, M)], ComparisonOp::Le, M - delta);

                // Force at least one of the disjuncts to be active
                // That is: b_x_left + b_x_right + b_y_below + b_y_above + b_z_front + b_z_back >= 1.
                problem.add_constraint(
                    [
                        (b_x_left, 1.0),
                        (b_x_right, 1.0),
                        (b_y_below, 1.0),
                        (b_y_above, 1.0),
                        (b_z_front, 1.0),
                        (b_z_back, 1.0),
                    ],
                    ComparisonOp::Ge,
                    1.0,
                );
            }
        }

        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            let edges_in_direction = self
                .structure
                .edge_ids()
                .into_iter()
                .map(|e| {
                    let endpoints = self.structure.endpoints(e);
                    (
                        e,
                        self.region_to_vertex.get_by_right(&endpoints.0).unwrap().to_owned(),
                        self.region_to_vertex.get_by_right(&endpoints.1).unwrap().to_owned(),
                    )
                })
                .map(|(e, r1, r2)| (e, dual.loop_structure.edge_between_faces(r1, r2).unwrap().0))
                .filter(|&(_, s)| dual.segment_to_direction(s) == direction)
                .filter(|&(_, s)| dual.segment_to_orientation(s) == Orientation::Forwards);

            for (edge_id, _) in edges_in_direction {
                let (x1, y1, z1) = vert_to_var[&self.structure.endpoints(edge_id).0];
                let (x2, y2, z2) = vert_to_var[&self.structure.endpoints(edge_id).1];
                // Add the variable for this edge
                let edge_length = problem.add_var(1., (0.0, 100.0));

                // for x-direction: edge_length = x2 - x1, etc.
                match direction {
                    PrincipalDirection::X => {
                        problem.add_constraint([(x1, 1.0), (x2, -1.0), (edge_length, 1.0)], ComparisonOp::Eq, 0.0);
                    }
                    PrincipalDirection::Y => {
                        problem.add_constraint([(y1, 1.0), (y2, -1.0), (edge_length, 1.0)], ComparisonOp::Eq, 0.0);
                    }
                    PrincipalDirection::Z => {
                        problem.add_constraint([(z1, 1.0), (z2, -1.0), (edge_length, 1.0)], ComparisonOp::Eq, 0.0);
                    }
                }
            }
        }
        // Add all forward edges as extra variable, their value should be minimized
        for edge_id in self.structure.edge_ids() {
            let endpoints = self.structure.endpoints(edge_id);
            let (x1, y1, z1) = vert_to_var[&endpoints.0];
            let (x2, y2, z2) = vert_to_var[&endpoints.1];
            let extra_var = problem.add_integer_var(0.0, (0, 100));
            problem.add_constraint([(x1, 1.0), (x2, -1.0), (extra_var, 1.0)], ComparisonOp::Ge, 0.0);
            problem.add_constraint([(x1, 1.0), (x2, -1.0), (extra_var, 1.0)], ComparisonOp::Le, 100.0);
            problem.add_constraint([(y1, 1.0), (y2, -1.0), (extra_var, 1.0)], ComparisonOp::Ge, 0.0);
            problem.add_constraint([(y1, 1.0), (y2, -1.0), (extra_var, 1.0)], ComparisonOp::Le, 100.0);
            problem.add_constraint([(z1, 1.0), (z2, -1.0), (extra_var, 1.0)], ComparisonOp::Ge, 0.0);
            problem.add_constraint([(z1, 1.0), (z2, -1.0), (extra_var, 1.0)], ComparisonOp::Le, 100.0);
        }

        // Print info about the problem
        // println!("Solving {problem:?}");

        // --- Solve the LP ---
        let solution = problem.solve().unwrap();

        // Print the results.
        // println!("Optimal objective (volume): {}", solution.objective());

        // Assign the positions to the vertices
        for vert_id in self.structure.vert_ids() {
            let (x, y, z) = vert_to_var[&vert_id];
            let position = Vector3D::new(solution[x], solution[y], solution[z]);
            // println!("Vertex {vert_id:?} at {position:?}");
            self.structure.verts[vert_id].set_position(position);
        }

        // let mut zone_to_target = HashMap::new();
        // let mut smallest = 0.1;

        // if let Some(layout) = layout {
        //     for zone_id in dual.zones.keys() {
        //         let direction = dual.zones[zone_id].direction;
        //         let regions = &dual.zones[zone_id].regions;
        //         let average_coord = hutspot::math::calculate_average_f64(
        //             regions
        //                 .iter()
        //                 .filter_map(|&r| self.region_to_vertex.get_by_left(&r))
        //                 .filter_map(|&v| layout.vert_to_corner.get_by_left(&v))
        //                 .map(|&c| layout.granulated_mesh.position(c)[direction as usize]),
        //         );

        //         zone_to_target.insert(zone_id, average_coord * 1000.);
        //     }

        //     let min = zone_to_target.values().min_by_key(|&&a| OrderedFloat(a)).unwrap().to_owned();
        //     let max = zone_to_target.values().max_by_key(|&&a| OrderedFloat(a)).unwrap().to_owned();
        //     let max_dist = max - min;
        //     smallest = max_dist * 0.001;
        // }

        // let mut zone_to_dependencies: HashMap<_, Vec<f64>> = HashMap::new();
        // let mut zone_to_coordinate = HashMap::new();
        // for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
        //     let mut root_is_set = false;
        //     // Get topological sort of the zones
        //     let level_graph = &dual.level_graphs[direction as usize];
        //     let mut topological_sort =
        //         hutspot::graph::topological_sort::<ZoneID>(&level_graph.keys().copied().collect_vec(), |z| level_graph[&z].clone()).unwrap();
        //     for zone in topological_sort.clone() {
        //         let mut coordinate = zone_to_target.get(&zone).copied().unwrap_or(0.0);
        //         // Get the dependencies of the zone
        //         if let Some(dependencies) = zone_to_dependencies.get(&zone) {
        //             // Assign value as the maximum of the dependencies + 1
        //             coordinate = (dependencies.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap() + smallest).max(coordinate);
        //         } else {
        //             if root_is_set {
        //                 continue;
        //             }
        //             root_is_set = true;
        //         }
        //         // Assign this coordinate as a dependency to all the children
        //         for child in &level_graph[&zone] {
        //             zone_to_dependencies.entry(child).or_insert(vec![]).push(coordinate);
        //         }
        //         // Assign the coordinate to the zone
        //         zone_to_coordinate.insert(zone, coordinate);
        //     }
        //     topological_sort.reverse();
        //     for zone in topological_sort {
        //         if zone_to_coordinate.contains_key(&zone) {
        //             continue;
        //         }
        //         // Get the dependencies of the zone
        //         let dependencies = level_graph[&zone].iter().map(|z| zone_to_coordinate[z]).collect_vec();

        //         // Assign the coordinate to the zone
        //         if let Some(coordinate) = zone_to_target.get(&zone).copied() {
        //             zone_to_coordinate.insert(
        //                 zone,
        //                 (dependencies.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap_or(&1000000.) - smallest).min(coordinate),
        //             );
        //         } else {
        //             zone_to_coordinate.insert(
        //                 zone,
        //                 dependencies.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap_or(&1000000.) - smallest,
        //             );
        //         }
        //     }
        // }

        // for vertex_id in self.structure.vert_ids() {
        //     let region_id = self.region_to_vertex.get_by_right(&vertex_id).unwrap().to_owned();
        //     let coordinates = [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|d| {
        //         let zone = dual.region_to_zone(region_id, d);
        //         zone_to_coordinate[&zone]
        //     });
        //     self.structure.verts[vertex_id].set_position(Vector3D::new(coordinates[0], coordinates[1], coordinates[2]));
        // }
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

    fn face_box(&self, face_id: PolycubeFaceID, dual: &Dual) -> [(PolycubeVertID, PolycubeVertID); 3] {
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
            .filter(|&s| dual.segment_to_orientation(s) == Orientation::Forwards)
            .collect_vec();

        [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z].map(|direction| {
            let segment_in_direction = segments.iter().find(|&&s| dual.segment_to_direction(s) == direction);

            if let Some(&segment) = segment_in_direction {
                let [min_region, max_region] = dual.loop_structure.faces(segment);
                (
                    self.region_to_vertex.get_by_left(&min_region).unwrap().to_owned(),
                    self.region_to_vertex.get_by_left(&max_region).unwrap().to_owned(),
                )
            } else {
                // all vertices in the face have the same x-coordinate
                let arbitrary_v = self.structure.corners(face_id)[0];
                (arbitrary_v, arbitrary_v)
            }
        })
    }

    // Check if two faces (their bounding boxes) intersect.
    fn faces_intersect(&self, f_i: PolycubeFaceID, f_j: PolycubeFaceID, dual: &Dual) -> bool {
        // faces never intersect if they share a vertex
        if self.structure.corners(f_i).iter().any(|&v| self.structure.corners(f_j).contains(&v)) {
            return false;
        }

        let box_i = self.face_box(f_i, dual);
        let box_j = self.face_box(f_j, dual);

        for i in 0..3 {
            let (min_i, max_i) = box_i[i];
            let (min_j, max_j) = box_j[i];

            if self.structure.verts[min_i].position()[i] > self.structure.verts[max_j].position()[i]
                || self.structure.verts[max_i].position()[i] < self.structure.verts[min_j].position()[i]
            {
                return false;
            }
        }

        // println!("Faces {:?} and {:?} intersect", f_i, f_j);

        true
    }

    // Generate a polycuboid
    pub fn polycuboid(&self, dual: &Dual, layout: &mut Layout) {
        return;

        let mesh = layout.granulated_mesh.clone();

        let mut vert_to_var = HashMap::new();

        for zone_id in dual.zones.keys() {
            // Get the zone's corresponding loop regions
            let regions = &dual.zones[zone_id].regions;
            // Get the corresponding corners in the polycube
            let corners = regions
                .iter()
                .map(|r| self.region_to_vertex.get_by_left(r).unwrap().to_owned())
                .collect::<HashSet<_>>();
            // Get all faces that contain these corners
            let faces = self
                .structure
                .face_ids()
                .into_iter()
                .filter(|&f| self.structure.corners(f).iter().all(|&c| corners.contains(&c)))
                .collect_vec();
            // Get all edges that contain these corners
            let edges = self
                .structure
                .edge_ids()
                .into_iter()
                .filter(|&e| {
                    let (v1, v2) = self.structure.endpoints(e);
                    corners.contains(&v1) && corners.contains(&v2)
                })
                .collect_vec();

            let label = dual.zones[zone_id].direction;

            let mut vertices = HashSet::new();

            for face_id in faces {
                // Get the corresponding patch in the layout
                let patch = layout.face_to_patch.get(&face_id).unwrap();
                // Add the corresponding vertices
                vertices.extend(patch.faces.iter().flat_map(|&face_id| mesh.corners(face_id)));
            }

            for edge_id in edges {
                // Get the corresponding path in the layout
                let path = layout.edge_to_path.get(&edge_id).unwrap();
                // Add the corresponding vertices
                vertices.extend(path);
            }

            let vertices = vertices.into_iter().collect_vec();
            let root_vertex = vertices[0];
            // All of those vertices share `label` coordinate (with root_vertex)
            for vert_id in vertices {
                vert_to_var.insert((vert_id, label), (root_vertex, label));
            }
        }

        for d in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            for vert_id in mesh.vert_ids() {
                if vert_to_var.contains_key(&(vert_id, d)) {
                    continue;
                }
                vert_to_var.insert((vert_id, d), (vert_id, d));
            }
        }

        let mut id_map = HashMap::new();
        let mut next_id = 0;
        for vert_id in mesh.vert_ids() {
            for d in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                let dv = (vert_id, d);
                let root = vert_to_var.get(&dv).unwrap().to_owned();
                if let Some(&id) = id_map.get(&root) {
                    id_map.insert(dv, id);
                } else {
                    let id = next_id;
                    next_id += 1;
                    id_map.insert(root, id);

                    id_map.insert(dv, id);
                }
            }
        }
        let n_vars = next_id;

        println!("Number of variables: {}", n_vars);

        // Build the least-squares system A * x = b
        let mut rows = Vec::new(); // each element: (Vec<(col_index, coeff)>, rhs)

        for face_id in self.structure.face_ids() {
            // Get the corresponding patch in the layout
            let patch = layout.face_to_patch.get(&face_id).unwrap();
            // Get the corresponding label in the layout
            let label = self.label(face_id, dual);

            for &face_id in &patch.faces {
                let farea = mesh.area(face_id).sqrt();
                // for each edge, add difference constraints for d != label
                for &edge_id in &mesh.edges(face_id) {
                    let (v1, v2) = mesh.endpoints(edge_id);
                    let (p1, p2) = (mesh.position(v1), mesh.position(v2));

                    for d in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                        if d == label {
                            continue;
                        }
                        let orig_diff = p1[d as usize] - p2[d as usize];

                        // A row:   x[var_v1] - x[var_v2] = farea * orig_diff
                        let var_v1 = id_map.get(&(v1, d)).unwrap().to_owned();
                        let var_v2 = id_map.get(&(v2, d)).unwrap().to_owned();
                        rows.push((vec![(var_v1, 1.0), (var_v2, -1.0)], farea * orig_diff));
                    }
                }
            }
        }

        println!("Number of constraints: {}", rows.len());

        // Convert rows -> A (dense) and b
        let n_rows = rows.len();
        let mut A = DMatrix::<f64>::zeros(n_rows, n_vars);
        let mut b = DVector::<f64>::zeros(n_rows);

        for (i, (cols, rhs)) in rows.into_iter().enumerate() {
            for (col, val) in cols {
                A[(i, col)] = val;
            }
            b[i] = rhs;
        }

        // 5. Solve least-squares: min_x ||A x - b||^2
        //    We'll use normal equations or SVD fallback
        let x_solution = {
            let At = A.transpose();
            match (&At * &A).cholesky() {
                Some(chol) => chol.solve(&(&At * &b)),
                None => A.svd(false, false).solve(&b, 1e-12).expect("?"),
            }
        };

        // For each face, look at its 4 corners. The 4 corners define a bounding box. All other vertices must lie inside this bounding box.
        for face_id in self.structure.face_ids() {
            // Get the corners
            let verts = self.structure.corners(face_id);
            // Get their embedding
            let corners = verts.into_iter().map(|vert_id| layout.vert_to_corner.get_by_left(&vert_id).unwrap().clone());

            // Get the bounding box
            let min_x = corners
                .clone()
                .map(|v| OrderedFloat(x_solution[id_map.get(&(v, PrincipalDirection::X)).copied().unwrap()]))
                .min()
                .unwrap()
                .0;
            let min_y = corners
                .clone()
                .map(|v| OrderedFloat(x_solution[id_map.get(&(v, PrincipalDirection::Y)).copied().unwrap()]))
                .min()
                .unwrap()
                .0;
            let min_z = corners
                .clone()
                .map(|v| OrderedFloat(x_solution[id_map.get(&(v, PrincipalDirection::Z)).copied().unwrap()]))
                .min()
                .unwrap()
                .0;
            let max_x = corners
                .clone()
                .map(|v| OrderedFloat(x_solution[id_map.get(&(v, PrincipalDirection::X)).copied().unwrap()]))
                .max()
                .unwrap()
                .0;
            let max_y = corners
                .clone()
                .map(|v| OrderedFloat(x_solution[id_map.get(&(v, PrincipalDirection::Y)).copied().unwrap()]))
                .max()
                .unwrap()
                .0;
            let max_z = corners
                .clone()
                .map(|v| OrderedFloat(x_solution[id_map.get(&(v, PrincipalDirection::Z)).copied().unwrap()]))
                .max()
                .unwrap()
                .0;

            // Get the vertices in this patch
            let vertices = layout
                .face_to_patch
                .get(&face_id)
                .unwrap()
                .faces
                .iter()
                .flat_map(|&face_id| mesh.corners(face_id))
                .collect::<HashSet<_>>();

            for &vert_id in &vertices {
                let x = x_solution[id_map.get(&(vert_id, PrincipalDirection::X)).copied().unwrap()];
                let y = x_solution[id_map.get(&(vert_id, PrincipalDirection::Y)).copied().unwrap()];
                let z = x_solution[id_map.get(&(vert_id, PrincipalDirection::Z)).copied().unwrap()];

                // Clamp x y z
                let x = x.max(min_x).min(max_x);
                let y = y.max(min_y).min(max_y);
                let z = z.max(min_z).min(max_z);

                layout.granulated_mesh.verts[vert_id].set_position(Vector3D::new(x, y, z));
            }
        }
    }
}
