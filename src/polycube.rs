use crate::dual::{Dual, Orientation, RegionID, SegmentID, ZoneID};
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
use nalgebra::{DMatrix, DVector, Vector};
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
        let mut vert_to_coord = HashMap::new();
        for vert_id in self.structure.vert_ids() {
            vert_to_coord.insert(vert_id, [0., 0., 0.]);
        }

        // Fix the positions of the vertices that are in the same level
        for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
            for (level, zones) in dual.levels[direction as usize].iter().enumerate() {
                let verts_in_level = zones
                    .iter()
                    .flat_map(|&zone_id| {
                        dual.zones[zone_id]
                            .regions
                            .iter()
                            .map(|&region_id| self.region_to_vertex.get_by_left(&region_id).unwrap().to_owned())
                    })
                    .collect_vec();

                let value = dual.level_to_value[direction as usize][level];
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

    pub fn edge_to_segment(&self, edge_id: PolycubeEdgeID, dual: &Dual) -> SegmentID {
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
