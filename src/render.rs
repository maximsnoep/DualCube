use crate::dual::{to_principal_direction, PrincipalDirection, Side};
use crate::{Configuration, InputResource, MeshProperties, RenderedMesh, SolutionResource, BACKGROUND_COLOR, MESH_OFFSET, POLYCUBE_OFFSET};
use bevy::prelude::*;
use bevy::render::view::RenderLayers;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use shape::Circle;
use std::collections::HashMap;

// This function should be called when the mesh (RenderedMesh or Solution) is changed, to make sure that all odifications are visualized.
pub fn update(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    configuration: Res<Configuration>,
    rendered_mesh_query: Query<Entity, With<RenderedMesh>>,

    mut mesh_resmut: ResMut<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
) {
    if !mesh_resmut.is_changed() && !solution.is_changed() {
        return;
    }
    info!("InputResource or SolutionResource change has been detected. Updating all renders.");

    for entity in rendered_mesh_query.iter() {
        commands.entity(entity).despawn();
    }
    info!("Despawning any current renders.");

    if mesh_resmut.mesh.faces.is_empty() {
        return;
    }

    // Color maps for the mesh and polycube. Only filled with (primal) colors used if `black` is set to `false`. Default color is black.
    let mut mesh = mesh_resmut.mesh.bevy(&HashMap::new());

    if let Some(polycube) = &solution.current_solution.polycube.clone() {
        let mut polycube_mesh = polycube.structure.bevy(&HashMap::new());

        if let Some(Ok(lay)) = &solution.current_solution.layout {
            if !configuration.black {
                let mut mesh_color_map = HashMap::new();
                let mut poly_color_map = HashMap::new();

                for &face_id in &polycube.structure.face_ids() {
                    let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                    let (dir, side) = to_principal_direction(normal);
                    let color = dir.to_primal_color_sided(side);
                    let c = [color.r(), color.g(), color.b()];

                    // Color the mesh color map
                    for &triangle_id in &lay.face_to_patch[&face_id].faces {
                        mesh_color_map.insert(triangle_id, c);
                    }

                    // Color the polycube color map
                    poly_color_map.insert(face_id, c);
                }

                mesh = lay.granulated_mesh.bevy(&mesh_color_map);
                polycube_mesh = polycube.structure.bevy(&poly_color_map);
            }
        }

        // Draw polycube
        let aabb = polycube_mesh.compute_aabb().unwrap();
        let scale = 10. * (1. / aabb.half_extents.max_element());
        solution.properties.scale = scale;
        let translation = -scale * aabb.center;
        solution.properties.translation = Vector3D::new(translation.x.into(), translation.y.into(), translation.z.into()) + POLYCUBE_OFFSET;
        solution.properties.nr_of_vertices = polycube.structure.nr_verts();
        solution.properties.nr_of_edges = polycube.structure.nr_edges() / 2; // dcel -> single edge
        solution.properties.nr_of_faces = polycube.structure.nr_faces();

        // Spawn the polycube mesh
        commands.spawn((
            MaterialMeshBundle {
                mesh: meshes.add(polycube_mesh),
                transform: Transform {
                    translation: Vec3::new(
                        solution.properties.translation.x as f32,
                        solution.properties.translation.y as f32,
                        solution.properties.translation.z as f32,
                    ),
                    rotation: Quat::from_rotation_z(0f32),
                    scale: Vec3::splat(solution.properties.scale),
                },
                material: materials.add(StandardMaterial { unlit: true, ..default() }),
                ..default()
            },
            RenderLayers::layer(2),
            RenderedMesh,
        ));

        // Spawning two covers such that the two objects are seperated from each other.
        commands.spawn(PbrBundle {
            mesh: meshes.add(Circle::new(100.0).into()),
            transform: Transform {
                translation: Vec3::new(0., 0., 0.),
                rotation: Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
                scale: Vec3::splat(1.),
            },
            material: materials.add(StandardMaterial {
                base_color: BACKGROUND_COLOR,
                unlit: true,
                ..default()
            }),

            ..default()
        });

        commands.spawn(PbrBundle {
            mesh: meshes.add(Circle::new(100.0).into()),
            transform: Transform {
                translation: Vec3::new(0., 0., 0.),
                rotation: Quat::from_rotation_x(std::f32::consts::FRAC_PI_2 + std::f32::consts::PI),
                scale: Vec3::splat(1.),
            },
            material: materials.add(StandardMaterial {
                base_color: BACKGROUND_COLOR,
                unlit: true,
                ..default()
            }),

            ..default()
        });
    }

    // Draw the input mesh
    println!("Drawing the input mesh.");
    let aabb = mesh.compute_aabb().unwrap();
    let scale = 10. * (1. / aabb.half_extents.max_element());
    mesh_resmut.properties.scale = scale;
    let translation = -scale * aabb.center;
    mesh_resmut.properties.translation = Vector3D::new(translation.x.into(), translation.y.into(), translation.z.into()) + MESH_OFFSET;
    mesh_resmut.properties.nr_of_vertices = mesh_resmut.mesh.nr_verts();
    mesh_resmut.properties.nr_of_edges = mesh_resmut.mesh.nr_edges() / 2; // dcel -> single edge
    mesh_resmut.properties.nr_of_faces = mesh_resmut.mesh.nr_faces();

    commands.spawn((
        MaterialMeshBundle {
            mesh: meshes.add(mesh),
            transform: Transform {
                translation: Vec3::new(
                    mesh_resmut.properties.translation.x as f32,
                    mesh_resmut.properties.translation.y as f32,
                    mesh_resmut.properties.translation.z as f32,
                ),
                rotation: Quat::from_rotation_z(0f32),
                scale: Vec3::splat(mesh_resmut.properties.scale),
            },
            material: materials.add(StandardMaterial {
                perceptual_roughness: 0.7,
                unlit: !configuration.black,
                ..default()
            }),
            ..default()
        },
        RenderLayers::layer(1),
        RenderedMesh,
    ));

    // Add gizmos to cache.
    gizmos_cache.loops.clear();
    if let Ok(dual) = &solution.current_solution.dual {
        for segment_id in dual.loop_structure.edge_ids() {
            let direction = dual.loop_structure.edges[segment_id].direction;
            let side = dual.segment_to_side(segment_id, configuration.sides_mask);
            let mut offset = Vector3D::new(0., 0., 0.);
            let dist = 0.001 * f64::from(mesh_resmut.properties.scale);
            match side {
                Side::Upper => match direction {
                    PrincipalDirection::X => offset[0] += dist,
                    PrincipalDirection::Y => offset[1] += dist,
                    PrincipalDirection::Z => offset[2] += dist,
                },
                Side::Lower => match direction {
                    PrincipalDirection::X => offset[0] -= dist,
                    PrincipalDirection::Y => offset[1] -= dist,
                    PrincipalDirection::Z => offset[2] -= dist,
                },
            };
            let color = direction.to_dual_color_sided(side);
            for edgepair in solution.current_solution.get_pairs_of_sequence(&dual.segment_to_edges(segment_id)) {
                let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                add_line2(&mut gizmos_cache.loops, u, v, offset + n * 0.05, color, &mesh_resmut.properties);
            }
        }
    } else {
        for loop_id in solution.current_solution.loops.keys() {
            let color = solution.current_solution.loop_to_direction(loop_id).to_dual_color();
            for edgepair in solution.current_solution.get_pairs_of_loop(loop_id) {
                let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                add_line2(&mut gizmos_cache.loops, u, v, n * 0.05, color, &mesh_resmut.properties);
            }
        }
    }

    gizmos_cache.paths.clear();
    if let Some(Ok(lay)) = &solution.current_solution.layout {
        for (path_id, path) in &lay.edge_to_path {
            for vertexpair in path.windows(2) {
                let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                let u = lay.granulated_mesh.position(u_id);
                let v = lay.granulated_mesh.position(v_id);
                let n = lay.granulated_mesh.edge_normal(edge_id);
                add_line2(&mut gizmos_cache.paths, u, v, n * 0.05, hutspot::color::BLACK.into(), &mesh_resmut.properties);
            }
        }
    }

    gizmos_cache.wireframe.clear();
    for edge_id in mesh_resmut.mesh.edge_ids() {
        let (u_id, v_id) = mesh_resmut.mesh.endpoints(edge_id);
        let u = mesh_resmut.mesh.position(u_id);
        let v = mesh_resmut.mesh.position(v_id);
        let n = mesh_resmut.mesh.edge_normal(edge_id);
        add_line2(
            &mut gizmos_cache.wireframe,
            u,
            v,
            n * 0.05,
            hutspot::color::GRIJS.into(),
            &mesh_resmut.properties,
        );
    }

    gizmos_cache.vertices.clear();
    for vert_id in mesh_resmut.mesh.vert_ids() {
        let position = mesh_resmut.mesh.position(vert_id);
        let normal = mesh_resmut.mesh.vert_normal(vert_id);
        add_line2(
            &mut gizmos_cache.vertices,
            position,
            position + normal * 0.05,
            Vector3D::new(0., 0., 0.),
            hutspot::color::GRIJS.into(),
            &mesh_resmut.properties,
        );
    }

    gizmos_cache.normals.clear();
    for face_id in mesh_resmut.mesh.face_ids() {
        let position = mesh_resmut.mesh.centroid(face_id);
        let normal = mesh_resmut.mesh.normal(face_id);
        add_line2(
            &mut gizmos_cache.normals,
            position,
            position + normal * 0.05,
            Vector3D::new(0., 0., 0.),
            hutspot::color::GRIJS.into(),
            &mesh_resmut.properties,
        );
    }
}

#[derive(Default, Resource)]
pub struct GizmosCache {
    pub wireframe: Vec<Line>,
    pub vertices: Vec<Line>,
    pub normals: Vec<Line>,
    pub loops: Vec<Line>,
    pub paths: Vec<Line>,

    pub raycaster: Vec<Line>,
}
type Line = (Vec3, Vec3, Color);

// Draws the gizmos. This includes the wireframe, vertices, normals, and raycasts, etc.
pub fn gizmos(mut gizmos: Gizmos, gizmos_cache: Res<GizmosCache>, solution: Res<SolutionResource>, configuration: Res<Configuration>) {
    if configuration.draw_wireframe {
        for &(u, v, c) in &gizmos_cache.wireframe {
            gizmos.line(u, v, c);
        }
    }
    if configuration.draw_vertices {
        for &(u, v, c) in &gizmos_cache.vertices {
            gizmos.line(u, v, c);
        }
    }
    if configuration.draw_normals {
        for &(u, v, c) in &gizmos_cache.normals {
            gizmos.line(u, v, c);
        }
    }
    if configuration.interactive {
        for &(u, v, c) in &gizmos_cache.raycaster {
            gizmos.line(u, v, c);
        }
    }

    // Draw loops if `black` is set to `true`.
    if configuration.black {
        for &(u, v, c) in &gizmos_cache.loops {
            gizmos.line(u, v, c);
        }
    // Draw paths if `black` is set to `false`.
    } else {
        for &(u, v, c) in &gizmos_cache.paths {
            gizmos.line(u, v, c);
        }
    }

    // Polycube wireframe, does not need to be cached, since it is so simple.
    if let Some(primal) = &solution.current_solution.polycube {
        if configuration.black {
            // Draw all loop segments / faces axis aligned.
            for &face_id in &primal.structure.face_ids() {
                let original_id = primal.intersection_to_face.get_by_right(&face_id).unwrap();
                let this_centroid = primal.structure.centroid(face_id);

                let normal = (primal.structure.normal(face_id) as Vector3D).normalize();
                let orientation = to_principal_direction(normal).0;

                for &neighbor_id in &primal.structure.fneighbors(face_id) {
                    let next_original_id = primal.intersection_to_face.get_by_right(&neighbor_id).unwrap();

                    let edge_between = primal.structure.edge_between_faces(face_id, neighbor_id).unwrap().0;
                    let root = primal.structure.root(edge_between);
                    let root_pos = primal.structure.position(root);

                    let segment = solution
                        .current_solution
                        .dual
                        .as_ref()
                        .unwrap()
                        .loop_structure
                        .edge_between_verts(*original_id, *next_original_id)
                        .unwrap()
                        .0;

                    let direction = solution.current_solution.dual.as_ref().unwrap().loop_structure.edges[segment].direction;

                    for side in [Side::Upper, Side::Lower] {
                        let segment_direction = match (orientation, direction) {
                            (PrincipalDirection::X, PrincipalDirection::Y) | (PrincipalDirection::Y, PrincipalDirection::X) => PrincipalDirection::Z,
                            (PrincipalDirection::X, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::X) => PrincipalDirection::Y,
                            (PrincipalDirection::Y, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::Y) => PrincipalDirection::X,
                            _ => unreachable!(),
                        };

                        let mut direction_vector = this_centroid;
                        direction_vector[segment_direction as usize] = root_pos[segment_direction as usize];

                        let mut offset = Vector3D::new(0., 0., 0.);

                        let dist = 0.001 * f64::from(solution.properties.scale);

                        match side {
                            Side::Upper => match direction {
                                PrincipalDirection::X => offset[0] += dist,
                                PrincipalDirection::Y => offset[1] += dist,
                                PrincipalDirection::Z => offset[2] += dist,
                            },
                            Side::Lower => match direction {
                                PrincipalDirection::X => offset[0] -= dist,
                                PrincipalDirection::Y => offset[1] -= dist,
                                PrincipalDirection::Z => offset[2] -= dist,
                            },
                        };

                        let line = DrawableLine::from_line(
                            this_centroid,
                            direction_vector,
                            offset + normal * 0.01,
                            solution.properties.translation,
                            solution.properties.scale,
                        );

                        gizmos.line(line.u, line.v, direction.to_dual_color_sided(side));
                    }
                }
            }
        }

        // Draw the edges of the polycube.
        for edge_id in primal.structure.edge_ids() {
            let endpoints = primal.structure.endpoints(edge_id);
            let u = primal.structure.position(endpoints.0);
            let v = primal.structure.position(endpoints.1);
            let line = DrawableLine::from_line(
                u,
                v,
                primal.structure.normal(primal.structure.face(edge_id)) * 0.001,
                solution.properties.translation,
                solution.properties.scale,
            );
            gizmos.line(line.u, line.v, hutspot::color::BLACK.into());
        }
    }
}

pub fn add_line(lines: &mut Vec<Line>, position: Vector3D, normal: Vector3D, length: f32, color: Color, props: &MeshProperties) {
    let line = DrawableLine::from_vertex(position, normal, length, props.translation, props.scale);
    lines.push((line.u, line.v, color));
}

pub fn add_line2(lines: &mut Vec<Line>, position_a: Vector3D, position_b: Vector3D, offset: Vector3D, color: Color, props: &MeshProperties) {
    let line = DrawableLine::from_line(position_a, position_b, offset, props.translation, props.scale);
    lines.push((line.u, line.v, color));
}
