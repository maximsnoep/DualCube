use crate::camera::Objects;
use crate::dual::{to_principal_direction, PrincipalDirection, Side};
use crate::{vec3_to_vector3d, ColorMode, Configuration, InputResource, MainMesh, RenderedMesh, SolutionResource, BACKGROUND_COLOR};
use bevy::prelude::*;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct MeshProperties {
    pub source: String,
    pub scale: f32,
    pub translation: Vector3D,
}

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
    info!("InputResource or SolutionResource change has been detected. Updating all objects and gizmos.");

    info!("Despawning all current objects and gizmos.");
    for entity in rendered_mesh_query.iter() {
        commands.entity(entity).despawn();
    }
    gizmos_cache.lines.clear();

    if mesh_resmut.mesh.faces.is_empty() {
        warn!("Current mesh is empty.");
        return;
    }

    info!("Drawing all objects and gizmos.");

    for object in [
        Objects::MeshDualLoops,
        Objects::PolycubeDual,
        Objects::PolycubePrimal,
        Objects::MeshPolycubeLayout,
        Objects::MeshInput,
        Objects::MeshAlignmentScore,
        Objects::MeshOrthogonalityScore,
    ] {
        let offset = object.to_offset();

        match object {
            Objects::MeshDualLoops => {
                let mesh = mesh_resmut.mesh.bevy(&HashMap::new());
                let aabb = mesh.compute_aabb().unwrap();
                let scale = 10. * (1. / aabb.half_extents.max_element());
                let translation = vec3_to_vector3d((-scale * aabb.center).into()) + offset;
                mesh_resmut.properties.scale = scale;
                mesh_resmut.properties.translation = translation;

                commands.spawn((
                    MaterialMeshBundle {
                        mesh: meshes.add(mesh),
                        transform: Transform {
                            translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                            rotation: Quat::from_rotation_z(0f32),
                            scale: Vec3::splat(scale),
                        },
                        material: materials.add(StandardMaterial { unlit: true, ..default() }),
                        ..default()
                    },
                    RenderedMesh,
                    MainMesh,
                ));

                if let Ok(dual) = &solution.current_solution.dual {
                    for segment_id in dual.loop_structure.edge_ids() {
                        let direction = dual.loop_structure.edges[segment_id].direction;
                        let side = dual.segment_to_side(segment_id, configuration.sides_mask);
                        let mut offset = Vector3D::new(0., 0., 0.);
                        let dist = 0.001 * f64::from(scale);
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
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                offset + n * 0.01,
                                color,
                                &MeshProperties {
                                    source: "dual".to_string(),
                                    scale,
                                    translation,
                                },
                            );
                        }
                    }
                } else {
                    for loop_id in solution.current_solution.loops.keys() {
                        let color = solution.current_solution.loop_to_direction(loop_id).to_dual_color();
                        for edgepair in solution.current_solution.get_pairs_of_loop(loop_id) {
                            let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                            let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                            let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                n * 0.005,
                                color,
                                &MeshProperties {
                                    source: "dual".to_string(),
                                    scale,
                                    translation,
                                },
                            );
                        }
                    }
                }
            }
            Objects::PolycubeDual => {
                if let Some(polycube) = &solution.current_solution.polycube.clone() {
                    let mesh = polycube.structure.bevy(&HashMap::new());
                    let aabb = mesh.compute_aabb().unwrap();
                    let scale = 10. * (1. / aabb.half_extents.max_element());
                    let translation = vec3_to_vector3d((-scale * aabb.center).into()) + offset;
                    commands.spawn((
                        MaterialMeshBundle {
                            mesh: meshes.add(mesh),
                            transform: Transform {
                                translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                                rotation: Quat::from_rotation_z(0f32),
                                scale: Vec3::splat(scale),
                            },
                            material: materials.add(StandardMaterial { unlit: true, ..default() }),
                            ..default()
                        },
                        RenderedMesh,
                    ));

                    // Draw all loop segments / faces axis aligned.
                    for &face_id in &polycube.structure.face_ids() {
                        let original_id = polycube.intersection_to_face.get_by_right(&face_id).unwrap();
                        let this_centroid = polycube.structure.centroid(face_id);

                        let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                        let orientation = to_principal_direction(normal).0;

                        for &neighbor_id in &polycube.structure.fneighbors(face_id) {
                            let next_original_id = polycube.intersection_to_face.get_by_right(&neighbor_id).unwrap();

                            let edge_between = polycube.structure.edge_between_faces(face_id, neighbor_id).unwrap().0;
                            let root = polycube.structure.root(edge_between);
                            let root_pos = polycube.structure.position(root);

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

                                let dist = 0.001 * f64::from(scale);

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

                                let line = DrawableLine::from_line(this_centroid, direction_vector, offset + normal * 0.01, translation, scale);
                                let c = direction.to_dual_color_sided(side);
                                gizmos_cache.lines.push((line.u, line.v, c));
                            }
                        }
                    }

                    // Draw the edges of the polycube.
                    for edge_id in polycube.structure.edge_ids() {
                        let endpoints = polycube.structure.endpoints(edge_id);
                        let u = polycube.structure.position(endpoints.0);
                        let v = polycube.structure.position(endpoints.1);
                        let line = DrawableLine::from_line(u, v, polycube.structure.normal(polycube.structure.face(edge_id)) * 0.005, translation, scale);
                        let c = hutspot::color::GRAY;
                        gizmos_cache.lines.push((line.u, line.v, c));
                    }
                }
            }
            Objects::PolycubePrimal => {
                if let Some(polycube) = &solution.current_solution.polycube.clone() {
                    let mut colormap = HashMap::new();
                    for &face_id in &polycube.structure.face_ids() {
                        let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                        let (dir, side) = to_principal_direction(normal);
                        let color = dir.to_primal_color_sided(side);
                        colormap.insert(face_id, color);
                    }

                    let mesh = polycube.structure.bevy(&colormap);
                    let aabb = mesh.compute_aabb().unwrap();
                    let scale = 10. * (1. / aabb.half_extents.max_element());
                    let translation = vec3_to_vector3d((-scale * aabb.center).into()) + offset;
                    commands.spawn((
                        MaterialMeshBundle {
                            mesh: meshes.add(mesh),
                            transform: Transform {
                                translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                                rotation: Quat::from_rotation_z(0f32),
                                scale: Vec3::splat(scale),
                            },
                            material: materials.add(StandardMaterial { unlit: true, ..default() }),
                            ..default()
                        },
                        RenderedMesh,
                    ));

                    // Draw the edges of the polycube.
                    for edge_id in polycube.structure.edge_ids() {
                        let endpoints = polycube.structure.endpoints(edge_id);
                        let u = polycube.structure.position(endpoints.0);
                        let v = polycube.structure.position(endpoints.1);
                        let line = DrawableLine::from_line(u, v, polycube.structure.normal(polycube.structure.face(edge_id)) * 0.005, translation, scale);
                        let c = hutspot::color::BLACK;
                        gizmos_cache.lines.push((line.u, line.v, c));
                    }
                }
            }
            Objects::MeshPolycubeLayout => {
                if let Some(polycube) = &solution.current_solution.polycube {
                    if let Some(Ok(lay)) = &solution.current_solution.layout {
                        let mut layout_color_map = HashMap::new();

                        for &face_id in &polycube.structure.face_ids() {
                            let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                            let (dir, side) = to_principal_direction(normal);
                            let color = dir.to_primal_color_sided(side);
                            for &triangle_id in &lay.face_to_patch[&face_id].faces {
                                layout_color_map.insert(triangle_id, color);
                            }
                        }

                        let layout = lay.granulated_mesh.bevy(&layout_color_map);
                        let aabb = layout.compute_aabb().unwrap();
                        let scale = 10. * (1. / aabb.half_extents.max_element());
                        let translation = vec3_to_vector3d((-scale * aabb.center).into()) + offset;

                        commands.spawn((
                            MaterialMeshBundle {
                                mesh: meshes.add(layout),
                                transform: Transform {
                                    translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                                    rotation: Quat::from_rotation_z(0f32),
                                    scale: Vec3::splat(scale),
                                },
                                material: materials.add(StandardMaterial { unlit: true, ..default() }),
                                ..default()
                            },
                            RenderedMesh,
                        ));

                        for path in lay.edge_to_path.values() {
                            for vertexpair in path.windows(2) {
                                if lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).is_none() {
                                    println!("Edge between {:?} and {:?} does not exist", vertexpair[0], vertexpair[1]);
                                    continue;
                                }
                                let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                                let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                                let u = lay.granulated_mesh.position(u_id);
                                let v = lay.granulated_mesh.position(v_id);
                                let n = lay.granulated_mesh.edge_normal(edge_id);
                                add_line2(
                                    &mut gizmos_cache.lines,
                                    u,
                                    v,
                                    n * 0.01,
                                    hutspot::color::BLACK,
                                    &MeshProperties {
                                        source: "layout".to_string(),
                                        scale,
                                        translation,
                                    },
                                );
                            }
                        }

                        for edge_id in lay.granulated_mesh.edge_ids() {
                            let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                            let u = lay.granulated_mesh.position(u_id);
                            let v = lay.granulated_mesh.position(v_id);
                            let n = lay.granulated_mesh.edge_normal(edge_id);
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                n * 0.005,
                                hutspot::color::LIGHT_GRAY,
                                &MeshProperties {
                                    source: "input".to_string(),
                                    scale,
                                    translation,
                                },
                            );
                        }
                    }
                }
            }
            Objects::MeshInput => {
                let mesh = mesh_resmut.mesh.bevy(&HashMap::new());
                let aabb = mesh.compute_aabb().unwrap();
                let scale = 10. * (1. / aabb.half_extents.max_element());
                let translation = vec3_to_vector3d((-scale * aabb.center).into()) + offset;

                commands.spawn((
                    MaterialMeshBundle {
                        mesh: meshes.add(mesh),
                        transform: Transform {
                            translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                            rotation: Quat::from_rotation_z(0f32),
                            scale: Vec3::splat(scale),
                        },
                        material: materials.add(StandardMaterial { unlit: true, ..default() }),
                        ..default()
                    },
                    RenderedMesh,
                ));

                for edge_id in mesh_resmut.mesh.edge_ids() {
                    let (u_id, v_id) = mesh_resmut.mesh.endpoints(edge_id);
                    let u = mesh_resmut.mesh.position(u_id);
                    let v = mesh_resmut.mesh.position(v_id);
                    let n = mesh_resmut.mesh.edge_normal(edge_id);
                    add_line2(
                        &mut gizmos_cache.lines,
                        u,
                        v,
                        n * 0.005,
                        hutspot::color::WHITE,
                        &MeshProperties {
                            source: "input".to_string(),
                            scale,
                            translation,
                        },
                    );
                }
            }
            Objects::MeshAlignmentScore => {
                if let Some(Ok(lay)) = &solution.current_solution.layout {
                    let mut layout_color_map = HashMap::new();

                    for &triangle_id in &lay.granulated_mesh.face_ids() {
                        let score = solution.current_solution.alignment_per_triangle[triangle_id];
                        let color = hutspot::color::map(score as f32, &hutspot::color::SCALE_MAGMA);
                        layout_color_map.insert(triangle_id, color);
                    }

                    let layout = lay.granulated_mesh.bevy(&layout_color_map);
                    let aabb = layout.compute_aabb().unwrap();
                    let scale = 10. * (1. / aabb.half_extents.max_element());
                    let translation = vec3_to_vector3d((-scale * aabb.center).into()) + offset;

                    commands.spawn((
                        MaterialMeshBundle {
                            mesh: meshes.add(layout),
                            transform: Transform {
                                translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                                rotation: Quat::from_rotation_z(0f32),
                                scale: Vec3::splat(scale),
                            },
                            material: materials.add(StandardMaterial { unlit: true, ..default() }),
                            ..default()
                        },
                        RenderedMesh,
                    ));

                    for path in lay.edge_to_path.values() {
                        for vertexpair in path.windows(2) {
                            let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                            let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                            let u = lay.granulated_mesh.position(u_id);
                            let v = lay.granulated_mesh.position(v_id);
                            let n = lay.granulated_mesh.edge_normal(edge_id);
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                n * 0.005,
                                hutspot::color::BLACK,
                                &MeshProperties {
                                    source: "layout".to_string(),
                                    scale,
                                    translation,
                                },
                            );
                        }
                    }
                }
            }
            Objects::MeshOrthogonalityScore => {
                if let Some(polycube) = &solution.current_solution.polycube {
                    if let Some(Ok(lay)) = &solution.current_solution.layout {
                        let mut layout_color_map = HashMap::new();

                        for &face_id in &polycube.structure.face_ids() {
                            let score = solution.current_solution.orthogonality_per_patch[face_id];
                            let color = hutspot::color::map(score as f32, &hutspot::color::SCALE_MAGMA);
                            for &triangle_id in &lay.face_to_patch[&face_id].faces {
                                layout_color_map.insert(triangle_id, color);
                            }
                        }

                        let layout = lay.granulated_mesh.bevy(&layout_color_map);
                        let aabb = layout.compute_aabb().unwrap();
                        let scale = 10. * (1. / aabb.half_extents.max_element());
                        let translation = vec3_to_vector3d((-scale * aabb.center).into()) + offset;

                        commands.spawn((
                            MaterialMeshBundle {
                                mesh: meshes.add(layout),
                                transform: Transform {
                                    translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                                    rotation: Quat::from_rotation_z(0f32),
                                    scale: Vec3::splat(scale),
                                },
                                material: materials.add(StandardMaterial { unlit: true, ..default() }),
                                ..default()
                            },
                            RenderedMesh,
                        ));

                        for path in lay.edge_to_path.values() {
                            for vertexpair in path.windows(2) {
                                let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                                let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                                let u = lay.granulated_mesh.position(u_id);
                                let v = lay.granulated_mesh.position(v_id);
                                let n = lay.granulated_mesh.edge_normal(edge_id);
                                add_line2(
                                    &mut gizmos_cache.lines,
                                    u,
                                    v,
                                    n * 0.005,
                                    hutspot::color::BLACK,
                                    &MeshProperties {
                                        source: "layout".to_string(),
                                        scale,
                                        translation,
                                    },
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    // Spawning covers such that the objects are seperated from each other.
    for object in [
        Objects::MeshDualLoops,
        Objects::PolycubeDual,
        Objects::PolycubePrimal,
        Objects::MeshPolycubeLayout,
        Objects::MeshInput,
        Objects::MeshAlignmentScore,
        Objects::MeshOrthogonalityScore,
    ] {
        let offset = object.to_offset();
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(400.)),
                transform: Transform {
                    translation: Vec3::new(offset.x as f32, offset.y as f32, offset.z as f32),
                    ..default()
                },
                material: materials.add(StandardMaterial {
                    base_color: BACKGROUND_COLOR,
                    unlit: true,
                    ..default()
                }),

                ..default()
            },
            RenderedMesh,
        ));
    }
}

#[derive(Default, Resource)]
pub struct GizmosCache {
    pub lines: Vec<Line>,
    pub raycaster: Vec<Line>,
}
type Line = (Vec3, Vec3, hutspot::color::Color);

// Draws the gizmos. This includes the wireframe, vertices, normals, and raycasts, etc.
pub fn gizmos(mut gizmos: Gizmos, gizmos_cache: Res<GizmosCache>, solution: Res<SolutionResource>, configuration: Res<Configuration>) {
    for &(u, v, c) in &gizmos_cache.lines {
        gizmos.line(u, v, Color::srgb(c[0], c[1], c[2]));
    }

    if configuration.interactive {
        for &(u, v, c) in &gizmos_cache.raycaster {
            gizmos.line(u, v, Color::srgb(c[0], c[1], c[2]));
        }
    }
}

pub fn add_line(lines: &mut Vec<Line>, position: Vector3D, normal: Vector3D, length: f32, color: hutspot::color::Color, props: &MeshProperties) {
    let line = DrawableLine::from_vertex(position, normal, length, props.translation, props.scale);
    lines.push((line.u, line.v, color));
}

pub fn add_line2(lines: &mut Vec<Line>, position_a: Vector3D, position_b: Vector3D, offset: Vector3D, color: hutspot::color::Color, props: &MeshProperties) {
    let line = DrawableLine::from_line(position_a, position_b, offset, props.translation, props.scale);
    lines.push((line.u, line.v, color));
}
