use std::collections::HashMap;

use crate::elements::to_principal_direction;
use crate::elements::PrincipalDirection;
use crate::elements::Side;
use crate::GizmoType;
use crate::GizmosCache;
use crate::InputResource;
use crate::RenderedMesh;
use crate::SolutionResource;
use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::render::camera::Viewport;
use douconel::douconel::FaceID;
use douconel::douconel::VertID;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use serde::Deserialize;
use serde::Serialize;
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController};

#[derive(Component)]
pub struct MainCamera;

#[derive(Component)]
pub struct SubCamera;

const POLYCUBE_OFFSET: Vec3 = Vec3::new(-10000., -10000., -10000.);

#[derive(Resource, Default, Debug, Clone, Serialize, Deserialize)]
pub struct Configuration {
    pub direction: PrincipalDirection,
    pub alpha: i32,
    pub beta: i32,

    pub sides_mask: [u32; 3],

    pub fps: f64,
    pub cur_selected: Option<(FaceID, VertID)>,

    pub black: bool,
    pub interactive: bool,
    pub swap_cameras: bool,
    pub draw_wireframe: bool,
    pub draw_wireframe_granny: bool,
    pub draw_vertices: bool,
    pub draw_normals: bool,
    pub draw_debug: bool,
}

/// Set up
pub fn setup(mut commands: Commands, mut ui: bevy_egui::EguiContexts, mut configuration: ResMut<Configuration>) {
    // Main camera
    commands
        .spawn(Camera3dBundle {
            camera: Camera { order: 0, ..default() },
            ..default()
        })
        .insert((OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(0.0, 5.0, 20.0),
            Vec3::new(0., 0., 0.),
            Vec3::Y,
        ),))
        .insert(MainCamera);

    // Sub camera
    commands
        .spawn(Camera3dBundle {
            camera: Camera { order: 1, ..default() },
            camera_3d: Camera3d {
                clear_color: bevy::core_pipeline::clear_color::ClearColorConfig::None,
                ..default()
            },
            ..default()
        })
        .insert(SubCamera);

    // SETUP FONT
    let mut fonts = bevy_egui::egui::FontDefinitions::default();
    fonts.font_data.insert(
        "BerkeleyMonoTrial".to_owned(),
        bevy_egui::egui::FontData::from_static(include_bytes!("../assets/BerkeleyMonoTrial-Regular.ttf")),
    );
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "BerkeleyMonoTrial".to_owned());
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Monospace)
        .or_default()
        .push("BerkeleyMonoTrial".to_owned());
    ui.ctx_mut().set_fonts(fonts);

    // SETUP DARK THEME
    ui.ctx_mut().set_visuals(bevy_egui::egui::Visuals::dark());
}

// Synchronize the transformations of the cameras
pub fn sync_cameras(main_camera: Query<&Transform, (With<MainCamera>, Without<SubCamera>)>, mut sub_camera: Query<&mut Transform, With<SubCamera>>) {
    for main_c in main_camera.iter() {
        for mut sub_c in &mut sub_camera {
            sub_c.translation = main_c.translation + POLYCUBE_OFFSET;
            sub_c.rotation = main_c.rotation;
        }
    }
}

// Update the FPS counter
pub fn fps(diagnostics: Res<DiagnosticsStore>, mut configuration: ResMut<Configuration>) {
    configuration.fps = -1.;
    if let Some(value) = diagnostics
        .get(FrameTimeDiagnosticsPlugin::FPS)
        .and_then(bevy::diagnostic::Diagnostic::smoothed)
    {
        configuration.fps = value;
    }
}

// Update camera viewports
pub fn set_camera_viewports(
    configuration: Res<Configuration>,
    windows: Query<&Window>,
    mut main_camera: Query<&mut Camera, (With<MainCamera>, Without<SubCamera>)>,
    mut sub_camera: Query<&mut Camera, With<SubCamera>>,
) {
    let window = windows.single();

    let (mut large_camera, mut small_camera) = if configuration.swap_cameras {
        (sub_camera.single_mut(), main_camera.single_mut())
    } else {
        (main_camera.single_mut(), sub_camera.single_mut())
    };

    large_camera.viewport = Some(Viewport {
        physical_position: UVec2::new(0, 0),
        physical_size: UVec2::new(window.resolution.physical_width(), window.resolution.physical_height()),
        ..default()
    });
    small_camera.viewport = Some(Viewport {
        physical_position: UVec2::new(11 * window.resolution.physical_width() / 16, 11 * window.resolution.physical_height() / 16),
        physical_size: UVec2::new(window.resolution.physical_width() / 4, window.resolution.physical_height() / 4),
        ..default()
    });
}

pub fn draw_gizmos(
    mut gizmos: Gizmos,
    gizmos_cache: Res<GizmosCache>,
    configuration: Res<Configuration>,
    solution: Res<SolutionResource>,
    mesh_resmut: Res<InputResource>,
) {
    for &(u, v, c, t) in &gizmos_cache.lines {
        match t {
            GizmoType::Wireframe => {
                if configuration.draw_wireframe {
                    gizmos.line(u, v, c);
                }
            }
            GizmoType::Vertex => {
                if configuration.draw_vertices {
                    gizmos.line(u, v, c);
                }
            }
            GizmoType::Normal => {
                if configuration.draw_normals {
                    gizmos.line(u, v, c);
                }
            }
        }
    }

    if configuration.draw_wireframe_granny {
        if let Some(granny) = &solution.dual.granulated_mesh {
            for edge_id in granny.edge_ids() {
                let (u_id, v_id) = granny.endpoints(edge_id);
                let u = granny.position(u_id);
                let v = granny.position(v_id);

                let line = DrawableLine::from_line(
                    u,
                    v,
                    Vector3D::new(0., 0., 0.),
                    mesh_resmut.properties.translation,
                    mesh_resmut.properties.scale,
                );

                gizmos.line(line.u, line.v, hutspot::color::WHITE.into());
            }
        }
    }

    for (&(face_id, vert_id), sol) in &solution.next {
        let u = mesh_resmut.mesh.position(vert_id);
        let v = mesh_resmut.mesh.centroid(face_id);
        let n = mesh_resmut.mesh.normal(face_id);
        let line = DrawableLine::from_line(u, v, n * 0.01, mesh_resmut.properties.translation, mesh_resmut.properties.scale);
        match sol {
            Some(_) => gizmos.line(line.u, line.v, hutspot::color::GREEN.into()),
            None => gizmos.line(line.u, line.v, hutspot::color::ROODT.into()),
        }
    }

    for &(u, v, c) in &gizmos_cache.raycast {
        gizmos.line(u, v, c);
    }

    for &(u, v, c) in &gizmos_cache.debug {
        gizmos.line(u, v, c);
    }

    // Draw the paths.
    if let Ok(primal) = &solution.primal {
        if let Some(granulated_mesh) = &solution.dual.granulated_mesh {
            for v in primal.vert_ids() {
                let vertex = primal.verts[v].pointer_primal_vertex;

                // draw vertex
                let u = granulated_mesh.position(vertex);
                let n = granulated_mesh.vert_normal(vertex);
                let line = DrawableLine::from_vertex(u, n, 0.05, mesh_resmut.properties.translation, mesh_resmut.properties.scale);
                gizmos.line(line.u, line.v, hutspot::color::BLACK.into());
            }

            for e in primal.edge_ids() {
                let path = &primal.edges[e];
                for vertexpair in path.windows(2) {
                    let u = granulated_mesh.position(vertexpair[0]);
                    let v = granulated_mesh.position(vertexpair[1]);
                    if let Some((edge, _)) = granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]) {
                        let n = granulated_mesh.edge_normal(edge);
                        // // draw the line
                        let line = DrawableLine::from_line(u, v, n * 0.05, mesh_resmut.properties.translation, mesh_resmut.properties.scale);
                        gizmos.line(line.u, line.v, hutspot::color::WHITE.into());
                    }
                }
            }

            // Draw all loop segments
            let loopstruct = solution.dual.get_loop_structure();
            for segment in loopstruct.edge_ids() {
                let pairs_between = solution.dual.get_pairs_of_sequence(&solution.dual.segment_to_edges(segment));
                let direction = solution.dual.segment_to_direction(segment);
                let side = solution.dual.segment_to_side(segment, configuration.sides_mask);

                for edge in pairs_between {
                    let mut offset = Vector3D::new(0., 0., 0.);

                    let dist = 0.001 * mesh_resmut.properties.scale as f64;

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

                    let u = mesh_resmut.mesh.midpoint(edge[0]);
                    let v = mesh_resmut.mesh.midpoint(edge[1]);
                    let n = mesh_resmut.mesh.edge_normal(edge[0]);

                    let line = DrawableLine::from_line(u, v, offset + n * 0.05, mesh_resmut.properties.translation, mesh_resmut.properties.scale);

                    gizmos.line(line.u, line.v, direction.to_dual_color_sided(side));
                }

                // POLYCUBE :::
                // Draw all loop segments / faces axis aligned.
                for (face_id, original_id) in &primal.faces {
                    let this_centroid = primal.centroid(face_id);

                    let normal = (primal.normal(face_id) as Vector3D).normalize();
                    let orientation = to_principal_direction(normal).0;

                    for &neighbor_id in &primal.fneighbors(face_id) {
                        let next_original_id = &primal.faces[neighbor_id];

                        let edge_between = primal.edge_between_faces(face_id, neighbor_id).unwrap().0;
                        let root = primal.root(edge_between);
                        let root_pos = primal.position(root);

                        let segment = solution
                            .dual
                            .get_loop_structure()
                            .edge_between_verts(*original_id, *next_original_id)
                            .unwrap()
                            .0;

                        let direction = solution.dual.segment_to_direction(segment);

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

                            let dist = 0.001 * solution.properties.scale as f64;

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
                                solution.properties.translation + Vector3D::new(POLYCUBE_OFFSET.x as f64, POLYCUBE_OFFSET.y as f64, POLYCUBE_OFFSET.z as f64),
                                solution.properties.scale,
                            );

                            gizmos.line(line.u, line.v, direction.to_dual_color_sided(side));
                        }
                    }
                }
            }
        }
    }
}

// This function should be called when the mesh (RenderedMesh) is changed, to make sure that modifications are visualized.
pub fn update_mesh(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    configuration: Res<Configuration>,

    rendered_mesh_query: Query<Entity, With<RenderedMesh>>,
) {
    if !mesh_resmut.is_changed() && !solution.is_changed() {
        return;
    }
    info!("Mesh has been changed. Updating the Bevy render.");

    for entity in rendered_mesh_query.iter() {
        commands.entity(entity).despawn();
    }
    info!("Despawning any current meshes.");

    if mesh_resmut.mesh.faces.is_empty() {
        warn!("Mesh is empty (?)");
        return;
    }

    let color_map = HashMap::new();
    let mesh = mesh_resmut.mesh.bevy(&color_map);

    // Spawn new mesh
    commands.spawn((
        MaterialMeshBundle {
            mesh: meshes.add(mesh.clone()),
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
                perceptual_roughness: 0.9,
                ..default()
            }),
            ..default()
        },
        RenderedMesh,
    ));

    let mut color_map = HashMap::new();
    if let Ok(polycube) = &solution.primal {
        for &face_id in &polycube.face_ids() {
            let normal = (polycube.normal(face_id) as Vector3D).normalize();
            let (dir, side) = to_principal_direction(normal);
            let color = dir.to_primal_color_sided(side);
            color_map.insert(face_id, [color.r(), color.g(), color.b()]);
        }
        let color_map = HashMap::new();

        let mesh = polycube.bevy(&color_map);
        let aabb = mesh.compute_aabb().unwrap();
        let scale = 10. * (1. / aabb.half_extents.max_element());
        let translation = -scale * aabb.center;

        solution.properties.translation = Vector3D::new(translation.x.into(), translation.y.into(), translation.z.into());
        solution.properties.scale = scale;

        // Spawn new mesh
        commands.spawn((
            MaterialMeshBundle {
                mesh: meshes.add(mesh),
                transform: Transform {
                    translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32) + POLYCUBE_OFFSET,
                    rotation: Quat::from_rotation_z(0f32),
                    scale: Vec3::splat(scale),
                },
                material: materials.add(StandardMaterial {
                    perceptual_roughness: 0.9,
                    ..default()
                }),
                ..default()
            },
            RenderedMesh,
        ));
    }

    // let mut color_map = HashMap::new();
    // if let Ok(polycube) = &solution.primal {
    //     for &face_id in &polycube.face_ids() {
    //         let normal = (polycube.normal(face_id) as Vector3D).normalize();
    //         let (dir, side) = to_principal_direction(normal);
    //         let color = dir.to_primal_color();
    //         color_map.insert(face_id, [color.r(), color.g(), color.b()]);
    //     }
    //     let mesh = polycube.bevy(&color_map);
    //     let aabb = mesh.compute_aabb().unwrap();
    //     let scale = 10. * (1. / aabb.half_extents.max_element());
    //     let translation = -scale * aabb.center;

    //     // Spawn new mesh
    //     commands.spawn((
    //         MaterialMeshBundle {
    //             mesh: meshes.add(mesh),
    //             transform: Transform {
    //                 translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
    //                 rotation: Quat::from_rotation_z(0f32),
    //                 scale: Vec3::splat(scale),
    //             },
    //             material: materials.add(StandardMaterial {
    //                 perceptual_roughness: 0.9,
    //                 ..default()
    //             }),
    //             ..default()
    //         },
    //         RenderedMesh,
    //     ));
    // }
}
