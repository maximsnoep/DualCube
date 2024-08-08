use crate::elements::to_principal_direction;
use crate::elements::PrincipalDirection;
use crate::InputResource;
use crate::RenderedMesh;
use crate::SolutionResource;
use crate::BACKGROUND_COLOR;
use crate::MESH_OFFSET;
use crate::POLYCUBE_OFFSET;
use bevy::core_pipeline::clear_color::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::render::camera::Viewport;
use bevy::render::view::RenderLayers;
use douconel::douconel::FaceID;
use douconel::douconel::VertID;
use hutspot::geom::Vector3D;
use serde::Deserialize;
use serde::Serialize;
use shape::Circle;
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController};
use std::collections::HashMap;

#[derive(Component)]
pub struct MainCamera;

#[derive(Component)]
pub struct SubCamera;

#[derive(Resource, Default, Debug, Clone, Serialize, Deserialize)]
pub struct Configuration {
    pub direction: PrincipalDirection,
    pub alpha: i32,
    pub beta: i32,

    pub sides_mask: [u32; 3],

    pub fps: f64,
    pub selected_face: Option<(FaceID, VertID)>,
    pub selected_solution: Option<(FaceID, VertID)>,

    pub black: bool,
    pub interactive: bool,
    pub swap_cameras: bool,
    pub draw_wireframe: bool,
    pub draw_vertices: bool,
    pub draw_normals: bool,
}

// Set up
pub fn setup(mut commands: Commands, mut ui: bevy_egui::EguiContexts, configuration: Res<Configuration>) {
    // Main camera
    commands
        .spawn(Camera3dBundle {
            camera: Camera { order: 0, ..default() },
            tonemapping: Tonemapping::None,
            ..default()
        })
        .insert((OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(0.0, 5.0, 20.0) + Vec3::new(MESH_OFFSET.x as f32, MESH_OFFSET.y as f32, MESH_OFFSET.z as f32),
            Vec3::new(0., 0., 0.) + Vec3::new(MESH_OFFSET.x as f32, MESH_OFFSET.y as f32, MESH_OFFSET.z as f32),
            Vec3::Y,
        ),))
        .insert(MainCamera)
        .insert(RenderLayers::from_layers(&[0, 1]));

    // Sub camera
    commands
        .spawn(Camera3dBundle {
            camera: Camera { order: 1, ..default() },
            camera_3d: Camera3d {
                clear_color: ClearColorConfig::None,
                ..default()
            },
            tonemapping: Tonemapping::None,
            ..default()
        })
        .insert(SubCamera)
        .insert(RenderLayers::from_layers(&[0, 2]));

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

// Synchronizes the transformations of the main and sub cameras. Such that the sub camera moves with the main camera.
pub fn sync_cameras(main_camera: Query<&Transform, (With<MainCamera>, Without<SubCamera>)>, mut sub_camera: Query<&mut Transform, With<SubCamera>>) {
    let main_c = main_camera.single();
    let sub_c = &mut sub_camera.single_mut();

    sub_c.translation = main_c.translation - Vec3::new(MESH_OFFSET.x as f32, MESH_OFFSET.y as f32, MESH_OFFSET.z as f32)
        + Vec3::new(POLYCUBE_OFFSET.x as f32, POLYCUBE_OFFSET.y as f32, POLYCUBE_OFFSET.z as f32);
    sub_c.rotation = main_c.rotation;
}

// Updates the FPS counter in `configuration`.
pub fn fps(diagnostics: Res<DiagnosticsStore>, mut configuration: ResMut<Configuration>) {
    configuration.fps = -1.;
    if let Some(value) = diagnostics
        .get(FrameTimeDiagnosticsPlugin::FPS)
        .and_then(bevy::diagnostic::Diagnostic::smoothed)
    {
        configuration.fps = value;
    }
}

// Updates the camera viewports. Default, the main camera spans the whole screen, and the sub camera is a quarter-sized camera in the bottom left corner.
// If `configuration.swap_cameras` is set to `true`, the sub camera spans the whole screen, and the main camera is a quarter-sized camera in the bottom left corner.
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

    commands.spawn(PbrBundle {
        mesh: meshes.add(Circle::new(100.0).into()),
        transform: Transform {
            translation: Vec3::new(0., 0., 0.),
            rotation: Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            scale: Vec3::splat(1.),
        },
        material: materials.add(StandardMaterial {
            base_color: BACKGROUND_COLOR,
            perceptual_roughness: 1.,
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
            perceptual_roughness: 1.,
            ..default()
        }),

        ..default()
    });

    let mut mesh_color_map = HashMap::new();
    let mut poly_color_map = HashMap::new();

    if !configuration.black {
        if let Ok(polycube) = &solution.primal {
            for &face_id in &polycube.face_ids() {
                let normal = (polycube.normal(face_id) as Vector3D).normalize();
                let (dir, side) = to_principal_direction(normal);
                let color = dir.to_primal_color_sided(side);
                let c = [color.r(), color.g(), color.b()];

                // Color the mesh color map
                for &face_id_t in &polycube.faces[face_id].1.faces {
                    mesh_color_map.insert(face_id_t, c);
                }

                // Color the polycube color map
                poly_color_map.insert(face_id, c);
            }
        }
    }

    // Spawn the mesh (input model)
    let mesh = if configuration.black || solution.primal.is_err() {
        mesh_resmut.mesh.bevy(&mesh_color_map)
    } else {
        solution.dual.granulated_mesh.as_ref().unwrap().bevy(&mesh_color_map)
    };

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
                perceptual_roughness: 1.,
                ..default()
            }),
            ..default()
        },
        RenderLayers::layer(1),
        RenderedMesh,
    ));

    // Spawn the polycube (if there is one)
    if let Ok(polycube) = &solution.primal {
        let polycube_mesh = polycube.bevy(&poly_color_map);
        let aabb = polycube_mesh.compute_aabb().unwrap();
        let scale = 10. * (1. / aabb.half_extents.max_element());
        let translation = -scale * aabb.center;

        solution.properties.translation = Vector3D::new(translation.x.into(), translation.y.into(), translation.z.into()) + POLYCUBE_OFFSET;
        solution.properties.scale = scale;

        // Spawn new mesh
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
                material: materials.add(StandardMaterial {
                    perceptual_roughness: 1.,
                    ..default()
                }),
                ..default()
            },
            RenderLayers::layer(2),
            RenderedMesh,
        ));
    }
}
