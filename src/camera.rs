use crate::{Configuration, MESH_OFFSET, POLYCUBE_OFFSET};
use bevy::core_pipeline::clear_color::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::camera::Viewport;
use bevy::render::view::RenderLayers;
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController};

#[derive(Component)]
pub struct MainCamera;

#[derive(Component)]
pub struct SubCamera;

pub fn setup(mut commands: Commands) {
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
}

pub fn update(
    mut main_camera: Query<(&mut Camera, &mut Transform), (With<MainCamera>, Without<SubCamera>)>,
    mut sub_camera: Query<(&mut Camera, &mut Transform), With<SubCamera>>,
    configuration: Res<Configuration>,
    windows: Query<&Window>,
) {
    let main_c = &mut main_camera.single_mut();
    let sub_c = &mut sub_camera.single_mut();

    // Synchronizes the transformations of the main and sub cameras. Such that the sub camera moves with the main camera.
    sub_c.1.translation = main_c.1.translation - Vec3::new(MESH_OFFSET.x as f32, MESH_OFFSET.y as f32, MESH_OFFSET.z as f32)
        + Vec3::new(POLYCUBE_OFFSET.x as f32, POLYCUBE_OFFSET.y as f32, POLYCUBE_OFFSET.z as f32);
    sub_c.1.rotation = main_c.1.rotation;

    // Updates the camera viewports. Default, the main camera spans the whole screen, and the sub camera is a quarter-sized camera in the bottom left corner.
    // If `configuration.swap_cameras` is set to `true`, the sub camera spans the whole screen, and the main camera is a quarter-sized camera in the bottom left corner.
    let window = windows.single();
    let (large_camera, small_camera) = if configuration.swap_cameras { (sub_c, main_c) } else { (main_c, sub_c) };
    large_camera.0.viewport = Some(Viewport {
        physical_position: UVec2::new(0, 0),
        physical_size: UVec2::new(window.resolution.physical_width(), window.resolution.physical_height()),
        ..default()
    });
    small_camera.0.viewport = Some(Viewport {
        physical_position: UVec2::new(11 * window.resolution.physical_width() / 16, 11 * window.resolution.physical_height() / 16),
        physical_size: UVec2::new(window.resolution.physical_width() / 4, window.resolution.physical_height() / 4),
        ..default()
    });
}
