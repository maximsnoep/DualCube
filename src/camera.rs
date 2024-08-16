use crate::{Configuration, OBJ_1_OFFSET, OBJ_2_OFFSET, OBJ_3_OFFSET};
use bevy::core_pipeline::clear_color::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::camera::Viewport;
use bevy::render::view::RenderLayers;
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController};

#[derive(Component)]
pub struct Obj1Camera;

#[derive(Component)]
pub struct Obj2Camera;

#[derive(Component)]
pub struct Obj3Camera;

pub fn setup(mut commands: Commands) {
    // Obj 1 camera
    commands
        .spawn(Camera3dBundle {
            tonemapping: Tonemapping::None,
            camera_3d: Camera3d {
                clear_color: ClearColorConfig::None,
                ..default()
            },
            ..default()
        })
        .insert((OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(0.0, 5.0, 20.0) + Vec3::new(OBJ_1_OFFSET.x as f32, OBJ_1_OFFSET.y as f32, OBJ_1_OFFSET.z as f32),
            Vec3::new(0., 0., 0.) + Vec3::new(OBJ_1_OFFSET.x as f32, OBJ_1_OFFSET.y as f32, OBJ_1_OFFSET.z as f32),
            Vec3::Y,
        ),))
        .insert(Obj1Camera)
        .insert(RenderLayers::from_layers(&[0, 1]));

    // Obj 2 camera
    commands
        .spawn(Camera3dBundle {
            tonemapping: Tonemapping::None,
            camera_3d: Camera3d {
                clear_color: ClearColorConfig::None,
                ..default()
            },
            ..default()
        })
        .insert(Obj2Camera)
        .insert(RenderLayers::from_layers(&[0, 2]));

    // Obj 3 camera
    commands
        .spawn(Camera3dBundle {
            tonemapping: Tonemapping::None,
            camera_3d: Camera3d {
                clear_color: ClearColorConfig::None,
                ..default()
            },
            ..default()
        })
        .insert(Obj3Camera)
        .insert(RenderLayers::from_layers(&[0, 3]));

    // pseudocam for clearcolor
    commands.spawn(Camera3dBundle { ..default() }).insert(RenderLayers::from_layers(&[9]));
}

pub fn update(
    mut obj1camera: Query<(&mut Camera, &mut Transform), (With<Obj1Camera>, Without<Obj2Camera>, Without<Obj3Camera>)>,
    mut obj2camera: Query<(&mut Camera, &mut Transform), (With<Obj2Camera>, Without<Obj1Camera>, Without<Obj3Camera>)>,
    mut obj3camera: Query<(&mut Camera, &mut Transform), (With<Obj3Camera>, Without<Obj1Camera>, Without<Obj2Camera>)>,
    configuration: Res<Configuration>,
    windows: Query<&Window>,
) {
    let obj1camera_mut = &mut obj1camera.single_mut();
    let obj2camera_mut = &mut obj2camera.single_mut();
    let obj3camera_mut = &mut obj3camera.single_mut();

    // Synchronizes the transformations of the main and sub cameras. Such that the sub camera moves with the main camera.
    let normalized_translation = obj1camera_mut.1.translation - Vec3::new(OBJ_1_OFFSET.x as f32, OBJ_1_OFFSET.y as f32, OBJ_1_OFFSET.z as f32);
    obj2camera_mut.1.translation = normalized_translation + Vec3::new(OBJ_2_OFFSET.x as f32, OBJ_2_OFFSET.y as f32, OBJ_2_OFFSET.z as f32);
    obj2camera_mut.1.rotation = obj1camera_mut.1.rotation;
    obj3camera_mut.1.translation = normalized_translation + Vec3::new(OBJ_3_OFFSET.x as f32, OBJ_3_OFFSET.y as f32, OBJ_3_OFFSET.z as f32);
    obj3camera_mut.1.rotation = obj1camera_mut.1.rotation;

    // Updates the camera viewports. Default, the main camera spans the whole screen, and the sub camera is a quarter-sized camera in the bottom left corner.
    // If `configuration.swap_cameras` is set to `true`, the sub camera spans the whole screen, and the main camera is a quarter-sized camera in the bottom left corner.
    let (cam1, cam2, cam3) = match (configuration.swap_cameras, !configuration.black) {
        (false, false) => (obj1camera_mut, obj2camera_mut, obj3camera_mut),
        (false, true) => (obj3camera_mut, obj2camera_mut, obj1camera_mut),
        (true, false) => (obj2camera_mut, obj1camera_mut, obj3camera_mut),
        (true, true) => (obj2camera_mut, obj3camera_mut, obj1camera_mut),
    };

    // Set viewports
    let window = windows.single();
    cam1.0.viewport = Some(Viewport {
        physical_position: UVec2::new(0, 2 * window.resolution.physical_height() / 10),
        physical_size: UVec2::new(window.resolution.physical_width(), 8 * window.resolution.physical_height() / 10),
        ..default()
    });
    cam1.0.order = 99;

    cam2.0.viewport = Some(Viewport {
        physical_position: UVec2::new(11 * window.resolution.physical_width() / 16, 11 * window.resolution.physical_height() / 16),
        physical_size: UVec2::new(window.resolution.physical_width() / 4, window.resolution.physical_height() / 4),
        ..default()
    });
    cam2.0.order = 2;

    cam3.0.viewport = Some(Viewport {
        physical_position: UVec2::new(11 * window.resolution.physical_width() / 16, 6 * window.resolution.physical_height() / 16),
        physical_size: UVec2::new(window.resolution.physical_width() / 4, window.resolution.physical_height() / 4),
        ..default()
    });
    cam3.0.order = 1;
}
