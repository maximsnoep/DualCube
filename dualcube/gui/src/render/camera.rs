//! Camera setup and per-frame camera synchronization.

use super::Objects;
use crate::resources::Configuration;
use crate::ui::dock::UiResource;
use bevy::camera::visibility::RenderLayers;
use bevy::camera::{CameraOutputMode, RenderTarget, ScalingMode, Viewport};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::render_resource::{
    Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages,
};
use bevy_axes_gizmo::AxesGizmoSyncCamera;
use bevy_egui::{EguiGlobalSettings, PrimaryEguiContext};
use bevy_orbit_camera::*;
use bevy_toon::ToonMaterial;
use egui_dock::LeafNode;
use std::ops::Index;
use wgpu_types::BlendState;

const DEFAULT_CAMERA_EYE: Vec3 = Vec3::new(25.0, 25.0, 25.0);
const DEFAULT_CAMERA_TARGET: Vec3 = Vec3::new(0., 0., 0.);
const DEFAULT_CAMERA_TEXTURE_SIZE: u32 = 640 * 2;

/// Marks a camera as rendering one of the [`Objects`].
#[derive(Component, PartialEq, Eq, Hash, Debug, Copy, Clone, Default)]
pub struct CameraFor(pub Objects);

/// The render-to-texture image handles of the sub cameras.
#[derive(Resource, Default)]
pub struct CameraHandles {
    pub map: std::collections::HashMap<CameraFor, Handle<Image>>,
}

/// Spawns all cameras: the egui camera, the user-controlled main camera, and
/// one render-to-texture camera per object.
pub fn setup(
    mut commands: Commands,
    mut egui_global_settings: ResMut<EguiGlobalSettings>,
    mut images: ResMut<Assets<Image>>,
    mut handles: ResMut<CameraHandles>,
    cameras: Query<Entity, With<Camera>>,
    configuration: Res<Configuration>,
) {
    for camera in cameras.iter() {
        commands.entity(camera).despawn();
    }

    // Disable the automatic creation of a primary context to set it up manually for the camera we need.
    egui_global_settings.auto_create_primary_context = false;

    let clear_color = ClearColorConfig::Custom(super::clear_color(&configuration));

    // Egui camera.
    commands.spawn((
        // The `PrimaryEguiContext` component requires everything needed to render a primary context.
        PrimaryEguiContext,
        Camera2d,
        // Setting RenderLayers to none makes sure we won't render anything apart from the UI.
        RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: CameraOutputMode::Write {
                blend_state: Some(BlendState::ALPHA_BLENDING),
                clear_color: ClearColorConfig::None,
            },
            clear_color: ClearColorConfig::Custom(bevy::color::Color::NONE),
            ..default()
        },
    ));

    // Main camera. This is the camera that the user can control.
    commands.spawn((
        Camera3d::default(),
        Camera {
            clear_color: clear_color.clone(),
            ..Default::default()
        },
        AxesGizmoSyncCamera,
        Tonemapping::None,
        bevy_blossom::CameraMarker,
        bevy_orbit_camera::automatic::Marker,
        OrbitCameraBundle::new(
            Controller {
                mouse_rotate_sensitivity: Vec2::splat(0.2),
                mouse_translate_sensitivity: Vec2::splat(2.),
                mouse_wheel_zoom_sensitivity: 0.2,
                ..Default::default()
            },
            DEFAULT_CAMERA_EYE + Vec3::from(Objects::InputMesh),
            DEFAULT_CAMERA_TARGET + Vec3::from(Objects::InputMesh),
            Vec3::Y,
        ),
        CameraFor(Objects::InputMesh),
    ));

    // Sub cameras. These cameras render to a texture.
    let mut image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size: Extent3d {
                width: DEFAULT_CAMERA_TEXTURE_SIZE,
                height: DEFAULT_CAMERA_TEXTURE_SIZE,
                ..default()
            },
            dimension: TextureDimension::D2,
            format: TextureFormat::Bgra8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING
                | TextureUsages::COPY_DST
                | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };
    image.resize(image.texture_descriptor.size);

    for object in configuration.window_shows_object {
        let handle = images.add(image.clone());
        handles.map.insert(CameraFor(object), handle.clone());

        // The polycube-like objects get an orthographic projection.
        let projection = if object == Objects::PolycubeMap || object == Objects::Polycube {
            let mut proj = OrthographicProjection::default_3d();
            proj.scaling_mode = ScalingMode::FixedVertical {
                viewport_height: 30.,
            };
            Projection::Orthographic(proj)
        } else {
            Projection::default()
        };

        commands.spawn((
            Camera3d::default(),
            RenderTarget::Image(handle.into()),
            bevy_blossom::CameraMarker,
            Camera {
                clear_color: clear_color.clone(),
                ..Default::default()
            },
            projection,
            Tonemapping::None,
            CameraFor(object),
        ));
    }
}

/// Applies the configured mouse sensitivities to the main camera controller.
pub fn update_camera_settings(
    mut camera_controller: Query<&mut Controller>,
    configuration: Res<Configuration>,
) {
    let Ok(mut main_camera) = camera_controller.single_mut() else {
        // This system runs every frame; only warn once to avoid log spam.
        warn_once!("No main camera controller");
        return;
    };

    let rotate_sensitivity = Vec2::splat(configuration.camera_rotate_sensitivity);
    let translate_sensitivity = Vec2::splat(configuration.camera_translate_sensitivity);

    if main_camera.mouse_rotate_sensitivity != rotate_sensitivity {
        main_camera.mouse_rotate_sensitivity = rotate_sensitivity;
    }
    if main_camera.mouse_translate_sensitivity != translate_sensitivity {
        main_camera.mouse_translate_sensitivity = translate_sensitivity;
    }
    if main_camera.mouse_wheel_zoom_sensitivity != configuration.camera_zoom_sensitivity {
        main_camera.mouse_wheel_zoom_sensitivity = configuration.camera_zoom_sensitivity;
    }
}

/// Fits the main camera to its dock viewport, and keeps the sub cameras (and
/// the toon shading) in sync with the main camera transform.
pub fn update(
    configuration: Res<Configuration>,
    ui_resource: Res<UiResource>,
    mut custom_materials: ResMut<Assets<ToonMaterial>>,
    window: Single<&Window>,
    mut main_camera: Query<(&mut LookTransform, &Transform, &mut Camera), With<Controller>>,
    mut other_cameras: Query<
        (&mut Transform, &mut Projection, &mut Camera, &CameraFor),
        Without<Controller>,
    >,
) {
    let (mut look, main_transform, mut main_camera) = main_camera.single_mut().unwrap();

    let (_, node_index, _) = ui_resource.tree.find_tab(&Objects::InputMesh).unwrap();
    let main_surface = ui_resource.tree.main_surface().clone();
    let main_node = main_surface.index(node_index);
    let main_surface_viewport = match main_node {
        egui_dock::Node::Leaf(LeafNode { viewport, .. }) => *viewport,
        _ => unreachable!(),
    };

    let viewport_width = main_surface_viewport.max[0] - main_surface_viewport.min[0];
    let viewport_height = main_surface_viewport.max[1] - main_surface_viewport.min[1];

    if window.physical_size().x == 0
        || window.physical_size().y == 0
        || viewport_width <= 0.
        || viewport_height <= 0.
        || !viewport_width.is_finite()
        || !viewport_height.is_finite()
    {
        main_camera.is_active = false;
    } else {
        main_camera.is_active = true;
        let viewport = Viewport {
            physical_position: UVec2 {
                x: main_surface_viewport.min[0] as u32,
                y: main_surface_viewport.min[1] as u32,
            },
            physical_size: UVec2 {
                x: viewport_width as u32,
                y: viewport_height as u32,
            },
            ..Default::default()
        };
        let viewport_changed = main_camera.viewport.as_ref().is_none_or(|current| {
            current.physical_position != viewport.physical_position
                || current.physical_size != viewport.physical_size
                || current.depth != viewport.depth
        });
        if viewport_changed {
            main_camera.viewport = Some(viewport);
        }
    }

    look.up = configuration.camera_up.normalize();

    let normalized_translation = main_transform.translation - Vec3::from(Objects::InputMesh);
    let normalized_rotation = main_transform.rotation;
    let distance = normalized_translation.length().max(0.001);

    for (mut sub_transform, mut sub_projection, _sub_camera, sub_object) in &mut other_cameras {
        sub_transform.translation = normalized_translation + Vec3::from(sub_object.0);
        sub_transform.rotation = normalized_rotation;
        if let Projection::Orthographic(orthographic) = sub_projection.as_mut() {
            orthographic.scaling_mode = ScalingMode::FixedVertical {
                viewport_height: distance,
            };
        }
    }

    // The toon shading is based on the view direction of the camera.
    if let Some(view_dir) = normalized_translation.try_normalize() {
        for material in custom_materials.iter_mut() {
            material.1.view_dir = view_dir;
        }
    }
}
