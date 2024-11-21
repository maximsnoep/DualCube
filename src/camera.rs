use crate::CameraHandles;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages};
use hutspot::geom::Vector3D;
use serde::{Deserialize, Serialize};
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController};

#[derive(PartialEq, Eq, Hash, Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub enum Objects {
    MeshDualLoops,
    #[default]
    PolycubeDual,
    PolycubePrimal,
    MeshPolycubeLayout,
    MeshInput,
    MeshAlignmentScore,
    MeshOrthogonalityScore,
}

impl From<Objects> for String {
    fn from(val: Objects) -> Self {
        match val {
            Objects::MeshDualLoops => "dual loops",
            Objects::PolycubeDual => "polycube dual",
            Objects::PolycubePrimal => "polycube primal",
            Objects::MeshPolycubeLayout => "embedded layout",
            Objects::MeshInput => "input",
            Objects::MeshAlignmentScore => "alignment score",
            Objects::MeshOrthogonalityScore => "orthogonality score",
        }
        .to_owned()
    }
}

impl Objects {
    pub const fn to_offset(self) -> Vector3D {
        match self {
            Self::MeshDualLoops => Vector3D::new(0., 0., 0.),
            Self::PolycubeDual => Vector3D::new(1_000., 0., 0.),
            Self::PolycubePrimal => Vector3D::new(1_000., 1_000., 0.),
            Self::MeshPolycubeLayout => Vector3D::new(1_000., 1_000., 1_000.),
            Self::MeshInput => Vector3D::new(1_000., 0., 1_000.),
            Self::MeshAlignmentScore => Vector3D::new(0., 1_000., 0.),
            Self::MeshOrthogonalityScore => Vector3D::new(0., 1_000., 1_000.),
        }
    }
}

#[derive(Component, PartialEq, Eq, Hash, Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub struct CameraFor(pub Objects);

pub fn reset(commands: &mut Commands, cameras: &Query<Entity, With<CameraFor>>, images: &mut ResMut<Assets<Image>>, handles: &mut ResMut<CameraHandles>) {
    for camera in cameras.iter() {
        commands.entity(camera).despawn();
    }

    // Define all the cameras. There is one main camera. All other cameras are sub cameras (they render to a texture).

    // Main camera (for the mesh with dual loops).
    let offset = Objects::MeshDualLoops.to_offset();
    commands
        .spawn(Camera3dBundle {
            tonemapping: Tonemapping::None,
            ..default()
        })
        .insert((OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(25.0, 25.0, 35.0) + Vec3::new(offset.x as f32, offset.y as f32, offset.z as f32),
            Vec3::new(0., 0., 0.) + Vec3::new(offset.x as f32, offset.y as f32, offset.z as f32),
            Vec3::Y,
        ),))
        .insert(CameraFor(Objects::MeshDualLoops));

    // Sub cameras (render to a texture)
    // This is the texture that will be rendered to.
    let mut image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size: Extent3d {
                width: 640,
                height: 640,
                ..default()
            },
            dimension: TextureDimension::D2,
            format: TextureFormat::Bgra8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };
    image.resize(image.texture_descriptor.size);
    handles.map.clear();

    for object in [
        Objects::PolycubeDual,
        Objects::PolycubePrimal,
        Objects::MeshPolycubeLayout,
        Objects::MeshInput,
        Objects::MeshAlignmentScore,
        Objects::MeshOrthogonalityScore,
    ] {
        handles.map.insert(CameraFor(object), images.add(image.clone()));
        commands
            .spawn(Camera3dBundle {
                camera: Camera {
                    target: handles.map.get(&CameraFor(object)).unwrap().clone().into(),
                    ..Default::default()
                },
                tonemapping: Tonemapping::None,
                ..default()
            })
            .insert(CameraFor(object));
    }
}

pub fn setup(
    mut commands: Commands,
    cameras: Query<Entity, With<CameraFor>>,
    mut images: ResMut<Assets<Image>>,
    mut handles: ResMut<CameraHandles>,
    mut config_store: ResMut<GizmoConfigStore>,
) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    config.line_width = 3.;

    self::reset(&mut commands, &cameras, &mut images, &mut handles);
}

pub fn update(mut cameras: Query<(&mut Camera, &mut Transform, &CameraFor)>) {
    let offset = Objects::MeshDualLoops.to_offset();
    let main_camera = cameras.iter().find(|(_, _, camera_for)| camera_for.0 == Objects::MeshDualLoops).unwrap();
    let normalized_translation = main_camera.1.translation - Vec3::new(offset.x as f32, offset.y as f32, offset.z as f32);
    let normalized_rotation = main_camera.1.rotation;

    for mut camera in &mut cameras {
        let offset = camera.2 .0.to_offset();
        camera.1.translation = normalized_translation + Vec3::new(offset.x as f32, offset.y as f32, offset.z as f32);
        camera.1.rotation = normalized_rotation;
    }
}
