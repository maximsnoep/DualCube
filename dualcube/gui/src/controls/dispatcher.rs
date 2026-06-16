use super::loop_modification::loop_modification_system;
use super::segmentation_modification::segmentation_modification_system;
use super::shared::{CacheResource, InteractiveMode};
use crate::colors;
use crate::jobs::JobRequest;
use crate::render::gizmos::{vector3d_to_vec3, view_to_world, PerpetualGizmos};
use crate::render::store::MainMesh;
use crate::resources::{Configuration, InputResource, SolutionResource};
use bevy::picking::backend::ray::RayMap;
use bevy::prelude::*;
use itertools::Itertools;
use mehsh::prelude::*;

pub fn control_system(
    ray_map: Res<RayMap>,
    mut ray_cast: MeshRayCast,
    foo_query: Query<(), With<MainMesh>>,
    mouse: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mesh_resmut: Res<InputResource>,
    solution: ResMut<SolutionResource>,
    cache: ResMut<CacheResource>,
    mut gizmos: Gizmos<PerpetualGizmos>,
    mut configuration: ResMut<Configuration>,
    jobs: MessageWriter<JobRequest>,
) -> Result<(), BevyError> {
    configuration.raycasted = None;
    configuration.selected = None;

    if keyboard.pressed(KeyCode::ControlLeft) || mouse.pressed(MouseButton::Right) {
        return Ok(());
    }

    if configuration.interactive_mode == InteractiveMode::None {
        return Ok(());
    }

    // Only ray cast against entities with the main mesh marker component.
    let filter = |entity| foo_query.contains(entity);
    let settings = MeshRayCastSettings::default().with_filter(&filter);

    let Some(&(_, intersection)) = ray_map
        .iter()
        .filter_map(|(_, ray)| {
            let (_, hit) = ray_cast.cast_ray(*ray, &settings).first()?;
            Some((*ray, hit.point))
        })
        .collect_vec()
        .first()
    else {
        return Ok(());
    };

    let position = view_to_world(
        intersection,
        mesh_resmut.properties.translation,
        mesh_resmut.properties.scale,
    );
    let nearest_face = mesh_resmut.triangle_lookup.nearest(&position.into());

    // Draw the current raycast hit.
    let isometry1 = Isometry3d::new(
        intersection,
        Quat::from_rotation_arc(
            Vec3::Z,
            vector3d_to_vec3(mesh_resmut.mesh.normal(nearest_face)).normalize(),
        ),
    );
    gizmos.circle(isometry1, 0.1, colors::to_bevy(colors::BLACK));

    match configuration.interactive_mode {
        InteractiveMode::None => Ok(()),
        InteractiveMode::LoopModification => loop_modification_system(
            mouse,
            keyboard,
            mesh_resmut,
            solution,
            cache,
            gizmos,
            configuration,
            jobs,
            position,
            nearest_face,
        ),
        InteractiveMode::SegmentationModification => segmentation_modification_system(
            mouse,
            keyboard,
            mesh_resmut,
            solution,
            cache,
            gizmos,
            configuration,
            jobs,
            position,
            nearest_face,
        ),
    }
}
