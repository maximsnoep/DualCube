//! Rendering: cameras, render-object stores, and per-object scene construction.

pub mod camera;
pub mod gizmos;
mod input_mesh;
mod polycube;
mod polycube_map;
mod quad_mesh;
pub mod store;

use crate::resources::Configuration;
use bevy::prelude::*;
use bevy::time::common_conditions::on_timer;
use dualcube::prelude::*;
use enum_iterator::{all, Sequence};
use std::time::Duration;
use store::{RenderObject, RenderObjectStore};

/// Registers the render resources, the cameras, and the (re)spawn systems.
pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<store::RenderObjectStore>()
            .init_resource::<store::RenderObjectSettingStore>()
            .init_resource::<camera::CameraHandles>()
            .init_gizmo_group::<gizmos::PerpetualGizmos>()
            .add_systems(Startup, (camera::setup, gizmos::setup))
            .add_systems(
                Update,
                (
                    camera::update,
                    camera::update_camera_settings,
                    store::update_render_settings,
                ),
            )
            .add_systems(
                FixedUpdate,
                store::respawn_renders.run_if(on_timer(Duration::from_millis(1000))),
            );
    }
}

/// The scenes the application can show.
///
/// To add a new scene: add a variant, one line in [`Objects::spec`], and a
/// module with a `build` function that constructs its [`RenderObject`].
#[derive(PartialEq, Eq, Hash, Debug, Copy, Clone, Default, Sequence)]
pub enum Objects {
    InputMesh,
    #[default]
    Polycube,
    PolycubeMap,
    QuadMesh,
}

impl Objects {
    /// The display name and scene builder of each object.
    fn spec(self) -> (&'static str, fn(&Solution) -> Option<RenderObject>) {
        match self {
            Self::InputMesh => ("input mesh", input_mesh::build),
            Self::Polycube => ("polycube", polycube::build),
            Self::PolycubeMap => ("polycube-map", polycube_map::build),
            Self::QuadMesh => ("quad mesh", quad_mesh::build),
        }
    }
}

impl std::fmt::Display for Objects {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.spec().0)
    }
}

/// World-space offset of each scene, spaced far enough apart that the scenes
/// never overlap.
impl From<Objects> for Vec3 {
    fn from(object: Objects) -> Self {
        Self::new(0., 0., 1_000. * object as u8 as f32)
    }
}

/// Builds the complete [`RenderObjectStore`] for the given solution.
#[must_use]
pub fn refresh(solution: &Solution) -> RenderObjectStore {
    let mut store = RenderObjectStore::default();
    for object in all::<Objects>() {
        let (_, build) = object.spec();
        if let Some(render_object) = build(solution) {
            store.add_object(object, render_object);
        }
    }
    store
}

/// The configured background color as a Bevy color.
pub(crate) fn clear_color(configuration: &Configuration) -> Color {
    Color::srgb_u8(
        configuration.clear_color[0],
        configuration.clear_color[1],
        configuration.clear_color[2],
    )
}
