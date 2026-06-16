//! Rendering: cameras, render-object stores, and per-object scene construction.

pub mod camera;
pub mod gizmos;
pub mod objects {
    pub mod input_mesh;
    pub mod polycube;
    pub mod polycube_map;
    pub mod quad_mesh;
}
pub mod store;

use crate::{
    colors,
    resources::{Configuration, SolutionResource},
};
use bevy::prelude::*;
use dualcube::prelude::*;
use enum_iterator::{all, Sequence};
use store::{FlowGraphGizmo, RenderObject, RenderObjectStore};

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
                    update_flow_graph_top_percent,
                    store::update_render_settings,
                    store::respawn_renders,
                ),
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
    fn spec(
        self,
    ) -> (
        &'static str,
        fn(&Solution, &Configuration) -> Option<RenderObject>,
    ) {
        match self {
            Self::InputMesh => ("input mesh", objects::input_mesh::build),
            Self::Polycube => ("polycube", objects::polycube::build),
            Self::PolycubeMap => ("polycube-map", objects::polycube_map::build),
            Self::QuadMesh => ("quad mesh", objects::quad_mesh::build),
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
pub fn refresh(solution: &Solution, configuration: &Configuration) -> RenderObjectStore {
    let mut store = RenderObjectStore::default();
    for object in all::<Objects>() {
        let (_, build) = object.spec();
        if let Some(render_object) = build(solution, configuration) {
            store.add_object(object, render_object);
        }
    }
    store
}

/// The configured background color as a Bevy color.
fn update_flow_graph_top_percent(
    configuration: Res<Configuration>,
    solution: Res<SolutionResource>,
    mut gizmo_assets: ResMut<Assets<GizmoAsset>>,
    flow_graph_gizmos: Query<(&FlowGraphGizmo, &Gizmo)>,
    added_flow_graph_gizmos: Query<(), Added<FlowGraphGizmo>>,
    mut previous_top_percent: Local<Option<f32>>,
) {
    let top_percent_changed = !previous_top_percent.is_some_and(|previous| {
        (previous - configuration.flow_graph_top_percent).abs() <= f32::EPSILON
    });
    let flow_graph_gizmos_spawned = !added_flow_graph_gizmos.is_empty();

    if !top_percent_changed && !flow_graph_gizmos_spawned {
        return;
    }

    *previous_top_percent = Some(configuration.flow_graph_top_percent);

    let Some(flow_graphs) = &solution.current_solution.flow_graphs else {
        return;
    };

    let input = solution.current_solution.mesh_ref.as_ref();
    let (scale, translation) = input.scale_translation();

    for (flow_graph_gizmo, gizmo) in &flow_graph_gizmos {
        let graph = &flow_graphs[flow_graph_gizmo.direction as usize];
        let color = colors::from_direction(flow_graph_gizmo.direction, None, None);
        if let Some(asset) = gizmo_assets.get_mut(&gizmo.handle) {
            *asset = gizmos::flow_graph_gizmos(
                &graph.edges(),
                input,
                color,
                translation,
                scale,
                configuration.flow_graph_top_percent,
            );
        }
    }
}

/// The configured background color as a Bevy color.
pub(crate) fn clear_color(configuration: &Configuration) -> Color {
    Color::srgb_u8(
        configuration.clear_color[0],
        configuration.clear_color[1],
        configuration.clear_color[2],
    )
}
