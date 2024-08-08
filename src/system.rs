use crate::elements::PrincipalDirection;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use douconel::douconel::{FaceID, VertID};
use serde::{Deserialize, Serialize};

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
