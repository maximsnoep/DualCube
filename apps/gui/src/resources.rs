//! Application-wide state: the configuration, the input mesh, and the
//! current solution.

use crate::controls::InteractiveMode;
use crate::render::Objects;
use crate::render::store::MeshProperties;
use bevy::prelude::*;
use dualcube::prelude::*;
use std::sync::Arc;

/// The phases of the polycube pipeline (used to stop the pipeline early).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Phase {
    None,
    Loops,
    Dual,
    Layout,
    Polycube,
}

#[derive(Resource, Debug, Clone)]
pub struct Configuration {
    pub direction: Direction,

    pub unit: bool,
    #[allow(dead_code)]
    pub omega: usize,
    pub iterations: usize,
    pub pool1: usize,
    pub pool2: usize,

    pub raycasted: Option<[EdgeID; 2]>,
    pub selected: Option<[EdgeID; 2]>,

    pub loop_anchors: Vec<[EdgeID; 2]>,

    /// Reserved (currently unused).
    #[allow(dead_code)]
    pub automatic: bool,

    pub interactive_mode: InteractiveMode,

    pub window_shows_object: [Objects; 3],

    pub camera_rotate_sensitivity: f32,
    pub camera_translate_sensitivity: f32,
    pub camera_zoom_sensitivity: f32,
    /// Reserved (currently unused).
    #[allow(dead_code)]
    pub automatic_rotation_camera: bool,

    pub camera_up: Vec3,

    pub stop: Phase,

    pub clear_color: [u8; 3],

    pub fields_params: FieldParams,
    pub graph_params: GraphParams,
    pub flow_graph_top_percent: f32,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            direction: Direction::X,

            unit: true,
            omega: 5,
            iterations: 10,
            pool1: 10,
            pool2: 30,

            loop_anchors: vec![],

            camera_up: Vec3::Y,

            stop: Phase::None,

            raycasted: None,
            selected: None,
            automatic: false,
            interactive_mode: InteractiveMode::None,
            window_shows_object: [Objects::PolycubeMap, Objects::QuadMesh, Objects::Polycube],
            clear_color: if cfg!(feature = "light_mode") {
                [255, 255, 255]
            } else {
                [27, 27, 27]
            },
            camera_rotate_sensitivity: 0.2,
            camera_translate_sensitivity: 2.,
            camera_zoom_sensitivity: 0.2,
            automatic_rotation_camera: true,

            fields_params: FieldParams::default(),
            graph_params: GraphParams::default(),
            flow_graph_top_percent: 20.0,
        }
    }
}

/// The input mesh with its lookup structures and per-axis flow graphs.
#[derive(Default, Debug, Clone, Resource)]
pub struct InputResource {
    pub mesh: Arc<mehsh::prelude::Mesh<INPUT>>,
    pub properties: MeshProperties,
    #[allow(dead_code)]
    pub vertex_lookup: VertLocation<INPUT>,
    pub triangle_lookup: FaceLocation<INPUT>,
}

impl InputResource {
    pub fn new(mesh: Arc<mehsh::prelude::Mesh<INPUT>>) -> Self {
        if mesh.nr_verts() == 0 {
            return InputResource::default();
        }
        let vertex_lookup = mesh.kdtree();
        let triangle_lookup = mesh.bvh();

        let mut properties = MeshProperties::default();
        (properties.scale, properties.translation) = mesh.scale_translation();
        properties.source = String::from("im blue dabadee dabada");

        Self {
            mesh,
            properties,
            vertex_lookup,
            triangle_lookup,
        }
    }
}

/// The current solution and the candidate solutions per loop seed.
#[derive(Debug, Clone, Resource)]
pub struct SolutionResource {
    pub current_solution: Solution,
    pub next: [HashMap<[EdgeID; 2], Option<Solution>>; 3],
    pub selected_corner: Option<VertKey<POLYCUBE>>,
}

impl Default for SolutionResource {
    fn default() -> Self {
        Self {
            current_solution: Solution::new(Arc::new(mehsh::mesh::connectivity::Mesh::default())),
            next: [HashMap::new(), HashMap::new(), HashMap::new()],
            selected_corner: None,
        }
    }
}
