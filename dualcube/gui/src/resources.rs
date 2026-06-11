//! Application-wide state: the configuration, the input mesh, and the
//! current solution.

use crate::controls::InteractiveMode;
use crate::render::store::MeshProperties;
use crate::render::Objects;
use bevy::prelude::*;
use dualcube::polycube::POLYCUBE;
use dualcube::prelude::*;
use dualcube::solutions::Solution;
use itertools::Itertools;
use mehsh::prelude::*;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use std::collections::HashMap;
use std::sync::Arc;

/// The phases of the polycube pipeline (used to stop the pipeline early).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Phase {
    None,
    Input,
    Loops,
    Dual,
    Layout,
    Polycube,
    #[allow(dead_code)]
    Quad,
}

#[derive(Resource, Debug, Clone)]
pub struct Configuration {
    pub direction: PrincipalDirection,
    pub alpha: f64,

    pub unit: bool,
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

    pub fields_params: dualcube::gfield::FieldParams,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            direction: PrincipalDirection::X,
            alpha: 0.5,

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

            fields_params: dualcube::gfield::FieldParams::default(),
        }
    }
}

/// The input mesh with its lookup structures and per-axis flow graphs.
#[derive(Default, Debug, Clone, Resource)]
pub struct InputResource {
    pub mesh: Arc<mehsh::prelude::Mesh<INPUT>>,
    pub properties: MeshProperties,
    /// Reserved (currently unused, but kept up-to-date with the mesh).
    #[allow(dead_code)]
    pub vertex_lookup: mehsh::prelude::VertLocation<INPUT>,
    pub triangle_lookup: mehsh::prelude::FaceLocation<INPUT>,
    pub flow_graphs: [grapff::fixed::FixedGraph<EdgeID, f64>; 3],
}

impl InputResource {
    pub fn new(mesh: Arc<mehsh::prelude::Mesh<INPUT>>) -> Self {
        if mesh.nr_verts() == 0 {
            return InputResource::default();
        }
        let vertex_lookup = mesh.kdtree();
        let triangle_lookup = mesh.bvh();
        let mut flow_graphs = [
            grapff::fixed::FixedGraph::default(),
            grapff::fixed::FixedGraph::default(),
            grapff::fixed::FixedGraph::default(),
        ];

        let mut properties = MeshProperties::default();
        (properties.scale, properties.translation) = mesh.scale_translation();
        properties.source = String::from("im blue dabadee dabada");

        let nodes = mesh.edge_ids();

        for axis in [
            PrincipalDirection::X,
            PrincipalDirection::Y,
            PrincipalDirection::Z,
        ] {
            let edges = nodes
                .clone()
                .into_par_iter()
                .flat_map(|node| {
                    mesh.neighbor_function_edgegraph()(node)
                        .into_iter()
                        .map(|neighbor| {
                            let face1 = mesh.face(node);
                            let face2 = mesh.face(neighbor);

                            if face1 == face2 {
                                let normal = mesh.normal(face1);
                                let m1 = mesh.position(node);
                                let m2 = mesh.position(neighbor);
                                let direction = m2 - m1;
                                let cross = direction.cross(&normal);
                                let angle = cross.angle(&axis.into());

                                (node, neighbor, angle)
                            } else {
                                assert!(mesh.twin(node) == neighbor);
                                (node, neighbor, 0.)
                            }
                        })
                        .collect_vec()
                })
                .collect::<Vec<_>>();

            flow_graphs[axis as usize] = grapff::fixed::FixedGraph::from(nodes.clone(), edges);
        }

        Self {
            mesh,
            properties,
            vertex_lookup,
            triangle_lookup,
            flow_graphs,
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
