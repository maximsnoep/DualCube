mod dual;
mod graph;
mod layout;
mod polycube;
mod render;
mod solutions;
mod ui;

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use bevy::tasks::futures_lite::future;
use bevy::tasks::{block_on, AsyncComputeTaskPool, Task};
use bevy::window::WindowMode;
use bevy::winit::WinitWindows;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::Douconel;
use douconel::{douconel::Empty, douconel_embedded::EmbeddedVertex};
use dual::Orientation;
use graph::Graaf;
use hutspot::consts::PI;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ordered_float::OrderedFloat;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use render::{add_line, add_line2, CameraFor, GizmosCache, MeshProperties, Objects};
use serde::{Deserialize, Serialize};
use smooth_bevy_cameras::controllers::orbit::OrbitCameraPlugin;
use smooth_bevy_cameras::LookTransformPlugin;
use solutions::{Loop, Solution};
use std::collections::HashMap;
use std::env;
use std::fmt::Display;
use std::fs::{self, File};
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use winit::window::Icon;

// pub const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::srgb(255. / 255., 255. / 255., 255. / 255.);
pub const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::srgb(27. / 255., 27. / 255., 27. / 255.);

slotmap::new_key_type! {
    pub struct VertID;
    pub struct EdgeID;
    pub struct FaceID;
}

// Principal directions, used to characterize a polycube (each edge and face is associated with a principal direction)
#[derive(Copy, Clone, Default, PartialEq, Eq, Debug, Hash, Serialize, Deserialize)]
pub enum PrincipalDirection {
    #[default]
    X,
    Y,
    Z,
}

impl Display for PrincipalDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::X => write!(f, "X-axis"),
            Self::Y => write!(f, "Y-axis"),
            Self::Z => write!(f, "Z-axis"),
        }
    }
}

#[derive(Copy, Clone, Default, Debug, Serialize, Deserialize)]
pub enum Perspective {
    Primal,
    #[default]
    Dual,
}

pub const fn to_color(direction: PrincipalDirection, perspective: Perspective, orientation: Option<Orientation>) -> hutspot::color::Color {
    match (perspective, direction, orientation) {
        (Perspective::Primal, PrincipalDirection::X, None) => hutspot::color::RED,
        (Perspective::Primal, PrincipalDirection::Y, None) => hutspot::color::BLUE,
        (Perspective::Primal, PrincipalDirection::Z, None) => hutspot::color::YELLOW,
        (Perspective::Primal, PrincipalDirection::X, Some(Orientation::Forwards)) => hutspot::color::RED,
        (Perspective::Primal, PrincipalDirection::X, Some(Orientation::Backwards)) => hutspot::color::RED_LIGHT,
        (Perspective::Primal, PrincipalDirection::Y, Some(Orientation::Forwards)) => hutspot::color::BLUE,
        (Perspective::Primal, PrincipalDirection::Y, Some(Orientation::Backwards)) => hutspot::color::BLUE_LIGHT,
        (Perspective::Primal, PrincipalDirection::Z, Some(Orientation::Forwards)) => hutspot::color::YELLOW,
        (Perspective::Primal, PrincipalDirection::Z, Some(Orientation::Backwards)) => hutspot::color::YELLOW_LIGHT,
        (Perspective::Dual, PrincipalDirection::X, None) => hutspot::color::GREEN,
        (Perspective::Dual, PrincipalDirection::Y, None) => hutspot::color::ORANGE,
        (Perspective::Dual, PrincipalDirection::Z, None) => hutspot::color::PURPLE,
        (Perspective::Dual, PrincipalDirection::X, Some(Orientation::Forwards)) => hutspot::color::GREEN,
        (Perspective::Dual, PrincipalDirection::X, Some(Orientation::Backwards)) => hutspot::color::GREEN_LIGHT,
        (Perspective::Dual, PrincipalDirection::Y, Some(Orientation::Forwards)) => hutspot::color::ORANGE,
        (Perspective::Dual, PrincipalDirection::Y, Some(Orientation::Backwards)) => hutspot::color::ORANG_LIGHT,
        (Perspective::Dual, PrincipalDirection::Z, Some(Orientation::Forwards)) => hutspot::color::PURPLE,
        (Perspective::Dual, PrincipalDirection::Z, Some(Orientation::Backwards)) => hutspot::color::PURPLE_LIGHT,
    }
}

impl From<PrincipalDirection> for Vector3D {
    fn from(dir: PrincipalDirection) -> Self {
        match dir {
            PrincipalDirection::X => Self::new(1., 0., 0.),
            PrincipalDirection::Y => Self::new(0., 1., 0.),
            PrincipalDirection::Z => Self::new(0., 0., 1.),
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum Side {
    Upper,
    Lower,
}

pub fn to_principal_direction(v: Vector3D) -> (PrincipalDirection, Orientation) {
    let x_is_max = v.x.abs() > v.y.abs() && v.x.abs() > v.z.abs();
    let y_is_max = v.y.abs() > v.x.abs() && v.y.abs() > v.z.abs();
    let z_is_max = v.z.abs() > v.x.abs() && v.z.abs() > v.y.abs();
    assert!(x_is_max ^ y_is_max ^ z_is_max, "{v:?}");

    if x_is_max {
        if v.x > 0. {
            (PrincipalDirection::X, Orientation::Forwards)
        } else {
            (PrincipalDirection::X, Orientation::Backwards)
        }
    } else if y_is_max {
        if v.y > 0. {
            (PrincipalDirection::Y, Orientation::Forwards)
        } else {
            (PrincipalDirection::Y, Orientation::Backwards)
        }
    } else if z_is_max {
        if v.z > 0. {
            (PrincipalDirection::Z, Orientation::Forwards)
        } else {
            (PrincipalDirection::Z, Orientation::Backwards)
        }
    } else {
        unreachable!()
    }
}

pub type EmbeddedMesh = Douconel<VertID, EmbeddedVertex, EdgeID, Empty, FaceID, Empty>;

#[derive(Resource, Debug, Clone, Serialize, Deserialize)]
pub struct Configuration {
    pub direction: PrincipalDirection,
    pub alpha: f64,

    pub should_continue: bool,

    pub sides_mask: [u32; 3],

    pub fps: f64,
    pub raycasted: Option<[EdgeID; 2]>,
    pub selected: Option<[EdgeID; 2]>,

    pub automatic: bool,
    pub interactive: bool,
    pub delete_mode: bool,

    pub ui_is_hovered: [bool; 32],
    pub window_shows_object: [Objects; 4],
    pub window_has_size: [f32; 4],
    pub window_has_position: [(f32, f32); 4],

    pub hex_mesh_status: HexMeshStatus,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
enum HexMeshStatus {
    None,
    Loading,
    Done(HexMeshScore),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct HexMeshScore {
    hausdorff: f32,
    avg_jacob: f32,
    min_jacob: f32,
    max_jacob: f32,
    irregular: f32,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            direction: PrincipalDirection::X,
            alpha: 0.5,
            should_continue: false,
            sides_mask: [0, 0, 0],
            fps: -1.,
            raycasted: None,
            selected: None,
            automatic: false,
            interactive: false,
            delete_mode: false,
            ui_is_hovered: [false; 32],
            window_shows_object: [
                Objects::MeshPolycubeLayout,
                Objects::PolycubePrimal,
                Objects::MeshAlignmentScore,
                Objects::MeshInput,
            ],
            window_has_size: [256., 256., 256., 0.],
            window_has_position: [(0., 0.); 4],
            hex_mesh_status: HexMeshStatus::None,
        }
    }
}

// Updates the FPS counter in `configuration`.
pub fn fps(diagnostics: Res<DiagnosticsStore>, mut configuration: ResMut<Configuration>) {
    configuration.fps = -1.;
    if let Some(value) = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(bevy::diagnostic::Diagnostic::smoothed)
    {
        configuration.fps = value;
    }
}

#[derive(Resource, Default)]
struct Tasks {
    generating_chunks: HashMap<usize, Task<Option<Solution>>>,
}

#[derive(Resource, Default)]
struct HexTasks {
    generating_chunks: HashMap<usize, Task<Option<HexMeshScore>>>,
}

#[derive(Resource, Default)]
pub struct CameraHandles {
    pub map: HashMap<CameraFor, Handle<Image>>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SaveStateObject {
    mesh: EmbeddedMesh,
    loops: Vec<Loop>,
    configuration: Configuration,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SaveStateObjectBackwardscompatibility {
    pub mesh: EmbeddedMesh,
    pub loops: Vec<Loop>,
}

#[derive(Component)]
pub struct RenderedMesh;

#[derive(Component)]
pub struct MainMesh;

#[derive(Event, Debug)]
pub enum ActionEvent {
    LoadFile(PathBuf),
    ExportAll,
    ExportState,
    ExportSolution,
    ExportNLR,
    ToHexmesh,
    ResetCamera,
    Mutate,
}

// implement default for KdTree using the New Type Idiom
#[derive(Debug, Clone, Serialize, Deserialize)]
struct TreeD(KdTree<f64, VertID, [f64; 3]>);
impl TreeD {
    fn nearest(&self, point: &[f64; 3]) -> (f64, VertID) {
        let neighbors = self.0.nearest(point, 1, &squared_euclidean).unwrap();
        let (d, i) = neighbors.first().unwrap();
        (*d, **i)
    }

    fn add(&mut self, point: [f64; 3], index: VertID) {
        self.0.add(point, index).unwrap();
    }
}
impl Default for TreeD {
    fn default() -> Self {
        Self(KdTree::new(3))
    }
}

#[derive(Default, Resource)]
pub struct CacheResource {
    cache: [HashMap<[EdgeID; 2], Vec<([EdgeID; 2], OrderedFloat<f64>)>>; 3],
}

#[derive(Default, Debug, Clone, Resource, Serialize, Deserialize)]
pub struct InputResource {
    mesh: Arc<EmbeddedMesh>,
    properties: MeshProperties,
    properties2: MeshProperties,
    vertex_lookup: TreeD,
    flow_graphs: [Graaf<[EdgeID; 2], (f64, f64, f64)>; 3],
}

#[derive(Debug, Clone, Resource)]
pub struct SolutionResource {
    current_solution: Solution,
    next: [HashMap<[EdgeID; 2], Option<Solution>>; 3],
}

impl Default for SolutionResource {
    fn default() -> Self {
        Self {
            current_solution: Solution::new(Arc::new(Douconel::default())),
            next: [HashMap::new(), HashMap::new(), HashMap::new()],
        }
    }
}

fn main() {
    App::new()
        .init_resource::<InputResource>()
        .init_resource::<CacheResource>()
        .init_resource::<GizmosCache>()
        .init_resource::<SolutionResource>()
        .init_resource::<Configuration>()
        .init_resource::<Tasks>()
        .init_resource::<HexTasks>()
        .init_resource::<CameraHandles>()
        .insert_resource(ClearColor(BACKGROUND_COLOR))
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0,
        })
        // Load default plugins
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bloopy".to_string(),
                mode: WindowMode::BorderlessFullscreen,
                ..Default::default()
            }),
            ..Default::default()
        }))
        // Cursor ray
        .add_plugins(CursorRayPlugin)
        // Plugin for FPS
        .add_plugins(FrameTimeDiagnosticsPlugin)
        // Plugin for GUI
        .add_plugins(EguiPlugin)
        // Plugin for smooth camera
        .add_plugins(LookTransformPlugin)
        .add_plugins(OrbitCameraPlugin::default())
        // Setups
        .add_systems(Startup, ui::setup)
        .add_systems(Startup, render::setup)
        .add_systems(Startup, set_window_icon)
        // Updates
        .add_systems(Update, ui::update)
        .add_systems(Update, render::update)
        .add_systems(Update, render::gizmos)
        .add_systems(Update, handle_events)
        .add_systems(Update, handle_solution_tasks)
        .add_systems(Update, handle_hexmesh_tasks)
        .add_systems(Update, raycast)
        .add_systems(Update, fps)
        // .add_systems(Update, handle_tasks)
        .add_event::<ActionEvent>()
        .run();
}

fn set_window_icon(
    // we have to use `NonSend` here
    windows: NonSend<WinitWindows>,
) {
    let image = image::open("assets/logo2.png").expect("Failed to open icon path").into_rgba8();

    let (icon_rgba, icon_width, icon_height) = {
        let (width, height) = image.dimensions();
        let rgba = image.into_raw();
        (rgba, width, height)
    };
    let icon = Icon::from_rgba(icon_rgba, icon_width, icon_height).unwrap();

    // do it for all windows
    for window in windows.windows.values() {
        window.set_window_icon(Some(icon.clone()));
    }
}

pub fn handle_hexmesh_tasks(mut tasks: ResMut<HexTasks>, mut configuration: ResMut<Configuration>) {
    tasks.generating_chunks.retain(|id, task| {
        // check on our task to see how it's doing :)
        let status = block_on(future::poll_once(task));

        // keep the entry in our HashMap only if the task is not done yet
        let retain = status.is_none();

        // if this task is done, handle the data it returned!
        if let Some(chunk_data) = status {
            println!("HexTask {} is done!", id);

            if let Some(label) = chunk_data {
                configuration.hex_mesh_status = HexMeshStatus::Done(label);
            }
        }

        retain
    });
}

pub fn handle_solution_tasks(
    mut tasks: ResMut<Tasks>,
    mut solution: ResMut<SolutionResource>,
    mut ev_writer: EventWriter<ActionEvent>,
    configuration: Res<Configuration>,
) {
    tasks.generating_chunks.retain(|id, task| {
        // check on our task to see how it's doing :)
        let status = block_on(future::poll_once(task));

        // keep the entry in our HashMap only if the task is not done yet
        let retain = status.is_none();

        // if this task is done, handle the data it returned!
        if let Some(chunk_data) = status {
            println!("Task {} is done!", id);

            if let Some(new_solution) = chunk_data {
                if *id == 1 && new_solution.get_quality() * 0.98 > solution.current_solution.get_quality() {
                    println!("New solution is better than current solution. Updating...");
                    solution.current_solution = new_solution;
                } else if *id == 0 && new_solution.get_quality() * 1.01 > solution.current_solution.get_quality() {
                    println!("Solution overwritten.");
                    solution.current_solution = new_solution;
                } else {
                    println!("New solution is worse than current solution. Discarding...");
                }
            }

            if configuration.should_continue {
                ev_writer.send(ActionEvent::Mutate);
            }
        }

        retain
    });
}

pub fn handle_events(
    mut ev_reader: EventReader<ActionEvent>,
    mut mesh_resmut: ResMut<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut configuration: ResMut<Configuration>,

    mut tasks: ResMut<Tasks>,
    mut hextasks: ResMut<HexTasks>,

    mut commands: Commands,
    cameras: Query<Entity, With<Camera>>,

    mut images: ResMut<Assets<Image>>,
    mut camera_handles: ResMut<CameraHandles>,
) {
    for ev in ev_reader.read() {
        info!("Received event {ev:?}. Handling...");
        match ev {
            ActionEvent::LoadFile(path) => {
                let current_configuration = (
                    configuration.window_has_position,
                    configuration.window_has_size,
                    configuration.window_shows_object,
                );

                *configuration = Configuration::default();
                *mesh_resmut = InputResource::default();
                *solution = SolutionResource::default();

                match path.extension().unwrap().to_str() {
                    Some("obj" | "stl") => {
                        mesh_resmut.mesh = match Douconel::from_file(path) {
                            Ok(res) => {
                                let mut mesh = res.0;
                                let n = 10_000 - std::cmp::min(10_000, mesh.nr_edges());
                                mesh.refine(n);
                                Arc::new(mesh)
                            }
                            Err(err) => {
                                panic!("Error while parsing STL file {path:?}: {err:?}");
                            }
                        };

                        solution.current_solution = Solution::new(mesh_resmut.mesh.clone());
                    }
                    Some("save") => {
                        if let Ok(loaded_state) = serde_json::from_reader::<_, SaveStateObject>(BufReader::new(File::open(path).unwrap())) {
                            mesh_resmut.mesh = Arc::new(loaded_state.mesh);
                            solution.current_solution = Solution::new(mesh_resmut.mesh.clone());
                            *configuration = loaded_state.configuration.clone();
                            configuration.window_has_position = current_configuration.0;
                            configuration.window_has_size = current_configuration.1;
                            configuration.window_shows_object = current_configuration.2;

                            for saved_loop in loaded_state.loops {
                                solution.current_solution.add_loop(saved_loop);
                            }
                        } else if let Ok(loaded_state) =
                            serde_json::from_reader::<_, SaveStateObjectBackwardscompatibility>(BufReader::new(File::open(path).unwrap()))
                        {
                            mesh_resmut.mesh = Arc::new(loaded_state.mesh);
                            solution.current_solution = Solution::new(mesh_resmut.mesh.clone());

                            for saved_loop in loaded_state.loops {
                                solution.current_solution.add_loop(saved_loop);
                            }
                        } else {
                            println!("Error while parsing save file {path:?}");
                        }
                    }
                    _ => panic!("File format not supported."),
                }

                if mesh_resmut.mesh.vert_ids().is_empty() {
                    return;
                }

                configuration.hex_mesh_status = HexMeshStatus::None;

                let mut patterns = TreeD::default();
                for v_id in mesh_resmut.mesh.vert_ids() {
                    patterns.add(mesh_resmut.mesh.position(v_id).into(), v_id);
                }
                mesh_resmut.vertex_lookup = patterns;

                mesh_resmut.properties.source = String::from(path.to_str().unwrap());

                let nodes = mesh_resmut
                    .mesh
                    .face_ids()
                    .into_par_iter()
                    .flat_map(|face_id| {
                        let edges = mesh_resmut.mesh.edges(face_id);
                        assert!(edges.len() == 3);
                        vec![
                            [edges[0], edges[1]],
                            [edges[1], edges[0]],
                            [edges[0], edges[2]],
                            [edges[2], edges[0]],
                            [edges[1], edges[2]],
                            [edges[2], edges[1]],
                        ]
                    })
                    .collect::<Vec<_>>();

                for direction in [PrincipalDirection::X, PrincipalDirection::Y, PrincipalDirection::Z] {
                    let edges = nodes
                        .clone()
                        .into_par_iter()
                        .flat_map(|node| {
                            mesh_resmut.mesh.neighbor_function_edgepairgraph()(node)
                                .into_iter()
                                .map(|neighbor| {
                                    assert!(mesh_resmut.mesh.twin(node[1]) == neighbor[0]);
                                    let middle_edge = node[1];
                                    let (m1, m2) = mesh_resmut.mesh.endpoints(middle_edge);

                                    let start_edge = node[0];
                                    let end_edge = neighbor[1];
                                    // Vector from middle_edge to start_edge
                                    let vector_a = mesh_resmut.mesh.midpoint(start_edge) - mesh_resmut.mesh.midpoint(middle_edge);
                                    // Vector from middle_edge to end_edge
                                    let vector_b = mesh_resmut.mesh.midpoint(end_edge) - mesh_resmut.mesh.midpoint(middle_edge);

                                    // Vector from middle_edge to m1
                                    let vector_m1 = mesh_resmut.mesh.position(m1) - mesh_resmut.mesh.midpoint(middle_edge);
                                    // Vector from middle_edge to m2
                                    let vector_m2 = mesh_resmut.mesh.position(m2) - mesh_resmut.mesh.midpoint(middle_edge);

                                    // Angle around m1
                                    let angle_around_m1 = mesh_resmut.mesh.vec_angle(vector_a, vector_m1) + mesh_resmut.mesh.vec_angle(vector_b, vector_m1);

                                    // Angle around m2
                                    let angle_around_m2 = mesh_resmut.mesh.vec_angle(vector_a, vector_m2) + mesh_resmut.mesh.vec_angle(vector_b, vector_m2);

                                    // Whichever angle is shorter is the "real" angle
                                    let angle = f64::min(angle_around_m1, angle_around_m2);

                                    if !(0. ..=PI).contains(&angle) {
                                        warn!("{angle} is degenerate!!!");
                                    }

                                    // assert!((0. ..=PI).contains(&angle));

                                    // Weight is based on how far the angle is from 180 degrees
                                    let angular_weight = PI - angle;

                                    // Alignment per edge
                                    // Vector_a
                                    // Find the face that is bounded by the two edges
                                    let face_a = mesh_resmut.mesh.face(start_edge);
                                    let alignment_vector_a = (-vector_a).cross(&mesh_resmut.mesh.normal(face_a)).angle(&direction.into());

                                    // Vector_b
                                    // Find the face that is bounded by the two edges
                                    let face_b = mesh_resmut.mesh.face(end_edge);
                                    let alignment_vector_b = vector_b.cross(&mesh_resmut.mesh.normal(face_b)).angle(&direction.into());

                                    let weight = (angular_weight, (alignment_vector_a + alignment_vector_b) / 2., 0.);

                                    (node, neighbor, weight)
                                })
                                .collect_vec()
                        })
                        .collect::<Vec<_>>();

                    mesh_resmut.flow_graphs[direction as usize] = Graaf::from(nodes.clone(), edges);
                }

                if solution.current_solution.loops.is_empty() {
                    // solution.current_solution.initialize(&mesh_resmut.flow_graphs);
                }

                solution.current_solution.reconstruct_solution();
            }
            ActionEvent::ExportNLR => {
                let cur = format!("{}", env::current_dir().unwrap().clone().display());

                let t = SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_secs();
                let n = mesh_resmut
                    .properties
                    .source
                    .split("\\")
                    .last()
                    .unwrap()
                    .split('.')
                    .next()
                    .unwrap()
                    .split(' ')
                    .next()
                    .unwrap();

                let path_topol = format!("{cur}/out/{t}.topol");
                let path_geom = format!("{cur}/out/{t}.geom",);
                let path_cdim = format!("{cur}/out/{t}.cdim",);

                solution
                    .current_solution
                    .export_to_nlr(&PathBuf::from(path_topol), &PathBuf::from(path_geom), &PathBuf::from(path_cdim));
            }
            ActionEvent::ExportAll => {
                if mesh_resmut.mesh.vert_ids().is_empty() {
                    return;
                }

                let t = SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_secs();
                let n = mesh_resmut
                    .properties
                    .source
                    .split("\\")
                    .last()
                    .unwrap()
                    .split('.')
                    .next()
                    .unwrap()
                    .split(' ')
                    .next()
                    .unwrap();

                // TODO: fix name

                let path_save = format!("./out/{t}.save");
                let path_obj = format!("./out/{t}.obj",);
                let path_flag = format!("./out/{t}.flag",);

                let state = SaveStateObject {
                    mesh: (*mesh_resmut.mesh).clone(),
                    loops: solution.current_solution.loops.values().cloned().collect(),
                    configuration: configuration.clone(),
                };

                fs::write(&PathBuf::from(path_save), serde_json::to_string(&state).unwrap());

                solution.current_solution.export(&PathBuf::from(path_obj), &PathBuf::from(path_flag));
            }

            ActionEvent::ExportState => {
                if mesh_resmut.mesh.vert_ids().is_empty() {
                    return;
                }

                let cur = format!("{}", env::current_dir().unwrap().clone().display());

                let t = SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_secs();
                let n = mesh_resmut
                    .properties
                    .source
                    .split("\\")
                    .last()
                    .unwrap()
                    .split('.')
                    .next()
                    .unwrap()
                    .split(' ')
                    .next()
                    .unwrap();

                let path_save = format!("{cur}/out/{t}.save",);

                let state = SaveStateObject {
                    mesh: (*mesh_resmut.mesh).clone(),
                    loops: solution.current_solution.loops.values().cloned().collect(),
                    configuration: configuration.clone(),
                };

                info!("writing to {cur}/out/{n}_{t}.save");
                info!("writing to {cur}/out/{n}_{t}.save");
                info!("writing to {cur}/out/{n}_{t}.save");
                info!("writing to {cur}/out/{n}_{t}.save");

                let res = fs::write(PathBuf::from(path_save), serde_json::to_string(&state).unwrap());
                info!("{:?}", res);
            }

            ActionEvent::ExportSolution => {
                if mesh_resmut.mesh.vert_ids().is_empty() {
                    return;
                }

                let t = SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_secs();
                let n = mesh_resmut
                    .properties
                    .source
                    .split("\\")
                    .last()
                    .unwrap()
                    .split('.')
                    .next()
                    .unwrap()
                    .split(' ')
                    .next()
                    .unwrap();

                let path_obj = format!("./out/{n}_{t:?}.obj");
                let path_flag = format!("./out/{n}_{t:?}.flag");

                solution.current_solution.export(&PathBuf::from(path_obj), &PathBuf::from(path_flag));
            }
            ActionEvent::ToHexmesh => {
                if mesh_resmut.mesh.vert_ids().is_empty() {
                    return;
                }

                let t = SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_secs();
                let n = mesh_resmut.properties.source.split("\\").last().unwrap().split('.').next().unwrap().to_owned();

                let path_save = format!("./out/{n}_{t}.save");
                let path_obj = format!("./out/{n}_{t}.obj",);
                let path_flag = format!("./out/{n}_{t}.flag",);

                let state = SaveStateObject {
                    mesh: (*mesh_resmut.mesh).clone(),
                    loops: solution.current_solution.loops.values().cloned().collect(),
                    configuration: configuration.clone(),
                };

                fs::write(&PathBuf::from(path_save), serde_json::to_string(&state).unwrap());

                solution.current_solution.export(&PathBuf::from(path_obj), &PathBuf::from(path_flag));

                configuration.hex_mesh_status = HexMeshStatus::Loading;

                let task_pool = AsyncComputeTaskPool::get();
                let task = task_pool.spawn(async move {
                    Command::new("wsl")
                        .args([
                            "~/polycube-to-hexmesh/pipeline.sh",
                            &format!("/mnt/c/Users/20182085/Documents/douconel-test-env/out/{n}_{t}.obj"),
                            "-flag",
                            &format!("/mnt/c/Users/20182085/Documents/douconel-test-env/out/{n}_{t}.flag"),
                        ])
                        .output()
                        .expect("failed to execute process");

                    // outputs to '~/polycube-to-hexmesh/out/{n}_{t}_hex.mesh'

                    let out = Command::new("wsl")
                        .args([
                            "python3",
                            "~/polycube-to-hexmesh/evaluator.py",
                            &format!("~/polycube-to-hexmesh/tmp/{n}_{t}_remesh.mesh"),
                            &format!("~/polycube-to-hexmesh/out/{n}_{t}_hex.mesh"),
                        ])
                        .output()
                        .expect("failed to execute process");

                    let out_str = String::from_utf8(out.stdout).unwrap();
                    let out_vec = out_str.split('\n').filter(|s| !s.is_empty()).collect_vec();

                    let score = HexMeshScore {
                        hausdorff: out_vec[0].parse().unwrap(),
                        avg_jacob: out_vec[1].parse().unwrap(),
                        min_jacob: out_vec[2].parse().unwrap(),
                        max_jacob: out_vec[3].parse().unwrap(),
                        irregular: out_vec[4].parse().unwrap(),
                    };

                    Some(score)
                });

                hextasks.generating_chunks.insert(999, task);
            }
            ActionEvent::ResetCamera => {
                render::reset(&mut commands, &cameras, &mut images, &mut camera_handles);
            }
            ActionEvent::Mutate => {
                let task_pool = AsyncComputeTaskPool::get();

                let cloned_solution = solution.current_solution.clone();
                let cloned_flow_graphs = mesh_resmut.flow_graphs.clone();

                if (cloned_solution.loops.len() < 10) {
                    let task = task_pool.spawn(async move {
                        {
                            cloned_solution.mutate_add_loop(10, &cloned_flow_graphs)
                        }
                    });
                    tasks.generating_chunks.insert(0, task);
                } else {
                    if rand::random() {
                        let task = task_pool.spawn(async move {
                            {
                                cloned_solution.mutate_add_loop(10, &cloned_flow_graphs)
                            }
                        });
                        tasks.generating_chunks.insert(0, task);
                    } else {
                        let task = task_pool.spawn(async move {
                            {
                                cloned_solution.mutate_del_loop(10)
                            }
                        });
                        tasks.generating_chunks.insert(1, task);
                    }
                }
            }
        }
    }
}

#[inline]
fn vec3_to_vector3d(v: Vec3) -> Vector3D {
    Vector3D::new(v.x.into(), v.y.into(), v.z.into())
}

fn raycast(
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mut mouse: ResMut<ButtonInput<MouseButton>>,
    mut evr_motion: EventReader<MouseMotion>,
    mut evr_scroll: EventReader<MouseWheel>,
    mut keyboard: ResMut<ButtonInput<KeyCode>>,
    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    mut configuration: ResMut<Configuration>,
    query: Query<Entity, With<MainMesh>>,
) {
    configuration.raycasted = None;
    configuration.selected = None;
    gizmos_cache.raycaster.clear();

    if configuration.ui_is_hovered.iter().any(|&x| x) {
        return;
    }

    if !configuration.interactive || cursor_ray.is_none() {
        return;
    }

    // Safe unwrap because `cursor_ray` is not none
    let intersections = raycast.cast_ray(cursor_ray.unwrap(), &default());

    if intersections.is_empty() {
        return;
    }

    // Safe index because we know there is at least one intersection because `intersections` is not empty
    let intersected_entity = intersections[0].0;
    let intersection = &intersections[0].1;

    let main_mesh_entity = query.single();
    if intersected_entity != main_mesh_entity {
        return;
    }

    // For drawing purposes.
    let normal = vec3_to_vector3d(intersection.normal().normalize());
    let position = hutspot::draw::invert_transform_coordinates(
        vec3_to_vector3d(intersection.position()),
        mesh_resmut.properties.translation,
        mesh_resmut.properties.scale,
    );
    add_line(
        &mut gizmos_cache.raycaster,
        position,
        normal,
        0.1,
        to_color(configuration.direction, Perspective::Dual, None),
        Vec3::new(
            mesh_resmut.properties.translation.x as f32,
            mesh_resmut.properties.translation.y as f32,
            mesh_resmut.properties.translation.z as f32,
        ),
        mesh_resmut.properties.scale,
    );

    for (&edgepair, sol) in &solution.next[configuration.direction as usize] {
        let u = mesh_resmut.mesh.midpoint(edgepair[0]);
        let v = mesh_resmut.mesh.midpoint(edgepair[1]);
        let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edgepair[0]));
        let color = match sol {
            Some(_) => to_color(configuration.direction, Perspective::Dual, None),
            None => hutspot::color::BLACK.into(),
        };
        add_line2(
            &mut gizmos_cache.raycaster,
            u,
            v,
            n * 0.01,
            color,
            Vec3::new(
                mesh_resmut.properties.translation.x as f32,
                mesh_resmut.properties.translation.y as f32,
                mesh_resmut.properties.translation.z as f32,
            ),
            mesh_resmut.properties.scale,
        );
    }

    if keyboard.pressed(KeyCode::ControlLeft) {
        return;
    }

    for ev in evr_motion.read() {
        if (ev.delta.x.abs() + ev.delta.y.abs()) > 2. {
            return;
        }
    }

    for ev in evr_scroll.read() {
        if (ev.x.abs() + ev.y.abs()) > 0. {
            return;
        }
    }

    // Match the selected verts of the selected triangle (face of three vertices).
    if intersection.triangle().is_none() {
        return;
    }
    let verts = [
        vec3_to_vector3d(intersection.triangle().unwrap()[0].into()),
        vec3_to_vector3d(intersection.triangle().unwrap()[1].into()),
        vec3_to_vector3d(intersection.triangle().unwrap()[2].into()),
    ]
    .into_iter()
    .map(|p| hutspot::draw::invert_transform_coordinates(p, mesh_resmut.properties.translation, mesh_resmut.properties.scale))
    .map(|p| mesh_resmut.vertex_lookup.nearest(&p.into()).1)
    .sorted_by(|&a, &b| {
        mesh_resmut
            .mesh
            .position(a)
            .metric_distance(&position)
            .partial_cmp(&mesh_resmut.mesh.position(b).metric_distance(&position))
            .unwrap()
    })
    .collect_vec();

    // Match the selected face.
    if mesh_resmut.mesh.face_with_verts(&verts).is_none() {
        return;
    }
    let face_id = mesh_resmut.mesh.face_with_verts(&verts).unwrap();
    let edgepair = mesh_resmut.mesh.edges_in_face_with_vert(face_id, verts[0]).unwrap();
    configuration.raycasted = Some(edgepair);

    if keyboard.pressed(KeyCode::Space) {
        // Find closest loop.

        let option_a = [edgepair[0], edgepair[1]];
        let option_b = [edgepair[1], edgepair[0]];

        if let Some(loop_id) = solution.current_solution.loops.keys().find(|&loop_id| {
            let edges = solution.current_solution.get_pairs_of_loop(loop_id);
            edges.contains(&option_a) || edges.contains(&option_b)
        }) {
            if mouse.just_pressed(MouseButton::Left) || mouse.just_released(MouseButton::Left) {
                mouse.clear_just_pressed(MouseButton::Left);
                mouse.clear_just_released(MouseButton::Left);

                let mut new_sol = solution.current_solution.clone();
                new_sol.del_loop(loop_id);
                if new_sol.reconstruct_solution().is_ok() {
                    solution.current_solution = new_sol;
                    solution.next[0].clear();
                    solution.next[1].clear();
                    solution.next[2].clear();
                    cache.cache[0].clear();
                    cache.cache[1].clear();
                    cache.cache[2].clear();
                }
                return;
            }
        }
        return;
    }

    // If shift button is pressed, we want to show the closest solution to the current face vertex combination.
    if keyboard.pressed(KeyCode::ShiftLeft) {
        keyboard.clear_just_pressed(KeyCode::ShiftLeft);

        // Find the closest face vertex combination to the current face vertex combination.
        // Map to distance. Then get the solution with the smallest distance.
        let closest_solution = solution.next[configuration.direction as usize]
            .iter()
            .map(|(&edgepair, sol)| {
                (
                    ((mesh_resmut.mesh.midpoint(edgepair[0]) + mesh_resmut.mesh.midpoint(edgepair[1])) / 2.).metric_distance(&position),
                    sol,
                    edgepair,
                )
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        if let Some((_, closest_solution, signature)) = closest_solution {
            configuration.selected = Some(signature);

            if let Some(some_solution) = closest_solution {
                for loop_id in some_solution.loops.keys() {
                    let direction = some_solution.loop_to_direction(loop_id);
                    let color = to_color(direction, Perspective::Dual, None);
                    for &edgepair in &some_solution.get_pairs_of_loop(loop_id) {
                        let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                        let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                        let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                        add_line2(
                            &mut gizmos_cache.raycaster,
                            u,
                            v,
                            n * 0.05,
                            color,
                            Vec3::new(
                                mesh_resmut.properties.translation.x as f32,
                                mesh_resmut.properties.translation.y as f32,
                                mesh_resmut.properties.translation.z as f32,
                            ),
                            mesh_resmut.properties.scale,
                        );
                    }
                }

                // If the right mouse button is pressed, we want to save the candidate solution as the current solution.
                if mouse.just_pressed(MouseButton::Left) || mouse.just_released(MouseButton::Left) {
                    mouse.clear_just_pressed(MouseButton::Left);
                    mouse.clear_just_released(MouseButton::Left);

                    solution.current_solution = some_solution.clone();
                    solution.next[0].clear();
                    solution.next[1].clear();
                    solution.next[2].clear();
                    cache.cache[0].clear();
                    cache.cache[1].clear();
                    cache.cache[2].clear();
                    return;
                }
            }
        }

        return;
    }

    if !mouse.pressed(MouseButton::Left) {
        return;
    }

    if let Ok(dual) = &solution.current_solution.dual {
        if mouse.just_pressed(MouseButton::Left) || mouse.just_released(MouseButton::Left) {
            let selected_edges = mesh_resmut.mesh.edges_in_face_with_vert(face_id, verts[0]).unwrap();
            let alpha = rand::random::<f64>();
            let measure = |(a, b, c): (f64, f64, f64)| alpha * a.powi(10) + (1. - alpha) * b.powi(10);

            if let Some(candidate_loop) = solution.current_solution.construct_unbounded_loop(
                selected_edges,
                configuration.direction,
                &mesh_resmut.flow_graphs[configuration.direction as usize],
                measure,
            ) {
                let mut candidate_solution = solution.current_solution.clone();
                candidate_solution.add_loop(Loop {
                    edges: candidate_loop,
                    direction: configuration.direction,
                });
                if candidate_solution.reconstruct_solution().is_ok() {
                    solution.next[configuration.direction as usize].insert([selected_edges[0], selected_edges[1]], Some(candidate_solution));
                }
            }
        }
    } else {
        // The left mouse button is pressed, and no solution has been computed yet.
        // We will compute a solution for this face and vertex combination.
        let mut timer = hutspot::timer::Timer::new();
        solution.next[configuration.direction as usize].insert(edgepair, None);

        // Compute the occupied edges (edges that are already part of the solution, they are covered by loops)
        let occupied = solution.current_solution.occupied_edgepairs();
        timer.report("Computed `occupied`, hashmap of occupied edges");
        timer.reset();

        let filter = |(a, b): (&[EdgeID; 2], &[EdgeID; 2])| {
            !occupied.contains(a)
                && !occupied.contains(b)
                && [a[0], a[1], b[0], b[1]].iter().all(|&edge| {
                    solution
                        .current_solution
                        .loops_on_edge(edge)
                        .iter()
                        .filter(|&&loop_id| solution.current_solution.loop_to_direction(loop_id) == configuration.direction)
                        .count()
                        == 0
                })
        };

        let g_original = &mesh_resmut.flow_graphs[configuration.direction as usize];
        let g = g_original.filter(filter);

        let measure = |(a, b, c): (f64, f64, f64)| configuration.alpha * a.powi(10) + (1. - configuration.alpha) * b.powi(10);

        // Starting edges.
        let [e1, e2] = mesh_resmut.mesh.edges_in_face_with_vert(face_id, verts[0]).unwrap();
        let a = g.node_to_index(&[e1, e2]).unwrap();
        let b = g.node_to_index(&[e2, e1]).unwrap();
        let option_a = g
            .shortest_cycle(a, &measure)
            .unwrap_or_default()
            .into_iter()
            .flat_map(|node_index| g.index_to_node(node_index).unwrap().to_owned())
            .collect_vec();
        let option_b = g
            .shortest_cycle(b, &measure)
            .unwrap_or_default()
            .into_iter()
            .flat_map(|node_index| g.index_to_node(node_index).unwrap().to_owned())
            .collect_vec();

        // The path may contain self intersections. We can remove these.
        // If duplicated vertices are present, remove everything between them.
        let mut cleaned_option_a = vec![];
        for edge_id in option_a {
            if cleaned_option_a.contains(&edge_id) {
                cleaned_option_a = cleaned_option_a.into_iter().take_while(|&x| x != edge_id).collect_vec();
            }
            cleaned_option_a.push(edge_id);
        }

        let mut cleaned_option_b = vec![];
        for edge_id in option_b {
            if cleaned_option_b.contains(&edge_id) {
                cleaned_option_b = cleaned_option_b.into_iter().take_while(|&x| x != edge_id).collect_vec();
            }
            cleaned_option_b.push(edge_id);
        }

        if cleaned_option_a.len() <= 5 && cleaned_option_b.len() <= 5 {
            println!("Path is empty and/or invalid.");
            return;
        }

        if cleaned_option_a.len() <= 5 {
            cleaned_option_a = cleaned_option_b.clone();
        }

        let measure2 = |(a, b, c): (f64, f64, f64)| b.powi(10);

        println!("Option A: {:?}", cleaned_option_a);
        println!("Option B: {:?}", cleaned_option_b);

        // TODO: check better whether solution exists.
        if cleaned_option_a.is_empty() || cleaned_option_b.is_empty() {
            return;
        }

        let weight_option_a: f64 = cleaned_option_a
            .chunks(2)
            .map(|window| [window[0], window[1]])
            .collect_vec()
            .windows(2)
            .map(|pairs| {
                let pair1 = pairs[0];
                let pair2 = pairs[1];

                let pair1_node = g.node_to_index(&pair1).unwrap();
                let pair2_node = g.node_to_index(&pair2).unwrap();

                let weight = g.get_weight(pair1_node, pair2_node);
                measure2(weight)
            })
            .sum();

        let weight_option_b: f64 = cleaned_option_b
            .chunks(2)
            .map(|window| [window[0], window[1]])
            .collect_vec()
            .windows(2)
            .map(|pairs| {
                let pair1 = pairs[0];
                let pair2 = pairs[1];

                let pair1_node = g.node_to_index(&pair1).unwrap();
                let pair2_node = g.node_to_index(&pair2).unwrap();

                let weight = g.get_weight(pair1_node, pair2_node);
                measure2(weight)
            })
            .sum();

        let best_option = if weight_option_a > weight_option_b {
            cleaned_option_b
        } else {
            cleaned_option_a
        };

        timer.report("Path computation");
        timer.reset();

        let mut real_solution = solution.current_solution.clone();

        real_solution.add_loop(Loop {
            edges: best_option,
            direction: configuration.direction,
        });
        real_solution.reconstruct_solution();

        solution.next[configuration.direction as usize].insert(edgepair, Some(real_solution));
    }
}
