mod draw;
mod dual;
mod elements;
mod system;
mod ui;

use crate::ui::ui;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::{Douconel, EdgeID, Empty, FaceID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use dual::{Dual, Polycube, PropertyViolationError};
use elements::Loop;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ordered_float::OrderedFloat;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use smooth_bevy_cameras::controllers::orbit::OrbitCameraPlugin;
use smooth_bevy_cameras::LookTransformPlugin;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::BufReader;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use system::{draw_gizmos, fps, set_camera_viewports, setup, sync_cameras, update_mesh, Configuration};

const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::rgb(27. / 255., 27. / 255., 27. / 255.);

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SaveStateObject {
    mesh: InputResource,
    solution: SolutionResource,
    configuration: Configuration,
}

#[derive(Component)]
pub struct RenderedMesh;

#[derive(Event, Debug)]
pub enum ActionEvent {
    LoadFile(PathBuf),
    ExportState,
}

#[derive(Default, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum LoopScoring {
    #[default]
    PathLength,
    LoopDistribution,
    SingularitySeparationCount,
    SingularitySeparationSpread,
}

#[derive(Default, Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DrawLoopType {
    None,
    #[default]
    Undirected,
    Directed,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct MeshProperties {
    pub source: String,
    pub nr_of_faces: usize,
    pub nr_of_edges: usize,
    pub nr_of_vertices: usize,
    pub scale: f32,
    pub translation: Vector3D,
}

// implement default for KdTree using the New Type Idiom
#[derive(Debug, Clone, Serialize, Deserialize)]
struct TreeD(KdTree<f64, usize, [f64; 3]>);
impl TreeD {
    fn nearest(&self, point: &[f64; 3]) -> (f64, usize) {
        let neighbors = self.0.nearest(point, 1, &squared_euclidean).unwrap();
        let (d, i) = neighbors.first().unwrap();
        (*d, **i)
    }

    fn add(&mut self, point: [f64; 3], index: usize) {
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

#[derive(Copy, Clone)]
enum GizmoType {
    Wireframe,
    Vertex,
    Normal,
}

type Line = (Vec3, Vec3, Color, GizmoType);

#[derive(Default, Resource)]
pub struct GizmosCache {
    lines: Vec<Line>,
    raycast: Vec<(Vec3, Vec3, Color)>,
    debug: Vec<(Vec3, Vec3, Color)>,
}

type EmbeddedMesh = Douconel<EmbeddedVertex, Empty, Empty>;

#[derive(Default, Debug, Clone, Resource, Serialize, Deserialize)]
pub struct InputResource {
    mesh: Arc<EmbeddedMesh>,
    properties: MeshProperties,
    vertex_lookup: TreeD,
}

#[derive(Debug, Clone, Resource, Serialize, Deserialize)]
pub struct SolutionResource {
    dual: Dual,
    primal: Result<Polycube, PropertyViolationError>,
    next: HashMap<(FaceID, VertID), Option<(Dual, Result<Polycube, PropertyViolationError>)>>,
    properties: MeshProperties,
}

impl Default for SolutionResource {
    fn default() -> Self {
        Self {
            dual: Dual::default(),
            primal: Err(PropertyViolationError::UnknownError),
            next: HashMap::new(),
            properties: MeshProperties::default(),
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
        .insert_resource(ClearColor(BACKGROUND_COLOR))
        .insert_resource(AmbientLight { brightness: 1.0, ..default() })
        // Load default plugins
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "bevy-test-env".to_string(),
                mode: WindowMode::BorderlessFullscreen,
                ..Default::default()
            }),
            ..Default::default()
        }))
        // Plugin for raycasting
        .add_plugins(DefaultRaycastingPlugin)
        // Plugin for FPS
        .add_plugins(FrameTimeDiagnosticsPlugin)
        // Plugin for GUI
        .add_plugins(EguiPlugin)
        // Plugin for smooth camera
        .add_plugins(LookTransformPlugin)
        .add_plugins(OrbitCameraPlugin::default())
        // User specified
        .add_systems(Startup, setup)
        .add_systems(Update, sync_cameras)
        .add_systems(Update, ui)
        .add_systems(Update, handle_events)
        .add_systems(Update, draw_gizmos)
        .add_systems(Update, update_mesh)
        .add_systems(Update, raycast)
        .add_systems(Update, set_camera_viewports)
        .add_systems(Update, fps)
        .add_event::<ActionEvent>()
        .run();
}

pub fn handle_events(
    mut ev_reader: EventReader<ActionEvent>,
    mut mesh_resmut: ResMut<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    mut configuration: ResMut<Configuration>,
) {
    for ev in ev_reader.read() {
        info!("Received event {ev:?}. Handling...");
        match ev {
            ActionEvent::LoadFile(path) => {
                match path.extension().unwrap().to_str() {
                    Some("obj" | "stl") => {
                        *configuration = Configuration::default();
                        *mesh_resmut = InputResource::default();
                        *solution = SolutionResource::default();

                        mesh_resmut.mesh = match Douconel::from_file(path) {
                            Ok(res) => Arc::new(res.0),
                            Err(err) => {
                                panic!("Error while parsing STL file {path:?}: {err:?}");
                            }
                        };

                        let mut patterns = TreeD::default();
                        for (i, v_id) in mesh_resmut.mesh.vert_ids().into_iter().enumerate() {
                            patterns.add(mesh_resmut.mesh.position(v_id).into(), i);
                        }
                        mesh_resmut.vertex_lookup = patterns;

                        solution.dual = Dual::new(mesh_resmut.mesh.clone());
                    }
                    Some("save") => {
                        let loaded_state: SaveStateObject = serde_json::from_reader(BufReader::new(File::open(path).unwrap())).unwrap();
                        *mesh_resmut = loaded_state.mesh;
                        *solution = loaded_state.solution;
                        *configuration = loaded_state.configuration;
                    }
                    _ => panic!("File format not supported."),
                }

                mesh_resmut.properties.source = String::from(path.to_str().unwrap());

                mesh_resmut.properties.nr_of_vertices = mesh_resmut.mesh.nr_verts();
                mesh_resmut.properties.nr_of_edges = mesh_resmut.mesh.nr_edges() / 2; // dcel -> single edge
                mesh_resmut.properties.nr_of_faces = mesh_resmut.mesh.nr_faces();

                let mesh = mesh_resmut.mesh.bevy(&HashMap::new());
                let aabb = mesh.compute_aabb().unwrap();
                mesh_resmut.properties.scale = 10. * (1. / aabb.half_extents.max_element());
                let translation = -mesh_resmut.properties.scale * aabb.center;
                mesh_resmut.properties.translation = Vector3D::new(translation.x.into(), translation.y.into(), translation.z.into());

                for edge_id in mesh_resmut.mesh.edge_ids() {
                    draw::add_edge(
                        &mut gizmos_cache.lines,
                        edge_id,
                        &mesh_resmut.mesh,
                        mesh_resmut.properties.translation,
                        mesh_resmut.properties.scale,
                    );
                }

                for vert_id in mesh_resmut.mesh.vert_ids() {
                    draw::add_vertex(
                        &mut gizmos_cache.lines,
                        vert_id,
                        &mesh_resmut.mesh,
                        mesh_resmut.properties.translation,
                        mesh_resmut.properties.scale,
                    );
                }

                for face_id in mesh_resmut.mesh.face_ids() {
                    draw::add_face_normal(
                        &mut gizmos_cache.lines,
                        face_id,
                        &mesh_resmut.mesh,
                        mesh_resmut.properties.translation,
                        mesh_resmut.properties.scale,
                    );
                }
            }
            ActionEvent::ExportState => {
                let path = format!(
                    "./out/{}_{:?}.save",
                    mesh_resmut.properties.source.split("\\").last().unwrap().split(".").next().unwrap(),
                    SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_millis()
                );

                let state = SaveStateObject {
                    mesh: mesh_resmut.as_mut().clone(),
                    solution: solution.as_mut().clone(),
                    configuration: configuration.as_mut().clone(),
                };

                fs::write(&PathBuf::from(path), serde_json::to_string(&state).unwrap());
            }
        }
    }
}

fn raycast(
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mut mouse: ResMut<Input<MouseButton>>,
    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    mut configuration: ResMut<Configuration>,
) {
    configuration.cur_selected = None;

    if !configuration.interactive {
        gizmos_cache.raycast.clear();
        return;
    }

    if cursor_ray.is_none() {
        return;
    }

    let intersections = raycast.cast_ray(cursor_ray.unwrap(), &default());
    if intersections.is_empty() {
        return;
    }

    let intersection = &intersections[0].1;
    let normal = Vector3D::new(intersection.normal().x.into(), intersection.normal().y.into(), intersection.normal().z.into());
    let position_in_render = Vector3D::new(
        intersection.position().x.into(),
        intersection.position().y.into(),
        intersection.position().z.into(),
    );

    let position = hutspot::draw::invert_transform_coordinates(position_in_render, mesh_resmut.properties.translation, mesh_resmut.properties.scale);
    let line = DrawableLine::from_vertex(position, normal, 5., mesh_resmut.properties.translation, mesh_resmut.properties.scale);
    gizmos_cache.raycast.clear();
    gizmos_cache.raycast.push((line.u, line.v, hutspot::color::ROODT.into()));

    let triangle_ids = if let Some(triangle) = intersection.triangle() {
        [
            Vector3D::new(triangle.v0.x.into(), triangle.v0.y.into(), triangle.v0.z.into()),
            Vector3D::new(triangle.v1.x.into(), triangle.v1.y.into(), triangle.v1.z.into()),
            Vector3D::new(triangle.v2.x.into(), triangle.v2.y.into(), triangle.v2.z.into()),
        ]
        .into_iter()
        .map(|pos| {
            mesh_resmut.mesh.vert_ids()[mesh_resmut
                .vertex_lookup
                .nearest(&hutspot::draw::invert_transform_coordinates(pos, mesh_resmut.properties.translation, mesh_resmut.properties.scale).into())
                .1]
        })
        // .map(|v| (v, v.metric_distance(&position_in_render)))
        // .sorted_by(|(a, a_dist), (b, b_dist)| a_dist.partial_cmp(b_dist).unwrap())
        .collect_vec()
    } else {
        return;
    };

    // if configuration.region_selection {
    //     let face = solution
    //         .dual
    //         .get_loop_structure()
    //         .faces
    //         .iter()
    //         .find(|(_, face)| face.verts.contains(&triangle_ids[0]))
    //         .unwrap()
    //         .1;

    //     for &vert_id in &face.verts {
    //         let p = mesh_resmut.mesh.position(vert_id);
    //         let n = mesh_resmut.mesh.vert_normal(vert_id);
    //         let line = DrawableLine::from_vertex(p, n, 0.05, configuration.translation, configuration.scale);

    //         gizmos_cache.raycast.push((line.u, line.v, face.color));
    //     }
    // }

    if !configuration.interactive {
        return;
    }

    // Select the nearest vert (sort by distance to intersection point)
    let vert_id = triangle_ids
        .iter()
        .map(|&id| (id, mesh_resmut.mesh.position(id)))
        .min_by(|&(_, a_pos), &(_, b_pos)| a_pos.metric_distance(&position).partial_cmp(&b_pos.metric_distance(&position)).unwrap())
        .unwrap()
        .0;
    let face_id = mesh_resmut.mesh.face_with_verts(&triangle_ids).unwrap();

    configuration.cur_selected = Some((face_id, vert_id));

    if let Some(None) = solution.next.get(&(face_id, vert_id)) {
        return;
    }

    if let Some(Some((sol, poly))) = solution.next.get(&(face_id, vert_id)).cloned() {
        for &loop_id in &sol.get_loop_ids() {
            for &edge in &sol.get_pairs_of_loop(loop_id) {
                let u = mesh_resmut.mesh.midpoint(edge[0]);
                let v = mesh_resmut.mesh.midpoint(edge[1]);
                let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0]));
                for line in DrawableLine::from_arrow(
                    u,
                    v,
                    n,
                    0.9,
                    mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0])) * 0.001,
                    mesh_resmut.properties.translation,
                    mesh_resmut.properties.scale,
                ) {
                    gizmos_cache.raycast.push((line.u, line.v, sol.loop_to_direction(loop_id).to_dual_color()));
                }
            }
        }

        if mouse.just_pressed(MouseButton::Right) || mouse.just_released(MouseButton::Right) {
            mouse.clear_just_pressed(MouseButton::Right);
            mouse.clear_just_released(MouseButton::Right);

            solution.dual = sol;
            solution.primal = poly;

            solution.next.clear();
            cache.cache[0].clear();
            cache.cache[1].clear();
            cache.cache[2].clear();
        }

        return;
    }

    if !mouse.pressed(MouseButton::Left) {
        return;
    }

    solution.next.insert((face_id, vert_id), None);

    let mut timer = hutspot::timer::Timer::new();
    println!("Computing path...");

    let mut occupied = HashMap::new();

    for (edge1, loop1) in &solution.dual.occupied {
        for edge2 in mesh_resmut.mesh.edges(mesh_resmut.mesh.face(edge1)) {
            if edge1 == edge2 {
                continue;
            }
            if loop1.iter().any(|l| solution.dual.loops_on_edge(edge2).contains(l)) {
                occupied.insert([edge1, edge2], true);
                occupied.insert([edge2, edge1], true);
            }
        }
    }
    timer.report("Occupied edges");
    timer.reset();

    let edge_to_neighbors = mesh_resmut
        .mesh
        .edge_ids()
        .par_iter()
        .map(|&edge| {
            if solution
                .dual
                .loops_on_edge(edge)
                .iter()
                .filter(|&&loop_id| solution.dual.loop_to_direction(loop_id) == configuration.direction)
                .count()
                > 0
            {
                (edge, vec![])
            } else {
                let neighbors = mesh_resmut.mesh.neighbor_function_edgepairgraph()([edge, edge]);
                (edge, neighbors)
            }
        })
        .collect::<HashMap<_, _>>();
    timer.report("Edge to neighbors");
    timer.reset();

    // The neighborhood function: filters out all edges that are already used in the solution
    let nfunction = |edgepair: [EdgeID; 2]| {
        if occupied.contains_key(&edgepair) {
            vec![]
        } else {
            edge_to_neighbors[&edgepair[1]].clone()
        }
    };

    // The weight function
    let wfunction = mesh_resmut
        .mesh
        .weight_function_angle_edgepairs_aligned(configuration.alpha, configuration.beta, configuration.direction.into());

    let cache_ref = &mut cache.cache[configuration.direction as usize];
    let [e1, e2] = mesh_resmut.mesh.edges_in_face_with_vert(face_id, vert_id).unwrap();

    if occupied.contains_key(&[e1, e2]) || occupied.contains_key(&[e2, e1]) {
        println!("Path is not possible.");
        return;
    }

    let total_path = [
        hutspot::graph::find_shortest_cycle([e1, e2], nfunction, &wfunction, cache_ref),
        hutspot::graph::find_shortest_cycle([e2, e1], nfunction, &wfunction, cache_ref),
    ]
    .into_par_iter()
    .flatten()
    .collect::<Vec<_>>()
    .into_iter()
    .sorted_by(|&(_, a), &(_, b)| a.cmp(&b))
    .next()
    .unwrap_or_default()
    .0
    .into_iter()
    .flatten()
    .collect_vec();

    if total_path.len() < 2 {
        println!("Path is empty.");
        return;
    }
    timer.report("Path computation");
    timer.reset();

    // Clean path, if duplicated vertices are present, remove everything between them.
    let mut cur_path = vec![];
    for v in total_path {
        if cur_path.contains(&v) {
            cur_path = cur_path.into_iter().take_while(|&x| x != v).collect_vec();
            cur_path.push(v);
        } else {
            cur_path.push(v);
        }
    }
    let total_path = cur_path;
    timer.report("Path stripping");
    timer.reset();

    println!("Adding path...");
    let mut dual_copy = solution.dual.clone();
    dual_copy.add_loop(Loop {
        edges: total_path,
        direction: configuration.direction,
    });
    timer.report("Adding path");
    timer.reset();

    let polycube = dual_copy.build_loop_structure();

    solution.next.insert((face_id, vert_id), Some((dual_copy, polycube)));
}
