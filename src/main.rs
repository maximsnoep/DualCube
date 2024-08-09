mod camera;
mod dual;
mod graph;
mod layout;
mod polycube;
mod render;
mod ui;

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::{Douconel, EdgeID, Empty, FaceID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use dual::{Dual, Loop, PrincipalDirection, PropertyViolationError};
use graph::Graaf;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use layout::Layout;
use ordered_float::OrderedFloat;
use polycube::Polycube;
use rayon::prelude::*;
use render::{add_line, add_line2, GizmosCache};
use serde::{Deserialize, Serialize};
use smooth_bevy_cameras::controllers::orbit::OrbitCameraPlugin;
use smooth_bevy_cameras::LookTransformPlugin;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::BufReader;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

pub const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::rgb(248. / 255., 248. / 255., 248. / 255.);
pub const MESH_OFFSET: Vector3D = Vector3D::new(0., 1_000., 0.);
pub const POLYCUBE_OFFSET: Vector3D = Vector3D::new(0., -1_000., 0.);

#[derive(Resource, Default, Debug, Clone, Serialize, Deserialize)]
pub struct Configuration {
    pub direction: PrincipalDirection,
    pub alpha: i32,
    pub beta: i32,

    pub sides_mask: [u32; 3],

    pub fps: f64,
    pub selected_face: Option<(FaceID, VertID)>,
    pub selected_solution: Option<(FaceID, VertID)>,

    pub compute_primal: bool,
    pub delete_mode: bool,

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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SaveStateObject {
    mesh: EmbeddedMesh,
    loops: Vec<Loop>,
}

#[derive(Component)]
pub struct RenderedMesh;

#[derive(Event, Debug)]
pub enum ActionEvent {
    LoadFile(PathBuf),
    ExportState,
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

type EmbeddedMesh = Douconel<EmbeddedVertex, Empty, Empty>;

#[derive(Default, Debug, Clone, Resource, Serialize, Deserialize)]
pub struct InputResource {
    mesh: Arc<EmbeddedMesh>,
    properties: MeshProperties,
    vertex_lookup: TreeD,
    flow_graphs: [Graaf<[EdgeID; 2], (f64, f64, f64)>; 3],
}

#[derive(Debug, Clone, Resource)]
pub struct SolutionResource {
    dual: Dual,
    primal: Result<Polycube, PropertyViolationError>,
    layout: Result<Layout, PropertyViolationError>,
    next: [HashMap<(FaceID, VertID), Option<(Dual, Result<Polycube, PropertyViolationError>, Option<Result<Layout, PropertyViolationError>>)>>; 3],
    properties: MeshProperties,
}

impl Default for SolutionResource {
    fn default() -> Self {
        Self {
            dual: Dual::default(),
            primal: Err(PropertyViolationError::UnknownError),
            layout: Err(PropertyViolationError::UnknownError),
            next: [HashMap::new(), HashMap::new(), HashMap::new()],
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
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0,
        })
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
        // Setups
        .add_systems(Startup, ui::setup)
        .add_systems(Startup, camera::setup)
        // Updates
        .add_systems(Update, ui::update)
        .add_systems(Update, camera::update)
        .add_systems(Update, render::update)
        .add_systems(Update, render::gizmos)
        .add_systems(Update, handle_events)
        .add_systems(Update, raycast)
        .add_systems(Update, fps)
        .add_event::<ActionEvent>()
        .run();
}

pub fn handle_events(
    mut ev_reader: EventReader<ActionEvent>,
    mut mesh_resmut: ResMut<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut configuration: ResMut<Configuration>,
) {
    for ev in ev_reader.read() {
        info!("Received event {ev:?}. Handling...");
        match ev {
            ActionEvent::LoadFile(path) => {
                *configuration = Configuration::default();
                *mesh_resmut = InputResource::default();
                *solution = SolutionResource::default();

                match path.extension().unwrap().to_str() {
                    Some("obj" | "stl") => {
                        mesh_resmut.mesh = match Douconel::from_file(path) {
                            Ok(res) => Arc::new(res.0),
                            Err(err) => {
                                panic!("Error while parsing STL file {path:?}: {err:?}");
                            }
                        };
                        solution.dual = Dual::new(mesh_resmut.mesh.clone());
                    }
                    Some("save") => {
                        let loaded_state: SaveStateObject = serde_json::from_reader(BufReader::new(File::open(path).unwrap())).unwrap();

                        mesh_resmut.mesh = Arc::new(loaded_state.mesh);

                        solution.dual = Dual::new(mesh_resmut.mesh.clone());

                        for saved_loop in loaded_state.loops {
                            solution.dual.add_loop(saved_loop);
                        }
                    }
                    _ => panic!("File format not supported."),
                }

                configuration.interactive = true;
                configuration.alpha = 15;
                configuration.beta = 15;
                configuration.black = true;

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
                                    let vector_a = mesh_resmut.mesh.midpoint(node[1]) - mesh_resmut.mesh.midpoint(node[0]);
                                    let vector_b = mesh_resmut.mesh.midpoint(neighbor[1]) - mesh_resmut.mesh.midpoint(neighbor[0]);
                                    let weight = (
                                        mesh_resmut.mesh.vec_angle(vector_a, vector_b),
                                        mesh_resmut
                                            .mesh
                                            .vec_angle(vector_a.cross(&mesh_resmut.mesh.edge_normal(node[0])), direction.into()),
                                        mesh_resmut
                                            .mesh
                                            .vec_angle(vector_b.cross(&mesh_resmut.mesh.edge_normal(neighbor[0])), direction.into()),
                                    );
                                    (node, neighbor, weight)
                                })
                                .collect_vec()
                        })
                        .collect::<Vec<_>>();

                    mesh_resmut.flow_graphs[direction as usize] = Graaf::from(nodes.clone(), edges);
                }

                let polycube = solution.dual.dual_phase();

                if let Ok(mut polycube) = polycube {
                    if let Err(err) = solution.dual.primal_phase(&mut polycube) {
                        solution.primal = Err(err);
                    } else {
                        solution.primal = Ok(polycube);
                    }
                }
            }
            ActionEvent::ExportState => {
                let path = format!(
                    "./out/{}_{:?}.save",
                    mesh_resmut.properties.source.split("\\").last().unwrap().split(".").next().unwrap(),
                    SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_millis()
                );

                let state = SaveStateObject {
                    mesh: (*mesh_resmut.mesh).clone(),
                    loops: solution.dual.get_loops(),
                };

                fs::write(&PathBuf::from(path), serde_json::to_string(&state).unwrap());
            }
        }
    }
}

#[inline]
fn vec3_to_vector3d(v: Vec3) -> Vector3D {
    Vector3D::new(v.x.into(), v.y.into(), v.z.into())
}

fn raycast(
    time: Res<Time>,
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mut mouse: ResMut<Input<MouseButton>>,
    mut keyboard: ResMut<Input<KeyCode>>,
    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    mut configuration: ResMut<Configuration>,
) {
    configuration.selected_solution = None;
    configuration.selected_face = None;
    gizmos_cache.raycaster.clear();

    if !configuration.interactive || cursor_ray.is_none() {
        return;
    }

    // Safe unwrap because `cursor_ray` is not none
    let intersections = raycast.cast_ray(cursor_ray.unwrap(), &default());
    if intersections.is_empty() {
        return;
    }

    // Safe index because we know there is at least one intersection because `intersections` is not empty
    let intersection = &intersections[0].1;

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
        configuration.direction.to_dual_color(),
        &mesh_resmut.properties,
    );

    for (&(face_id, vert_id), sol) in &solution.next[configuration.direction as usize] {
        let u = mesh_resmut.mesh.position(vert_id);
        let v = mesh_resmut.mesh.centroid(face_id);
        let n = mesh_resmut.mesh.normal(face_id);
        let color = match sol {
            Some(_) => configuration.direction.to_dual_color(),
            None => hutspot::color::BLACK.into(),
        };
        add_line2(&mut gizmos_cache.raycaster, u, v, n * 0.01, color, &mesh_resmut.properties);
    }

    // Match the selected verts of the selected triangle (face of three vertices).
    if intersection.triangle().is_none() {
        return;
    }
    let verts = [
        vec3_to_vector3d(intersection.triangle().unwrap().v0.into()),
        vec3_to_vector3d(intersection.triangle().unwrap().v1.into()),
        vec3_to_vector3d(intersection.triangle().unwrap().v2.into()),
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
    configuration.selected_face = Some((face_id, verts[0]));

    if configuration.delete_mode {
        // Find highlighted loop.
        let edgepair = mesh_resmut.mesh.edges_in_face_with_vert(face_id, verts[0]).unwrap();
        let option_a = [edgepair[0], edgepair[1]];
        let option_b = [edgepair[1], edgepair[0]];

        let highlighted_loop = solution.dual.get_loop_ids().into_iter().find(|&loop_id| {
            let edges = solution.dual.get_pairs_of_loop(loop_id);
            edges.contains(&option_a) || edges.contains(&option_b)
        });

        if let Some(loop_id) = highlighted_loop {
            let mut new_sol = solution.dual.clone();
            new_sol.del_loop(loop_id);

            let polycube = new_sol.dual_phase();

            if configuration.compute_primal {
                if let Ok(mut polycube) = polycube {
                    let layout = new_sol.primal_phase(&mut polycube);
                    solution.next[configuration.direction as usize].insert((face_id, verts[0]), Some((new_sol, Ok(polycube), Some(layout))));
                }
            } else {
                solution.next[configuration.direction as usize].insert((face_id, verts[0]), Some((new_sol, polycube, None)));
            }
        }
    }

    // If shift button is pressed, we want to show the closest solution to the current face vertex combination.
    if keyboard.pressed(KeyCode::ShiftLeft) {
        keyboard.clear_just_pressed(KeyCode::ShiftLeft);

        // Find the closest face vertex combination to the current face vertex combination.
        // Map to distance. Then get the solution with the smallest distance.
        let closest_solution = solution.next[configuration.direction as usize]
            .iter()
            .map(|(&(face_id, vert_id), sol)| (mesh_resmut.mesh.position(vert_id).metric_distance(&position), sol, (face_id, vert_id)))
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        if let Some((_, valid_solution, signature)) = closest_solution {
            configuration.selected_solution = Some(signature);
            if let Some((sol, poly, Some(layout))) = valid_solution.clone() {
                if time.elapsed().as_millis() % 1000 < 800 {
                    for &loop_id in &sol.get_loop_ids() {
                        let color = sol.loop_to_direction(loop_id).to_dual_color();
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
                                gizmos_cache.raycaster.push((line.u, line.v, color));
                            }
                        }
                    }
                }

                // If the right mouse button is pressed, we want to save the candidate solution as the current solution.
                if mouse.just_pressed(MouseButton::Right) || mouse.just_released(MouseButton::Right) {
                    mouse.clear_just_pressed(MouseButton::Right);
                    mouse.clear_just_released(MouseButton::Right);
                    solution.dual = sol;
                    solution.primal = poly;
                    solution.layout = layout;
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
    }

    if !mouse.pressed(MouseButton::Left) {
        return;
    }

    // The left mouse button is pressed, and no solution has been computed yet.
    // We will compute a solution for this face and vertex combination.
    let mut timer = hutspot::timer::Timer::new();
    solution.next[configuration.direction as usize].insert((face_id, verts[0]), None);

    // Compute the occupied edges (edges that are already part of the solution, they are covered by loops)
    let occupied = solution.dual.occupied_edgepairs();
    timer.report("Computed `occupied`, hashmap of occupied edges");
    timer.reset();

    let filter = |(a, b): (&[EdgeID; 2], &[EdgeID; 2])| {
        !occupied.contains(a)
            && !occupied.contains(b)
            && [a[0], a[1], b[0], b[1]].iter().all(|&edge| {
                solution
                    .dual
                    .loops_on_edge(edge)
                    .iter()
                    .filter(|&&loop_id| solution.dual.loop_to_direction(loop_id) == configuration.direction)
                    .count()
                    == 0
            })
    };

    let g_original = &mesh_resmut.flow_graphs[configuration.direction as usize];
    let g = g_original.filter(filter);

    let measure = |(a, b, c): (f64, f64, f64)| a.powi(configuration.alpha) + b.powi(configuration.beta) + c.powi(configuration.beta);

    // Starting edges.
    let [e1, e2] = mesh_resmut.mesh.edges_in_face_with_vert(face_id, verts[0]).unwrap();
    let a = g.node_to_index(&[e1, e2]).unwrap();
    let b = g.node_to_index(&[e2, e1]).unwrap();
    let option_a = g.shortest_cycle(a, &measure).unwrap_or_default();
    let option_b = g.shortest_cycle(b, &measure).unwrap_or_default();
    let best_option = if option_a.len() > option_b.len() { option_a } else { option_b };

    timer.report("Path computation");
    timer.reset();

    // Map back to EdgeID
    let best_path = best_option
        .into_iter()
        .flat_map(|node_index| g.index_to_node(node_index).unwrap().to_owned())
        .collect_vec();

    timer.report("Select best Path computation");
    timer.reset();

    // If the best option is empty, we have no valid path.
    if best_path.len() < 5 {
        println!("Path is empty and/or invalid.");
        return;
    }

    // The path may contain self intersections. We can remove these.
    // If duplicated vertices are present, remove everything between them.
    let mut cleaned_option = vec![];
    for edge_id in best_path {
        if cleaned_option.contains(&edge_id) {
            cleaned_option = cleaned_option.into_iter().take_while(|&x| x != edge_id).collect_vec();
        }
        cleaned_option.push(edge_id);
    }
    timer.report("Cleaned path");
    timer.reset();

    // Create a new solution with the new loop added.
    let mut new_sol = solution.dual.clone();
    new_sol.add_loop(Loop {
        edges: cleaned_option,
        direction: configuration.direction,
    });
    let polycube = new_sol.dual_phase();

    if configuration.compute_primal {
        if let Ok(mut polycube) = polycube {
            let layout = new_sol.primal_phase(&mut polycube);
            solution.next[configuration.direction as usize].insert((face_id, verts[0]), Some((new_sol, Ok(polycube), Some(layout))));
        }
    } else {
        solution.next[configuration.direction as usize].insert((face_id, verts[0]), Some((new_sol, polycube, None)));
    }
}
