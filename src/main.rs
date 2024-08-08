mod camera;
mod dual;
mod elements;
mod render;
mod system;
mod ui;

use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::{Douconel, EdgeID, Empty, FaceID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use dual::{Dual, Polycube, PropertyViolationError};
use elements::Loop;
use graph::prelude::{CsrLayout, DirectedCsrGraph, GraphBuilder};
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ordered_float::OrderedFloat;
use petgraph::visit::{IntoNodeIdentifiers, IntoNodeReferences};
use petgraph::Directed;
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
use system::{fps, Configuration};

pub const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::rgb(248. / 255., 248. / 255., 248. / 255.);
pub const MESH_OFFSET: Vector3D = Vector3D::new(0., 1_000., 0.);
pub const POLYCUBE_OFFSET: Vector3D = Vector3D::new(0., -1_000., 0.);

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
}

#[derive(Debug, Clone, Resource, Serialize, Deserialize)]
pub struct SolutionResource {
    dual: Dual,
    primal: Result<Polycube, PropertyViolationError>,
    next: [HashMap<(FaceID, VertID), Option<(Dual, Result<Polycube, PropertyViolationError>)>>; 3],
    properties: MeshProperties,
}

impl Default for SolutionResource {
    fn default() -> Self {
        Self {
            dual: Dual::default(),
            primal: Err(PropertyViolationError::UnknownError),
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
                        for v_id in mesh_resmut.mesh.vert_ids() {
                            patterns.add(mesh_resmut.mesh.position(v_id).into(), v_id);
                        }
                        mesh_resmut.vertex_lookup = patterns;

                        solution.dual = Dual::new(mesh_resmut.mesh.clone());
                    }
                    Some("save") => {
                        let loaded_state: SaveStateObject = serde_json::from_reader(BufReader::new(File::open(path).unwrap())).unwrap();
                        *mesh_resmut = loaded_state.mesh;
                        *solution = loaded_state.solution;
                        *configuration = loaded_state.configuration;
                        mesh_resmut.as_mut();
                    }
                    _ => panic!("File format not supported."),
                }

                mesh_resmut.properties.source = String::from(path.to_str().unwrap());

                configuration.interactive = true;
                configuration.alpha = 15;
                configuration.beta = 15;
                configuration.black = true;
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
            if let Some((sol, poly)) = valid_solution.clone() {
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

    // TODO: probably use different approach for this.
    // Map from edges to neighbors.
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
        if occupied.contains(&edgepair) {
            vec![]
        } else {
            edge_to_neighbors[&edgepair[1]].clone()
        }
    };

    // The weight function
    let wfunction = mesh_resmut
        .mesh
        .weight_function_angle_edgepairs_aligned(configuration.alpha, configuration.beta, configuration.direction.into());

    // Cache for partial solutions.
    let cache_ref = &mut cache.cache[configuration.direction as usize];

    // Starting edges.
    let [e1, e2] = mesh_resmut.mesh.edges_in_face_with_vert(face_id, verts[0]).unwrap();

    // TODO: why is this unwrap safe?
    let option_a = hutspot::graph::find_shortest_cycle([e1, e2], nfunction, &wfunction).unwrap_or_else(|| (vec![], OrderedFloat(0.)));
    let option_b = hutspot::graph::find_shortest_cycle([e1, e2], nfunction, &wfunction).unwrap_or_else(|| (vec![], OrderedFloat(0.)));
    let best_option = if option_a.1 > option_b.1 {
        option_a.0.into_iter().flatten().collect_vec()
    } else {
        option_b.0.into_iter().flatten().collect_vec()
    };
    timer.report("CURRENT Path computation");
    timer.reset();

    // BENCHMARK WITH OTHER GRAPH CRATES:

    let mut nodes = vec![];
    let mut nodes_to_id = HashMap::new();
    for face_id in mesh_resmut.mesh.face_ids() {
        // Add a node for each edge combination.
        let edges = mesh_resmut.mesh.edges(face_id);
        assert!(edges.len() == 3);
        let combinations = vec![
            [edges[0], edges[1]],
            [edges[1], edges[0]],
            [edges[0], edges[2]],
            [edges[2], edges[0]],
            [edges[1], edges[2]],
            [edges[2], edges[1]],
        ];
        for combination in combinations {
            if !nodes_to_id.contains_key(&combination) {
                nodes.push(combination);
                nodes_to_id.insert(combination, nodes.len() - 1);
            }
        }
    }

    timer.report("Found nodes");
    timer.reset();

    // let mut edges = vec![];
    // for (&node, &node_id) in &nodes_to_id {
    //     for neighbor in mesh_resmut.mesh.neighbor_function_edgepairgraph()(node) {
    //         let neighbor_id = nodes_to_id[&neighbor];
    //         edges.push((node_id, neighbor_id, wfunction(node, neighbor)));
    //     }
    // }

    let edges = nodes
        .into_par_iter()
        .flat_map(|node| {
            mesh_resmut.mesh.neighbor_function_edgepairgraph()(node)
                .into_iter()
                .map(|neighbor| (nodes_to_id[&node], nodes_to_id[&neighbor], wfunction(node, neighbor)))
                .collect_vec()
        })
        .collect::<Vec<_>>();

    timer.report("Found edges");
    timer.reset();

    let g: DirectedCsrGraph<usize, (), OrderedFloat<f64>> = GraphBuilder::new().csr_layout(CsrLayout::Sorted).edges_with_values(edges).build();

    timer.report("Built graph");
    timer.reset();

    // If the best option is empty, we have no valid path.
    if best_option.len() < 5 {
        println!("Path is empty and/or invalid.");
        return;
    }

    // The path may contain self intersections. We can remove these.
    // If duplicated vertices are present, remove everything between them.
    let mut cleaned_option = vec![];
    for edge_id in best_option {
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
    let polycube = new_sol.build_loop_structure();
    solution.next[configuration.direction as usize].insert((face_id, verts[0]), Some((new_sol, polycube)));
}
