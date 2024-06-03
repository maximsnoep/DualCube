#![warn(clippy::all, clippy::pedantic, clippy::nursery, clippy::cargo)]
#![allow(clippy::missing_panics_doc, clippy::missing_errors_doc)]
mod dual;
mod ui;

use crate::dual::PrincipalDirection;
use crate::ui::ui;
use bevy::diagnostic::LogDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::time::common_conditions::on_timer;
use bevy::utils::petgraph::visit::IntoNodeReferences;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::{find_shortest_cycle, Douconel, EdgeID, FaceID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use dual::Dual;
use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ordered_float::OrderedFloat;
use petgraph::graphmap::*;
use petgraph::visit::IntoEdgeReferences;
use potpoursi::math::{inv_transform_coordinates, transform_coordinates};
use serde::{Deserialize, Serialize};
use smooth_bevy_cameras::controllers::orbit::{
    OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin,
};
use smooth_bevy_cameras::LookTransformPlugin;
use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Duration;

const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::WHITE;

#[derive(Component)]
pub struct RenderedMesh;

#[derive(Event, Debug)]
pub enum ActionEvent {
    LoadFile(PathBuf),
    AddLoop(PrincipalDirection),
}

#[derive(Default, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum LoopScoring {
    #[default]
    PathLength,
    LoopDistribution,
    SingularitySeparationCount,
    SingularitySeparationSpread,
}

pub struct Rules {
    pub intersections: bool,
    pub loop_regions: bool,
}

#[derive(Default, Resource, Clone)]
pub struct Configuration {
    pub source: String,
    pub nr_of_faces: usize,
    pub nr_of_edges: usize,
    pub nr_of_vertices: usize,

    pub direction: PrincipalDirection,

    pub scale: f32,
    pub translation: Vec3,

    pub draw_wireframe: bool,
    pub draw_dualgraph: bool,
    pub draw_line_midpoint_graph: bool,
    pub draw_edgegraph: bool,
    pub draw_loopgraph: bool,

    pub draw_acyclic: bool,
    pub angle_filter: f32,

    pub draw_next: bool,

    pub loop_scoring: LoopScoring,

    pub draw_loops: bool,
    pub draw_loops_plus: bool,

    pub id_selector: usize,

    pub iterations: usize,
    pub samples: usize,
}

// implement default for KdTree using the New Type Idiom
struct TreeD(KdTree<f32, usize, [f32; 3]>);
impl TreeD {
    fn nearest(&self, point: &[f32; 3]) -> (f32, usize) {
        let neighbors = self.0.nearest(point, 1, &squared_euclidean).unwrap();
        let (d, i) = neighbors.first().unwrap();
        (*d, **i)
    }

    fn nearests(&self, point: &[f32; 3], n: usize) -> Vec<(f32, usize)> {
        let neighbors = self.0.nearest(point, n, &squared_euclidean).unwrap();
        neighbors.iter().map(|(d, i)| (*d, **i)).collect_vec()
    }

    fn add(&mut self, point: [f32; 3], index: usize) {
        self.0.add(point, index).unwrap();
    }
}
impl Default for TreeD {
    fn default() -> Self {
        TreeD(KdTree::new(3))
    }
}

#[derive(Default, Resource)]
pub struct CacheResource {
    cache: [HashMap<(EdgeID, EdgeID), Vec<((EdgeID, EdgeID), OrderedFloat<f32>)>>; 3],
}

#[derive(Default, Resource)]
pub struct MeshResource {
    mesh: Douconel<EmbeddedVertex, (), ()>,

    vertex_lookup: TreeD,
    keys: Vec<VertID>,

    graph: DiGraphMap<VertID, f32>,
    dual_graph: DiGraphMap<FaceID, f32>,
    edge_graph: DiGraphMap<EdgeID, f32>,
    midpoint_graph: DiGraphMap<EdgeID, ()>,
    line_midpoint_graph: DiGraphMap<(EdgeID, EdgeID), ()>,
    line_midpoint_graph_weighted: [DiGraphMap<(EdgeID, EdgeID), f32>; 3],

    anchors: Vec<EdgeID>,

    paths: Vec<Vec<EdgeID>>,

    dual: Dual,
}

#[derive(Default, Debug, Copy, Clone, Serialize, Deserialize)]
pub struct EdgeWithDirection {
    direction: Option<PrincipalDirection>,
}

pub trait HasDirection {
    fn direction(&self) -> Option<PrincipalDirection>;
    fn set_direction(&mut self, direction: Option<PrincipalDirection>);
}

impl HasDirection for EdgeWithDirection {
    fn direction(&self) -> Option<PrincipalDirection> {
        self.direction
    }
    fn set_direction(&mut self, direction: Option<PrincipalDirection>) {
        self.direction = direction;
    }
}

fn main() {
    App::new()
        .init_resource::<MeshResource>()
        .init_resource::<CacheResource>()
        .init_resource::<Configuration>()
        .insert_resource(ClearColor(BACKGROUND_COLOR))
        .insert_resource(AmbientLight {
            brightness: 1.0,
            ..default()
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
        .add_plugins(DefaultRaycastingPlugin)
        // Plugin for diagnostics
        .add_plugins(LogDiagnosticsPlugin::default())
        // Plugin for GUI
        .add_plugins(EguiPlugin)
        // Plugin for smooth camera
        .add_plugins(LookTransformPlugin)
        .add_plugins(OrbitCameraPlugin::default())
        // User specified
        .add_systems(Startup, setup)
        .add_systems(Update, ui)
        .add_systems(Update, handle_events)
        .add_systems(Update, draw_gizmos)
        .add_systems(
            Update,
            update_mesh.run_if(on_timer(Duration::from_millis(100))),
        )
        .add_systems(Update, raycast)
        .add_event::<ActionEvent>()
        .run();
}

fn raycast(
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mouse: Res<Input<MouseButton>>,
    mut gizmos: Gizmos,
    mut mesh_resmut: ResMut<MeshResource>,
    mut cache: ResMut<CacheResource>,
    configuration: ResMut<Configuration>,
) {
    if let Some(cursor_ray) = **cursor_ray {
        let intersections = raycast.cast_ray(cursor_ray, &default());
        if !intersections.is_empty() {
            let intersection = &raycast.cast_ray(cursor_ray, &default())[0].1;

            let normal = intersection.normal();
            let position = inv_transform_coordinates(
                configuration.translation,
                configuration.scale,
                intersection.position().into(),
            );

            draw_pointer(&mut gizmos, &configuration, (position, normal), Color::RED);

            let triangle = intersection.triangle().unwrap();
            let v0 = inv_transform_coordinates(
                configuration.translation,
                configuration.scale,
                triangle.v0.into(),
            );
            let v1 = inv_transform_coordinates(
                configuration.translation,
                configuration.scale,
                triangle.v1.into(),
            );
            let v2 = inv_transform_coordinates(
                configuration.translation,
                configuration.scale,
                triangle.v2.into(),
            );

            // get nearest point of v0, v1, and v2
            let dist_v0 = position.distance(v0);
            let dist_v1 = position.distance(v1);
            let dist_v2 = position.distance(v2);
            let (nearest, second_nearest, third_nearest) = if dist_v0 < dist_v1 {
                if dist_v1 < dist_v2 {
                    (v0, v1, v2)
                } else {
                    if dist_v0 < dist_v2 {
                        (v0, v2, v1)
                    } else {
                        (v2, v0, v1)
                    }
                }
            } else {
                if dist_v0 < dist_v2 {
                    (v1, v0, v2)
                } else {
                    if dist_v1 < dist_v2 {
                        (v1, v2, v0)
                    } else {
                        (v2, v1, v0)
                    }
                }
            };

            // v positions to mesh ids
            let nearest_id = mesh_resmut.keys[mesh_resmut.vertex_lookup.nearest(&nearest.into()).1];
            let nearest_face = mesh_resmut
                .mesh
                .star(nearest_id)
                .into_iter()
                .sorted_by(|&face_a, &face_b| {
                    mesh_resmut
                        .mesh
                        .centroid(face_a)
                        .distance(position)
                        .partial_cmp(&mesh_resmut.mesh.centroid(face_b).distance(position))
                        .unwrap()
                })
                .next()
                .unwrap();

            let mut two_edges = mesh_resmut
                .mesh
                .edges(nearest_face)
                .into_iter()
                .filter(|&edge| {
                    let (u, v) = mesh_resmut.mesh.endpoints(edge);
                    u == nearest_id || v == nearest_id
                });

            let nearest_edge = two_edges.next().unwrap();
            let second_nearest_edge = two_edges.next().unwrap();

            draw_vertex(
                &mut gizmos,
                &configuration,
                (nearest, mesh_resmut.mesh.vert_normal(nearest_id)),
                Color::GREEN,
            );

            for edge in mesh_resmut.mesh.edges(nearest_face) {
                let (u, v) = mesh_resmut.mesh.endpoints(edge);
                let color = match configuration.direction {
                    PrincipalDirection::X => potpoursi::color::YZ_PRIMARY,
                    PrincipalDirection::Y => potpoursi::color::XZ_PRIMARY,
                    PrincipalDirection::Z => potpoursi::color::XY_PRIMARY,
                };
                let u_pos = mesh_resmut.mesh.position(u);
                let v_pos = mesh_resmut.mesh.position(v);
                let u_nor = mesh_resmut.mesh.vert_normal(u);
                let v_nor = mesh_resmut.mesh.vert_normal(v);
                draw_arrow(
                    &mut gizmos,
                    &configuration,
                    (u_pos, u_nor),
                    (v_pos, v_nor),
                    color,
                    0.6,
                );
            }

            // draw line from nearest to second nearest (midpoints)
            let u = mesh_resmut.mesh.midpoint(nearest_edge);
            let v = mesh_resmut.mesh.midpoint(second_nearest_edge);
            let un = mesh_resmut.mesh.edge_normal(nearest_edge);
            let vn = mesh_resmut.mesh.edge_normal(second_nearest_edge);
            draw_arrow(
                &mut gizmos,
                &configuration,
                (u, un),
                (v, vn),
                Color::PINK,
                0.8,
            );

            // let cache = Rc::new(RefCell::new(SecondaryMap::<
            //     EdgeID,
            //     Vec<(EdgeID, OrderedFloat<f32>)>,
            // >::new()));

            // let total_path = mesh_resmut
            //     .mesh
            //     .find_shortest_cycle(
            //         nearest_edge,
            //         mesh_resmut.mesh.neighbor_function_edgegraph(),
            //         mesh_resmut.mesh.weight_function_angle_edges(2),
            //         cache,
            //     )
            //     .unwrap();

            // for edge in total_path.0 {
            //     let (u_id, v_id) = mesh_resmut.mesh.endpoints(edge);

            //     let color = match configuration.direction {
            //         PrincipalDirection::X => potpoursi::color::YZ_PRIMARY,
            //         PrincipalDirection::Y => potpoursi::color::XZ_PRIMARY,
            //         PrincipalDirection::Z => potpoursi::color::XY_PRIMARY,
            //     };

            //     let u = mesh_resmut.mesh.position(u_id);
            //     let v = mesh_resmut.mesh.position(v_id);
            //     let un = mesh_resmut.mesh.vert_normal(u_id);
            //     let vn = mesh_resmut.mesh.vert_normal(v_id);
            //     draw_arrow(&mut gizmos, &configuration, (u, un), (v, vn), color, 0.6);
            // }

            let total_path = [
                find_shortest_cycle(
                    (nearest_edge, second_nearest_edge),
                    mesh_resmut.mesh.neighbor_function_edgepairgraph(),
                    mesh_resmut.mesh.weight_function_angle_edgepairs_aligned(
                        5,
                        5,
                        configuration.direction.to_vector(),
                    ),
                    &mut cache.cache[configuration.direction as usize],
                ),
                find_shortest_cycle(
                    (nearest_edge, second_nearest_edge),
                    mesh_resmut.mesh.neighbor_function_edgepairgraph(),
                    mesh_resmut.mesh.weight_function_angle_edgepairs_aligned(
                        2,
                        8,
                        configuration.direction.to_vector(),
                    ),
                    &mut cache.cache[configuration.direction as usize],
                ),
            ]
            .into_iter()
            .flatten()
            .sorted_by(|&(_, a), &(_, b)| a.cmp(&b))
            .next()
            .unwrap_or_default()
            .0;

            for edge in total_path {
                let u_pos = mesh_resmut.mesh.midpoint(edge.0);
                let v_pos = mesh_resmut.mesh.midpoint(edge.1);
                let u_nor = mesh_resmut.mesh.edge_normal(edge.0);
                let v_nor = mesh_resmut.mesh.edge_normal(edge.1);

                let color = match configuration.direction {
                    PrincipalDirection::X => potpoursi::color::YZ_PRIMARY,
                    PrincipalDirection::Y => potpoursi::color::XZ_PRIMARY,
                    PrincipalDirection::Z => potpoursi::color::XY_PRIMARY,
                };

                draw_arrow(
                    &mut gizmos,
                    &configuration,
                    (u_pos, u_nor),
                    (v_pos, v_nor),
                    color,
                    0.6,
                );
            }
        }
    }
}

/// Set up
fn setup(mut commands: Commands, mut egui_ctx: bevy_egui::EguiContexts) {
    commands
        .spawn(Camera3dBundle::default())
        .insert(OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(0.0, 5.0, 20.0),
            Vec3::new(0., 0., 0.),
            Vec3::Y,
        ));

    let mut fonts = bevy_egui::egui::FontDefinitions::default();
    fonts.font_data.insert(
        "my_font".to_owned(),
        bevy_egui::egui::FontData::from_static(include_bytes!(
            "../assets/BerkeleyMonoTrial-Regular.ttf"
        )),
    );
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "my_font".to_owned());
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Monospace)
        .or_default()
        .push("my_font".to_owned());
    egui_ctx.ctx_mut().set_fonts(fonts);
    egui_ctx
        .ctx_mut()
        .set_visuals(bevy_egui::egui::Visuals::light());
}

// // Graph s.t. node for each original edge, and (directed) edges between nodes if the original edges are adjacent
// pub fn line_graph<E>(
//     g: &DiGraphMap<EdgeID, E>,
//     mesh: &Douconel<EmbeddedVertex, (), EmbeddedFace>,
// ) -> DiGraphMap<(EdgeID, EdgeID), ()>
// where
//     E: Clone + Default,
// {
//     let mut edges = vec![];
//     for (u0, u1, _) in g.all_edges() {
//         for (v0, v1, _) in g.all_edges() {
//             if u1 == v0 {
//                 // Add hyperedge if u0 != v1 AND face(u0) != face(v1)
//                 // or on any of their twins...
//                 if mesh.face(u0) == mesh.face(v1)
//                     || mesh.face(mesh.twin(u0)) == mesh.face(v1)
//                     || mesh.face(u0) == mesh.face(mesh.twin(v1))
//                 {
//                     continue;
//                 }

//                 edges.push(((u0, u1), (v0, v1), ()));
//             }
//         }
//     }
//     DiGraphMap::<(_, _), ()>::from_edges(edges)
// }

// pub fn weight_e(
//     mesh: &Douconel<EmbeddedVertex, (), EmbeddedFace>,
//     e: ((EdgeID, EdgeID), (EdgeID, EdgeID)),
//     d: Vec3,
// ) -> f32 {
//     let primal_edge0 = e.0 .0;
//     let primal_edge1 = e.0 .1;
//     let primal_edge2 = e.1 .0;
//     let primal_edge3 = e.1 .1;

//     assert!(primal_edge1 == primal_edge2);

//     // get midpoint of edges
//     let u0 = mesh.midpoint(primal_edge0);
//     let v0 = mesh.midpoint(primal_edge1);
//     let w0 = mesh.midpoint(primal_edge3);

//     let u0n = mesh.edge_normal(primal_edge0);
//     let v0n = mesh.edge_normal(primal_edge1);
//     let w0n = mesh.edge_normal(primal_edge3);

//     // edge0 = (u0, v0)
//     let edge0 = v0 - u0;
//     // edge1 = (v0, w0)
//     let edge1 = w0 - v0;

//     // angle between edge0 and edge1
//     let gamma = edge0.angle_between(edge1);

//     // length of edge0
//     let len0 = edge0.length();

//     // length of edge1
//     let len1 = edge1.length();

//     // |e|
//     let l_e = (gamma * f32::min(len0, len1)) / (2.0 * f32::tan(gamma / 2.0));

//     // k^2 (e)
//     let k2_e = gamma.powi(2) / f32::min(len0, len1);

//     // midpoint of edge0
//     let n0 = (u0 + v0) / 2.;
//     let n0n = (u0n + v0n) / 2.;

//     // midpoint of edge1
//     let n1 = (v0 + w0) / 2.;
//     let n1n = (v0n + w0n) / 2.;

//     let edge0_dir = edge0.normalize();
//     let edge1_dir = edge1.normalize();

//     let edge0_nor = n0n;
//     let edge1_nor = n1n;

//     let edge0_cross = edge0_dir.cross(edge0_nor);
//     let edge0_angle = (d.angle_between(edge0_cross) / std::f32::consts::PI) * 180.;

//     let edge1_cross = edge1_dir.cross(edge1_nor);
//     let edge1_angle = (d.angle_between(edge1_cross) / std::f32::consts::PI) * 180.;

//     // q(e)
//     let q_e = f32::max(edge0_angle, edge1_angle).powi(2);

//     // w(e)
//     let alpha = 10.;
//     let beta = 0.5;
//     let w_e = l_e + alpha * q_e + beta * k2_e;

//     return w_e;
// }

// // weight based on :: blabla
// pub fn line_graph_weighted(
//     mesh: &Douconel<EmbeddedVertex, (), EmbeddedFace>,
//     g: &DiGraphMap<(EdgeID, EdgeID), ()>,
//     d: PrincipalDirection,
// ) -> DiGraphMap<(EdgeID, EdgeID), f32> {
//     let mut edges = vec![];
//     // go through current edges, compute weight, and add it to `edges`
//     for (edge0, edge1, _) in g.all_edges() {
//         let w = weight_e(mesh, (edge0, edge1), d.to_vector());
//         if w.is_nan() {
//             continue;
//         }
//         edges.push((edge0, edge1, w));
//     }
//     DiGraphMap::<(EdgeID, EdgeID), f32>::from_edges(edges)
// }

pub fn handle_events(
    mut ev_reader: EventReader<ActionEvent>,
    mut mesh_resmut: ResMut<MeshResource>,
    mut configuration: ResMut<Configuration>,
) {
    for ev in ev_reader.read() {
        info!("Received event {ev:?}. Handling...");
        match ev {
            ActionEvent::LoadFile(path) => {
                match path.extension().unwrap().to_str() {
                    Some("stl") => match Douconel::from_stl(path.to_str().unwrap()) {
                        Ok(res) => {
                            *configuration = Configuration::default();
                            configuration.source = String::from(path.to_str().unwrap());

                            mesh_resmut.mesh = res.0;
                            configuration.nr_of_vertices = mesh_resmut.mesh.nr_verts();
                            configuration.nr_of_edges = mesh_resmut.mesh.nr_edges() / 2; // dcel -> single edge
                            configuration.nr_of_faces = mesh_resmut.mesh.nr_faces();

                            let mut patterns = TreeD::default();
                            let keys = mesh_resmut.mesh.verts.keys().collect_vec();

                            for (i, &v_id) in keys.iter().enumerate() {
                                let p = mesh_resmut.mesh.position(v_id);

                                if patterns.0.size() > 0 {
                                    // lookup the position, to see if it already exists
                                    let d = patterns.nearest(&p.into()).0;
                                    if d < 0.00001 {
                                        println!("duplicate vertex found at {:?}", p);
                                        continue;
                                    }
                                }

                                println!("inserting vertex at {:?}", p);

                                patterns.add(p.into(), i);
                            }

                            mesh_resmut.vertex_lookup = patterns;

                            mesh_resmut.keys = keys;

                            mesh_resmut.graph = mesh_resmut.mesh.graph_euclidean();
                            mesh_resmut.dual_graph = mesh_resmut.mesh.dual_graph_euclidean();

                            let edges = mesh_resmut.mesh.edges.keys().into_iter().collect_vec();

                            for edge_id in edges {
                                mesh_resmut.dual.edge_to_paths.insert(edge_id, Vec::new());
                            }

                            let color_map = HashMap::new();

                            let mesh = mesh_resmut.mesh.bevy(&color_map);

                            let aabb = mesh.compute_aabb().unwrap();
                            configuration.scale = 10. * (1. / aabb.half_extents.max_element());
                            configuration.translation = (-configuration.scale * aabb.center).into();

                            configuration.loop_scoring = LoopScoring::PathLength;
                            configuration.samples = 1000;

                            // mesh_resmut
                            //     .cached_paths
                            //     .insert(PrincipalDirection::X, HashMap::new());
                            // mesh_resmut
                            //     .cached_paths
                            //     .insert(PrincipalDirection::Y, HashMap::new());
                            // mesh_resmut
                            //     .cached_paths
                            //     .insert(PrincipalDirection::Z, HashMap::new());
                        }
                        Err(err) => {
                            error!("Error while parsing STL file {path:?}: {err:?}");
                        }
                    },
                    _ => panic!("File format not supported."),
                }
            }
            ActionEvent::AddLoop(dir) => {}
        }
    }
}

// This function should be called when the mesh (RenderedMesh) is changed, to make sure that modifications are visualized.
fn update_mesh(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    mesh_resmut: Res<MeshResource>,
    configuration: Res<Configuration>,

    rendered_mesh_query: Query<Entity, With<RenderedMesh>>,
) {
    if !mesh_resmut.is_changed() {
        return;
    }
    info!("Mesh has been changed. Updating the Bevy render.");

    for entity in rendered_mesh_query.iter() {
        commands.entity(entity).despawn();
    }
    info!("Despawning any current meshes.");

    if mesh_resmut.mesh.faces.is_empty() {
        warn!("Mesh is empty (?)");
        return;
    }

    let color_map = HashMap::new();
    let mesh = mesh_resmut.mesh.bevy(&color_map);

    // Spawn new mesh
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(mesh),
            transform: Transform {
                translation: configuration.translation,
                rotation: Quat::from_rotation_z(0f32),
                scale: Vec3::splat(configuration.scale),
            },
            material: materials.add(StandardMaterial {
                perceptual_roughness: 0.9,
                ..default()
            }),
            ..default()
        },
        RenderedMesh,
    ));
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    mesh_resmut: Res<MeshResource>,
    configuration: Res<Configuration>,
) {
    let secondary_color = match configuration.direction {
        PrincipalDirection::X => potpoursi::color::YZ_PRIMARY,
        PrincipalDirection::Y => potpoursi::color::XZ_PRIMARY,
        PrincipalDirection::Z => potpoursi::color::XY_PRIMARY,
    };
    let secondary_color_light = match configuration.direction {
        PrincipalDirection::X => potpoursi::color::YZ_SECONDARY,
        PrincipalDirection::Y => potpoursi::color::XZ_SECONDARY,
        PrincipalDirection::Z => potpoursi::color::XY_SECONDARY,
    };

    // if true {
    //     // draw anchors
    //     for &(nearest_edge, second_nearest_edge) in &mesh_resmut.anchors {
    //         // draw line from nearest to second nearest (midpoints)
    //         let u = mesh_resmut.mesh.midpoint(nearest_edge);
    //         let v = mesh_resmut.mesh.midpoint(second_nearest_edge);
    //         let un = mesh_resmut.mesh.edge_normal(nearest_edge);
    //         let vn = mesh_resmut.mesh.edge_normal(second_nearest_edge);
    //         draw_arrow(
    //             &mut gizmos,
    //             &configuration,
    //             (u, un),
    //             (v, vn),
    //             Color::PINK,
    //             0.8,
    //         );
    //     }
    // }

    if configuration.draw_wireframe {
        let g: &DiGraphMap<VertID, f32> = &mesh_resmut.graph;

        for (u_id, v_id, _) in g.edge_references() {
            let u = mesh_resmut.mesh.position(u_id);
            let v = mesh_resmut.mesh.position(v_id);
            let un = mesh_resmut.mesh.vert_normal(u_id);
            let vn = mesh_resmut.mesh.vert_normal(v_id);
            draw_line(&mut gizmos, &configuration, (u, un), (v, vn), Color::GRAY);
        }

        for (v_id, _) in g.node_references() {
            let p = mesh_resmut.mesh.position(v_id);
            let n = mesh_resmut.mesh.vert_normal(v_id);
            draw_vertex(&mut gizmos, &configuration, (p, n), Color::GRAY);
        }
    }
}

pub fn draw_vertex(
    gizmos: &mut Gizmos,
    configuration: &Configuration,
    (p, n): (Vec3, Vec3),
    color: Color,
) {
    gizmos.line(
        transform_coordinates(configuration.translation, configuration.scale, p),
        transform_coordinates(configuration.translation, configuration.scale, p + n * 0.01),
        color,
    );
}

pub fn draw_pointer(
    gizmos: &mut Gizmos,
    configuration: &Configuration,
    (p, n): (Vec3, Vec3),
    color: Color,
) {
    gizmos.line(
        transform_coordinates(configuration.translation, configuration.scale, p),
        transform_coordinates(configuration.translation, configuration.scale, p + n * 0.1),
        color,
    );
}

pub fn draw_pointer_notransform(
    gizmos: &mut Gizmos,
    configuration: &Configuration,
    (p, n): (Vec3, Vec3),
    color: Color,
) {
    gizmos.line(p, p + n * 0.1, color);
}

pub fn draw_line(
    gizmos: &mut Gizmos,
    configuration: &Configuration,
    (u, un): (Vec3, Vec3),
    (v, vn): (Vec3, Vec3),
    color: Color,
) {
    gizmos.line(
        transform_coordinates(
            configuration.translation + un * 0.005,
            configuration.scale,
            u,
        ),
        transform_coordinates(
            configuration.translation + vn * 0.005,
            configuration.scale,
            v,
        ),
        color,
    );
}

pub fn draw_line_notransform(
    gizmos: &mut Gizmos,
    (u, un): (Vec3, Vec3),
    (v, vn): (Vec3, Vec3),
    color: Color,
) {
    gizmos.line(
        transform_coordinates(un * 0.01, 1., u),
        transform_coordinates(vn * 0.01, 1., v),
        color,
    );
}

pub fn draw_arrow(
    gizmos: &mut Gizmos,
    configuration: &Configuration,
    (u, un): (Vec3, Vec3),
    (v, vn): (Vec3, Vec3),
    color: Color,
    offset: f32,
) {
    let arrow = (v - u);
    let rev_arrow = (u - v);

    let quat = Quat::from_axis_angle(vn, ((5. + offset) / 6.) * std::f32::consts::PI);
    let quat2 = Quat::from_axis_angle(vn, ((7. - offset) / 6.) * std::f32::consts::PI);

    let arrow_head = quat.mul_vec3(arrow);
    let arrow_head2 = quat2.mul_vec3(arrow);

    let offset = 0.9;
    let arrow_head_offset = 0.1;

    gizmos.line(
        transform_coordinates(
            configuration.translation + un * 0.02,
            configuration.scale,
            v + rev_arrow * offset,
        ),
        transform_coordinates(
            configuration.translation + vn * 0.02,
            configuration.scale,
            u + arrow * offset,
        ),
        color,
    );

    gizmos.line(
        transform_coordinates(
            configuration.translation + vn * 0.02,
            configuration.scale,
            u + arrow * offset,
        ),
        transform_coordinates(
            configuration.translation + vn * 0.02,
            configuration.scale,
            u + arrow * offset + arrow_head * arrow_head_offset,
        ),
        color,
    );

    gizmos.line(
        transform_coordinates(
            configuration.translation + vn * 0.02,
            configuration.scale,
            u + arrow * offset,
        ),
        transform_coordinates(
            configuration.translation + vn * 0.02,
            configuration.scale,
            u + arrow * offset + arrow_head2 * arrow_head_offset,
        ),
        color,
    );
}
