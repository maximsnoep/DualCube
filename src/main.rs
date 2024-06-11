#![warn(clippy::all, clippy::pedantic, clippy::nursery, clippy::cargo)]
#![allow(clippy::missing_panics_doc, clippy::missing_errors_doc)]
mod dual;
mod ui;

use crate::dual::PrincipalDirection;
use crate::ui::ui;
use bevy::diagnostic::LogDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::time::common_conditions::on_timer;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::{Douconel, EdgeID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use dual::{Dual, Path};
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ordered_float::OrderedFloat;
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
    pub translation: Vector3D,

    pub draw_wireframe: bool,
    pub draw_vertices: bool,
    pub draw_normals: bool,
    pub draw_debug: bool,

    pub angle_filter: f32,
    pub draw_next: bool,
    pub loop_scoring: LoopScoring,
    pub draw_loops: bool,
    pub id_selector: usize,

    pub iterations: usize,
    pub samples: usize,
}

// implement default for KdTree using the New Type Idiom
struct TreeD(KdTree<f64, usize, [f64; 3]>);
impl TreeD {
    fn nearest(&self, point: &[f64; 3]) -> (f64, usize) {
        let neighbors = self.0.nearest(point, 1, &squared_euclidean).unwrap();
        let (d, i) = neighbors.first().unwrap();
        (*d, **i)
    }

    fn nearests(&self, point: &[f64; 3], n: usize) -> Vec<(f64, usize)> {
        let neighbors = self.0.nearest(point, n, &squared_euclidean).unwrap();
        neighbors.iter().map(|(d, i)| (*d, **i)).collect_vec()
    }

    fn add(&mut self, point: [f64; 3], index: usize) {
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
    cache: [HashMap<(EdgeID, EdgeID), Vec<((EdgeID, EdgeID), OrderedFloat<f64>)>>; 3],
}

#[derive(Copy, Clone)]
enum GizmoType {
    Wireframe,
    Vertex,
    Normal,
}

#[derive(Default, Resource)]
pub struct GizmosCache {
    lines: Vec<(Vec3, Vec3, Color, GizmoType)>,
    raycast: Vec<(Vec3, Vec3, Color)>,
    debug: Vec<(Vec3, Vec3, Color)>,
}

#[derive(Default, Resource)]
pub struct MeshResource {
    mesh: Douconel<EmbeddedVertex, (), ()>,

    vertex_lookup: TreeD,
    keys: Vec<VertID>,

    wireframe_cache: Vec<[Vector3D; 4]>,

    anchors: Vec<EdgeID>,
}

#[derive(Default, Resource)]
pub struct SolutionResource {
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
        .init_resource::<GizmosCache>()
        .init_resource::<SolutionResource>()
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
        .add_systems(Update, raycast.run_if(on_timer(Duration::from_millis(100))))
        .add_event::<ActionEvent>()
        .run();
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

pub fn handle_events(
    mut ev_reader: EventReader<ActionEvent>,
    mut mesh_resmut: ResMut<MeshResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    mut configuration: ResMut<Configuration>,
) {
    for ev in ev_reader.read() {
        info!("Received event {ev:?}. Handling...");
        match ev {
            ActionEvent::LoadFile(path) => {
                mesh_resmut.mesh = match path.extension().unwrap().to_str() {
                    Some("obj") => match Douconel::from_obj(path.to_str().unwrap()) {
                        Ok(res) => res.0,
                        Err(err) => {
                            panic!("Error while parsing STL file {path:?}: {err:?}");
                        }
                    },
                    // Some("stl") => match Douconel::from_stl(path.to_str().unwrap()) {
                    //     Ok(res) => res.0,
                    //     Err(err) => {
                    //         panic!("Error while parsing STL file {path:?}: {err:?}");
                    //     }
                    // },
                    _ => panic!("File format not supported."),
                };

                *configuration = Configuration::default();
                configuration.source = String::from(path.to_str().unwrap());

                configuration.nr_of_vertices = mesh_resmut.mesh.nr_verts();
                configuration.nr_of_edges = mesh_resmut.mesh.nr_edges() / 2; // dcel -> single edge
                configuration.nr_of_faces = mesh_resmut.mesh.nr_faces();

                let mut patterns = TreeD::default();
                mesh_resmut.keys = mesh_resmut.mesh.verts.keys().collect_vec();
                for (i, &v_id) in mesh_resmut.keys.iter().enumerate() {
                    patterns.add(mesh_resmut.mesh.position(v_id).into(), i);
                }
                mesh_resmut.vertex_lookup = patterns;

                let color_map = HashMap::new();

                let mesh = mesh_resmut.mesh.bevy(&color_map);

                let aabb = mesh.compute_aabb().unwrap();
                configuration.scale = 10. * (1. / aabb.half_extents.max_element());
                let translation = -configuration.scale * aabb.center;
                configuration.translation = Vector3D::new(
                    translation.x as f64,
                    translation.y as f64,
                    translation.z as f64,
                );

                configuration.loop_scoring = LoopScoring::PathLength;
                configuration.samples = 1000;

                mesh_resmut.wireframe_cache.clear();
                let keys = mesh_resmut.mesh.edges.keys().collect_vec();
                for edge_id in keys {
                    let endpoints = mesh_resmut.mesh.endpoints(edge_id);
                    let u = mesh_resmut.mesh.position(endpoints.0);
                    let v = mesh_resmut.mesh.position(endpoints.1);
                    let line = DrawableLine::from_line(
                        u,
                        v,
                        configuration.translation,
                        configuration.scale,
                    );
                    gizmos_cache.lines.push((
                        line.u,
                        line.v,
                        hutspot::color::GRIJS.into(),
                        GizmoType::Wireframe,
                    ));
                }

                for vert in mesh_resmut.mesh.verts.keys() {
                    let p = mesh_resmut.mesh.position(vert);
                    let n = mesh_resmut.mesh.vert_normal(vert);
                    let line = DrawableLine::from_vertex(
                        p,
                        n,
                        0.01,
                        configuration.translation,
                        configuration.scale,
                    );
                    gizmos_cache.lines.push((
                        line.u,
                        line.v,
                        hutspot::color::GRIJS.into(),
                        GizmoType::Vertex,
                    ));
                }

                for face in mesh_resmut.mesh.faces.keys() {
                    let p = mesh_resmut.mesh.centroid(face);
                    let n = mesh_resmut.mesh.normal(face);
                    let lines = DrawableLine::from_arrow(
                        p,
                        p + n,
                        p.cross(&n).normalize(),
                        0.05,
                        configuration.translation,
                        configuration.scale,
                    );
                    for line in lines.iter() {
                        gizmos_cache.lines.push((
                            line.u,
                            line.v,
                            hutspot::color::ROODT.into(),
                            GizmoType::Normal,
                        ));
                    }
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
                translation: Vec3::new(
                    configuration.translation.x as f32,
                    configuration.translation.y as f32,
                    configuration.translation.z as f32,
                ),
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
    gizmos_cache: Res<GizmosCache>,

    configuration: Res<Configuration>,

    solution: Res<SolutionResource>,
    mesh_resmut: Res<MeshResource>,
) {
    for &(u, v, c, t) in gizmos_cache.lines.iter() {
        match t {
            GizmoType::Wireframe => {
                if configuration.draw_wireframe {
                    gizmos.line(u, v, c);
                }
            }
            GizmoType::Vertex => {
                if configuration.draw_vertices {
                    gizmos.line(u, v, c);
                }
            }
            GizmoType::Normal => {
                if configuration.draw_normals {
                    gizmos.line(u, v, c);
                }
            }
        }
    }

    for &(u, v, c) in gizmos_cache.raycast.iter() {
        gizmos.line(u, v, c);
    }

    for &(u, v, c) in gizmos_cache.debug.iter() {
        gizmos.line(u, v, c);
    }

    for paths in &solution.dual.paths {
        let color = match paths.direction {
            PrincipalDirection::X => hutspot::color::PURPL.into(),
            PrincipalDirection::Y => hutspot::color::GREEN.into(),
            PrincipalDirection::Z => hutspot::color::ORANG.into(),
        };
        for edge in paths.edges.windows(2) {
            let u = mesh_resmut.mesh.midpoint(edge[0]);
            let v = mesh_resmut.mesh.midpoint(edge[1]);
            let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0]));
            for line in DrawableLine::from_arrow(
                u,
                v,
                n,
                0.9,
                configuration.translation,
                configuration.scale,
            ) {
                gizmos.line(line.u, line.v, color);
            }
        }
    }
}

fn raycast(
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mouse: Res<Input<MouseButton>>,
    mesh_resmut: Res<MeshResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    configuration: ResMut<Configuration>,
) {
    if let Some(cursor_ray) = **cursor_ray {
        let intersections = raycast.cast_ray(cursor_ray, &default());
        if !intersections.is_empty() {
            gizmos_cache.raycast.clear();

            let intersection = &raycast.cast_ray(cursor_ray, &default())[0].1;

            let normal = Vector3D::new(
                intersection.normal().x as f64,
                intersection.normal().y as f64,
                intersection.normal().z as f64,
            );
            let position_in_render = Vector3D::new(
                intersection.position().x as f64,
                intersection.position().y as f64,
                intersection.position().z as f64,
            );
            let triangle_in_render = [
                Vector3D::new(
                    intersection.triangle().unwrap().v0.x as f64,
                    intersection.triangle().unwrap().v0.y as f64,
                    intersection.triangle().unwrap().v0.z as f64,
                ),
                Vector3D::new(
                    intersection.triangle().unwrap().v1.x as f64,
                    intersection.triangle().unwrap().v1.y as f64,
                    intersection.triangle().unwrap().v1.z as f64,
                ),
                Vector3D::new(
                    intersection.triangle().unwrap().v2.x as f64,
                    intersection.triangle().unwrap().v2.y as f64,
                    intersection.triangle().unwrap().v2.z as f64,
                ),
            ]
            .into_iter()
            .sorted_by(|a, b| {
                a.metric_distance(&position_in_render)
                    .partial_cmp(&b.metric_distance(&position_in_render))
                    .unwrap()
            })
            .collect_vec();

            let position = hutspot::draw::invert_transform_coordinates(
                position_in_render,
                configuration.translation,
                configuration.scale,
            );

            let line = DrawableLine::from_vertex(
                position,
                normal,
                5.,
                configuration.translation,
                configuration.scale,
            );
            gizmos_cache
                .raycast
                .push((line.u, line.v, hutspot::color::ROODT.into()));

            // v positions to mesh ids
            let triangle_ids = triangle_in_render
                .into_iter()
                .map(|position_in_render| {
                    mesh_resmut.keys[mesh_resmut
                        .vertex_lookup
                        .nearest(
                            &hutspot::draw::invert_transform_coordinates(
                                position_in_render,
                                configuration.translation,
                                configuration.scale,
                            )
                            .into(),
                        )
                        .1]
                })
                .collect_vec();

            let line = DrawableLine::from_vertex(
                mesh_resmut.mesh.position(triangle_ids[0]),
                mesh_resmut.mesh.vert_normal(triangle_ids[0]),
                0.1,
                configuration.translation,
                configuration.scale,
            );
            gizmos_cache
                .raycast
                .push((line.u, line.v, hutspot::color::ROODT.into()));

            let line = DrawableLine::from_vertex(
                mesh_resmut.mesh.position(triangle_ids[1]),
                mesh_resmut.mesh.vert_normal(triangle_ids[1]),
                0.01,
                configuration.translation,
                configuration.scale,
            );
            gizmos_cache
                .raycast
                .push((line.u, line.v, hutspot::color::GRIJS.into()));

            let line = DrawableLine::from_vertex(
                mesh_resmut.mesh.position(triangle_ids[2]),
                mesh_resmut.mesh.vert_normal(triangle_ids[2]),
                0.01,
                configuration.translation,
                configuration.scale,
            );
            gizmos_cache
                .raycast
                .push((line.u, line.v, hutspot::color::GRIJS.into()));

            let nearest_face = mesh_resmut.mesh.face_with_verts(&triangle_ids).unwrap();

            let (edge1, edge2) = mesh_resmut
                .mesh
                .edges(nearest_face)
                .into_iter()
                .filter(|&edge| {
                    let (u, v) = mesh_resmut.mesh.endpoints(edge);
                    u == triangle_ids[0] || v == triangle_ids[0]
                })
                .collect_tuple()
                .unwrap();

            let total_path = [
                hutspot::graph::find_shortest_cycle(
                    (edge1, edge2),
                    mesh_resmut.mesh.neighbor_function_edgepairgraph(),
                    mesh_resmut.mesh.weight_function_angle_edgepairs_aligned(
                        5,
                        5,
                        configuration.direction.to_vector(),
                    ),
                    &mut cache.cache[configuration.direction as usize],
                ),
                hutspot::graph::find_shortest_cycle(
                    (edge2, edge1),
                    mesh_resmut.mesh.neighbor_function_edgepairgraph(),
                    mesh_resmut.mesh.weight_function_angle_edgepairs_aligned(
                        5,
                        5,
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

            let color = match configuration.direction {
                PrincipalDirection::X => hutspot::color::PURPL.into(),
                PrincipalDirection::Y => hutspot::color::GREEN.into(),
                PrincipalDirection::Z => hutspot::color::ORANG.into(),
            };

            for &edge in &total_path {
                let u = mesh_resmut.mesh.midpoint(edge.0);
                let v = mesh_resmut.mesh.midpoint(edge.1);
                let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge.0));
                for line in DrawableLine::from_arrow(
                    u,
                    v,
                    n,
                    0.9,
                    configuration.translation,
                    configuration.scale,
                ) {
                    gizmos_cache.raycast.push((line.u, line.v, color));
                }
            }

            if mouse.just_pressed(MouseButton::Left) {
                println!("Adding loop to solution.");
                let path = total_path.into_iter().map(|edge| edge.0).collect_vec();
                println!("{path:?}");

                solution.dual.paths.push(Path {
                    edges: path,
                    direction: configuration.direction,
                    order_token: 0.0,
                });
            }
        }
    }
}
