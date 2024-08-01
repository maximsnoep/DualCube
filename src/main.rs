mod draw;
mod dual;
mod elements;
mod ui;

use crate::elements::PrincipalDirection;
use crate::ui::ui;
use bevy::diagnostic::LogDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::{Douconel, EdgeID, Empty, FaceID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use dual::{Dual, Polycube, PropertyViolationError};
use elements::{to_principal_direction, Loop, Side};
use hutspot::color::ROODT;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin};
use smooth_bevy_cameras::LookTransformPlugin;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::BufReader;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::rgb(0. / 255., 0. / 255., 0. / 255.);

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SaveStateObject {
    mesh: MeshResource,
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

pub struct Rules {
    pub intersections: bool,
    pub loop_regions: bool,
}

#[derive(Default, Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RenderType {
    #[default]
    Original,
    Polycube,
}

impl std::fmt::Display for RenderType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Original => write!(f, "Original"),
            Self::Polycube => write!(f, "Polycube"),
        }
    }
}

#[derive(Default, Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DrawLoopType {
    #[default]
    None,
    Undirected,
    Directed,
}

impl std::fmt::Display for DrawLoopType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::None => write!(f, "No loops"),
            Self::Undirected => write!(f, "Undirected loops"),
            Self::Directed => write!(f, "Directed loops (with sides)"),
        }
    }
}

#[derive(Resource, Default, Debug, Clone, Serialize, Deserialize)]
pub struct Configuration {
    pub source: String,
    pub nr_of_faces: usize,
    pub nr_of_edges: usize,
    pub nr_of_vertices: usize,

    pub direction: PrincipalDirection,

    pub alpha: i32,
    pub beta: i32,

    pub sides_mask: [u32; 3],

    pub black: bool,

    pub render_type: RenderType,

    pub scale: f32,
    pub translation: Vector3D,

    pub interactive: bool,
    pub region_selection: bool,
    pub zone_selection: bool,

    pub draw_wireframe: bool,
    pub draw_vertices: bool,
    pub draw_normals: bool,
    pub draw_debug: bool,

    pub cur_selected: Option<(FaceID, VertID)>,

    pub angle_filter: f32,
    pub draw_next: bool,
    pub loop_scoring: LoopScoring,

    pub draw_loop_type: DrawLoopType,

    pub id_selector: usize,

    pub iterations: usize,
    pub samples: usize,
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
pub struct MeshResource {
    mesh: Arc<EmbeddedMesh>,
    vertex_lookup: TreeD,
}

#[derive(Debug, Clone, Resource, Serialize, Deserialize)]
pub struct SolutionResource {
    dual: Dual,
    primal: Result<Polycube, PropertyViolationError>,
    next: HashMap<(FaceID, VertID), Option<(Dual, Result<Polycube, PropertyViolationError>)>>,
}

impl Default for SolutionResource {
    fn default() -> Self {
        Self {
            dual: Dual::default(),
            primal: Err(PropertyViolationError::UnknownError),
            next: HashMap::new(),
        }
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
        .add_systems(Update, update_mesh)
        .add_systems(Update, raycast)
        .add_event::<ActionEvent>()
        .run();
}

/// Set up
fn setup(mut commands: Commands, mut ui: bevy_egui::EguiContexts) {
    commands.spawn(Camera3dBundle::default()).insert((OrbitCameraBundle::new(
        OrbitCameraController::default(),
        Vec3::new(0.0, 5.0, 20.0),
        Vec3::new(0., 0., 0.),
        Vec3::Y,
    ),));

    let mut fonts = bevy_egui::egui::FontDefinitions::default();
    fonts.font_data.insert(
        "BerkeleyMonoTrial".to_owned(),
        bevy_egui::egui::FontData::from_static(include_bytes!("../assets/BerkeleyMonoTrial-Regular.ttf")),
    );
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "BerkeleyMonoTrial".to_owned());
    fonts
        .families
        .entry(bevy_egui::egui::FontFamily::Monospace)
        .or_default()
        .push("BerkeleyMonoTrial".to_owned());
    ui.ctx_mut().set_fonts(fonts);
    ui.ctx_mut().set_visuals(bevy_egui::egui::Visuals::dark());
}

pub fn handle_events(
    mut ev_reader: EventReader<ActionEvent>,
    mut mesh_resmut: ResMut<MeshResource>,
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
                        *mesh_resmut = MeshResource::default();
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

                configuration.source = String::from(path.to_str().unwrap());

                configuration.nr_of_vertices = mesh_resmut.mesh.nr_verts();
                configuration.nr_of_edges = mesh_resmut.mesh.nr_edges() / 2; // dcel -> single edge
                configuration.nr_of_faces = mesh_resmut.mesh.nr_faces();

                let mesh = mesh_resmut.mesh.bevy(&HashMap::new());
                let aabb = mesh.compute_aabb().unwrap();
                configuration.scale = 10. * (1. / aabb.half_extents.max_element());
                let translation = -configuration.scale * aabb.center;
                configuration.translation = Vector3D::new(translation.x.into(), translation.y.into(), translation.z.into());

                for edge_id in mesh_resmut.mesh.edge_ids() {
                    draw::add_edge(
                        &mut gizmos_cache.lines,
                        edge_id,
                        &mesh_resmut.mesh,
                        configuration.translation,
                        configuration.scale,
                    );
                }

                for vert_id in mesh_resmut.mesh.vert_ids() {
                    draw::add_vertex(
                        &mut gizmos_cache.lines,
                        vert_id,
                        &mesh_resmut.mesh,
                        configuration.translation,
                        configuration.scale,
                    );
                }

                for face_id in mesh_resmut.mesh.face_ids() {
                    draw::add_face_normal(
                        &mut gizmos_cache.lines,
                        face_id,
                        &mesh_resmut.mesh,
                        configuration.translation,
                        configuration.scale,
                    );
                }
            }
            ActionEvent::ExportState => {
                let path = format!(
                    "./out/{}_{:?}.save",
                    configuration.source.split("\\").last().unwrap().split(".").next().unwrap(),
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

// This function should be called when the mesh (RenderedMesh) is changed, to make sure that modifications are visualized.
fn update_mesh(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    mesh_resmut: Res<MeshResource>,
    solution: Res<SolutionResource>,
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

    match configuration.render_type {
        RenderType::Original => {
            let color_map = HashMap::new();
            let mesh = mesh_resmut.mesh.bevy(&color_map);

            // Spawn new mesh
            commands.spawn((
                MaterialMeshBundle {
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
        RenderType::Polycube => {
            let mut color_map = HashMap::new();
            if let Ok(polycube) = &solution.primal {
                for &face_id in &polycube.face_ids() {
                    let normal = (polycube.normal(face_id) as Vector3D).normalize();
                    let (dir, side) = to_principal_direction(normal);
                    let color = dir.to_primal_color();
                    color_map.insert(face_id, [color.r(), color.g(), color.b()]);
                }
                let mesh = polycube.bevy(&color_map);
                let aabb = mesh.compute_aabb().unwrap();
                let scale = 10. * (1. / aabb.half_extents.max_element());
                let translation = -configuration.scale * aabb.center;

                // Spawn new mesh
                commands.spawn((
                    MaterialMeshBundle {
                        mesh: meshes.add(mesh),
                        transform: Transform {
                            translation: Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32),
                            rotation: Quat::from_rotation_z(0f32),
                            scale: Vec3::splat(scale),
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
        }
    };
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    gizmos_cache: Res<GizmosCache>,
    configuration: Res<Configuration>,
    solution: Res<SolutionResource>,
    mesh_resmut: Res<MeshResource>,
) {
    for &(u, v, c, t) in &gizmos_cache.lines {
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

    for (&(face_id, vert_id), sol) in &solution.next {
        let u = mesh_resmut.mesh.position(vert_id);
        let v = mesh_resmut.mesh.centroid(face_id);
        let n = mesh_resmut.mesh.normal(face_id);
        let line = DrawableLine::from_line(u, v, n * 0.01, configuration.translation, configuration.scale);
        match sol {
            Some(_) => gizmos.line(line.u, line.v, hutspot::color::GREEN.into()),
            None => gizmos.line(line.u, line.v, hutspot::color::ROODT.into()),
        }
    }

    for &(u, v, c) in &gizmos_cache.raycast {
        gizmos.line(u, v, c);
    }

    for &(u, v, c) in &gizmos_cache.debug {
        gizmos.line(u, v, c);
    }

    match (configuration.draw_loop_type, configuration.render_type) {
        (DrawLoopType::Undirected, RenderType::Original) => {
            // Draw all loops
            for l in solution.dual.get_loops() {
                for edge in &solution.dual.get_pairs_of_sequence(&l.edges) {
                    let u = mesh_resmut.mesh.midpoint(edge[0]);
                    let v = mesh_resmut.mesh.midpoint(edge[1]);
                    let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0]));
                    let line = DrawableLine::from_line(u, v, n * 0.01, configuration.translation, configuration.scale);
                    gizmos.line(line.u, line.v, l.direction.to_dual_color());
                }
            }

            if let Ok(primal) = &solution.primal {
                for v in primal.vert_ids() {
                    let vertex = primal.verts[v].pointer_primal_vertex;

                    // draw vertex
                    let u = mesh_resmut.mesh.position(vertex);
                    let n = mesh_resmut.mesh.vert_normal(vertex);
                    let line = DrawableLine::from_vertex(u, n, 0.05, configuration.translation, configuration.scale);
                    gizmos.line(line.u, line.v, ROODT.into());
                }

                for e in primal.edge_ids() {
                    let path = &primal.edges[e];
                    for vertexpair in path.windows(2) {
                        let u = mesh_resmut.mesh.position(vertexpair[0]);
                        let v = mesh_resmut.mesh.position(vertexpair[1]);
                        let edge = mesh_resmut.mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                        let n = mesh_resmut.mesh.edge_normal(edge);
                        // draw the line
                        let line = DrawableLine::from_line(u, v, n * 0.01, configuration.translation, configuration.scale);
                        gizmos.line(line.u, line.v, ROODT.into());
                    }
                }
            }
        }
        (DrawLoopType::Directed, RenderType::Original) => {
            // Draw all loop segments
            let loopstruct = solution.dual.get_loop_structure();
            for segment in loopstruct.edge_ids() {
                let pairs_between = solution.dual.get_pairs_of_sequence(&solution.dual.segment_to_edges(segment));

                let dir = solution.dual.segment_to_direction(segment);

                let adjacent_face = loopstruct.face(segment);
                let centroid = hutspot::math::calculate_average_f64(loopstruct.faces[adjacent_face].verts.iter().map(|&vert| mesh_resmut.mesh.position(vert)));

                for edge in pairs_between {
                    let vector0_to_centroid = (centroid - mesh_resmut.mesh.midpoint(edge[0])).normalize();
                    let vector1_to_centroid = (centroid - mesh_resmut.mesh.midpoint(edge[1])).normalize();

                    let u = mesh_resmut.mesh.midpoint(edge[0]) + vector0_to_centroid * 0.02;
                    let v = mesh_resmut.mesh.midpoint(edge[1]) + vector1_to_centroid * 0.02;
                    let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0]));
                    for line in DrawableLine::from_arrow(
                        u,
                        v,
                        n,
                        0.9,
                        mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0])) * 0.2,
                        configuration.translation,
                        configuration.scale,
                    ) {
                        let side = solution.dual.segment_to_side(segment, configuration.sides_mask);
                        gizmos.line(line.u, line.v, dir.to_dual_color_sided(side));
                    }
                }
            }
        }
        (DrawLoopType::Undirected, RenderType::Polycube) => {
            // Draw all loop segments / faces axis aligned.

            if let Ok(primal) = &solution.primal {
                for (face_id, original_id) in &primal.faces {
                    let this_centroid = primal.centroid(face_id);
                    let normal = (primal.normal(face_id) as Vector3D).normalize();
                    let orientation = to_principal_direction(normal).0;
                    for &neighbor_id in &primal.fneighbors(face_id) {
                        let next_original_id = &primal.faces[neighbor_id];
                        let edge_between = primal.edge_between_faces(face_id, neighbor_id).unwrap().0;
                        let root = primal.root(edge_between);
                        let root_pos = primal.position(root);
                        let segment = solution
                            .dual
                            .get_loop_structure()
                            .edge_between_verts(*original_id, *next_original_id)
                            .unwrap()
                            .0;
                        let direction = solution.dual.segment_to_direction(segment);
                        let segment_direction = match (orientation, direction) {
                            (PrincipalDirection::X, PrincipalDirection::Y) | (PrincipalDirection::Y, PrincipalDirection::X) => PrincipalDirection::Z,
                            (PrincipalDirection::X, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::X) => PrincipalDirection::Y,
                            (PrincipalDirection::Y, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::Y) => PrincipalDirection::X,
                            _ => unreachable!(),
                        };
                        let mut direction_vector = this_centroid;
                        direction_vector[segment_direction as usize] = root_pos[segment_direction as usize];

                        let line = DrawableLine::from_line(
                            this_centroid,
                            direction_vector,
                            Vector3D::new(0., 0., 0.),
                            configuration.translation,
                            configuration.scale,
                        );

                        gizmos.line(line.u, line.v, direction.to_dual_color());
                    }
                }
            }
        }
        (DrawLoopType::Directed, RenderType::Polycube) => {
            // Draw all loop segments / faces axis aligned.
            if let Ok(primal) = &solution.primal {
                for (face_id, original_id) in &primal.faces {
                    let this_centroid = primal.centroid(face_id);

                    let normal = (primal.normal(face_id) as Vector3D).normalize();
                    let orientation = to_principal_direction(normal).0;

                    for &neighbor_id in &primal.fneighbors(face_id) {
                        let next_original_id = &primal.faces[neighbor_id];

                        let edge_between = primal.edge_between_faces(face_id, neighbor_id).unwrap().0;
                        let root = primal.root(edge_between);
                        let root_pos = primal.position(root);

                        let segment = solution
                            .dual
                            .get_loop_structure()
                            .edge_between_verts(*original_id, *next_original_id)
                            .unwrap()
                            .0;

                        let direction = solution.dual.segment_to_direction(segment);

                        for side in [Side::Upper, Side::Lower] {
                            let segment_direction = match (orientation, direction) {
                                (PrincipalDirection::X, PrincipalDirection::Y) | (PrincipalDirection::Y, PrincipalDirection::X) => PrincipalDirection::Z,
                                (PrincipalDirection::X, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::X) => PrincipalDirection::Y,
                                (PrincipalDirection::Y, PrincipalDirection::Z) | (PrincipalDirection::Z, PrincipalDirection::Y) => PrincipalDirection::X,
                                _ => unreachable!(),
                            };

                            let mut direction_vector = this_centroid;
                            direction_vector[segment_direction as usize] = root_pos[segment_direction as usize];

                            let mut offset = Vector3D::new(0., 0., 0.);

                            let dist = 0.01;

                            match side {
                                Side::Upper => match direction {
                                    PrincipalDirection::X => offset[0] -= dist,
                                    PrincipalDirection::Y => offset[1] += dist,
                                    PrincipalDirection::Z => offset[2] -= dist,
                                },
                                Side::Lower => match direction {
                                    PrincipalDirection::X => offset[0] += dist,
                                    PrincipalDirection::Y => offset[1] -= dist,
                                    PrincipalDirection::Z => offset[2] += dist,
                                },
                            };

                            let line = DrawableLine::from_line(this_centroid, direction_vector, offset, configuration.translation, configuration.scale);

                            gizmos.line(line.u, line.v, direction.to_dual_color_sided(side));
                        }
                    }
                }
            }
        }
        _ => (),
    }
}

fn raycast(
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mut mouse: ResMut<Input<MouseButton>>,
    mesh_resmut: Res<MeshResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    mut configuration: ResMut<Configuration>,
) {
    configuration.cur_selected = None;

    if configuration.render_type != RenderType::Original || (!configuration.interactive && !configuration.region_selection && !configuration.zone_selection) {
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

    let position = hutspot::draw::invert_transform_coordinates(position_in_render, configuration.translation, configuration.scale);
    let line = DrawableLine::from_vertex(position, normal, 5., configuration.translation, configuration.scale);
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
                .nearest(&hutspot::draw::invert_transform_coordinates(pos, configuration.translation, configuration.scale).into())
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
                    configuration.translation,
                    configuration.scale,
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

    println!("Occupied edges: {:?}", occupied);

    // The neighborhood function: filters out all edges that are already used in the solution
    let nfunction = |edgepair: [EdgeID; 2]| {
        let self_intersection = edgepair
            .iter()
            .map(|&edge| solution.dual.loops_on_edge(edge))
            .flatten()
            .filter(|&loop_id| solution.dual.loop_to_direction(loop_id) == configuration.direction)
            .count()
            > 0;

        if self_intersection || occupied.get(&edgepair).is_some() {
            vec![]
        } else {
            mesh_resmut.mesh.neighbor_function_edgepairgraph()(edgepair)
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
    .into_iter()
    .flatten()
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

    println!("path: {:?}", total_path);

    // Clean path, if duplicated vertices are present, remove everything between them.
    let mut cur_path = vec![];
    for v in total_path {
        if cur_path.contains(&v) {
            println!("CUTTING PATH");
            cur_path = cur_path.into_iter().take_while(|&x| x != v).collect_vec();
            cur_path.push(v);
        } else {
            cur_path.push(v);
        }
    }

    let total_path = cur_path;

    println!("Adding loop...");
    let mut dual_copy = solution.dual.clone();
    let loop_id = dual_copy.add_loop(Loop {
        edges: total_path,
        direction: configuration.direction,
    });

    println!("Verify loop structure");

    let polycube = dual_copy.build_loop_structure();

    solution.next.insert((face_id, vert_id), Some((dual_copy, polycube)));
}
