#![warn(clippy::all, clippy::pedantic, clippy::nursery, clippy::cargo)]
#![allow(clippy::missing_panics_doc, clippy::missing_errors_doc)]
mod dual;
mod ui;

use crate::dual::PrincipalDirection;
use crate::ui::ui;
use bevy::diagnostic::LogDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::window::WindowMode;
use bevy_egui::EguiPlugin;
use bevy_mod_raycast::prelude::*;
use douconel::douconel::{Douconel, EdgeID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use dual::{Dual, Loop};
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
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::Arc;

const BACKGROUND_COLOR: bevy::prelude::Color =
    bevy::prelude::Color::rgb(27. / 255., 27. / 255., 27. / 255.);

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

#[derive(Default, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RenderType {
    #[default]
    Original,
    Polycube,
}

#[derive(Default, Resource, Clone)]
pub struct Configuration {
    pub source: String,
    pub nr_of_faces: usize,
    pub nr_of_edges: usize,
    pub nr_of_vertices: usize,

    pub direction: PrincipalDirection,

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

#[derive(Default, Resource)]
pub struct GizmosCache {
    lines: Vec<(Vec3, Vec3, Color, GizmoType)>,
    raycast: Vec<(Vec3, Vec3, Color)>,
    debug: Vec<(Vec3, Vec3, Color)>,
}

type EmbeddedMesh = Douconel<EmbeddedVertex, (), ()>;

#[derive(Default, Resource)]
pub struct MeshResource {
    mesh: Arc<EmbeddedMesh>,

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
        .add_systems(Update, update_mesh)
        .add_systems(Update, raycast)
        .add_event::<ActionEvent>()
        .run();
}

/// Set up
fn setup(mut commands: Commands, mut egui_ctx: bevy_egui::EguiContexts) {
    commands
        .spawn(Camera3dBundle::default())
        .insert((OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(0.0, 5.0, 20.0),
            Vec3::new(0., 0., 0.),
            Vec3::Y,
        ),));

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
        .set_visuals(bevy_egui::egui::Visuals::dark());
}

#[must_use]
pub fn dir_to_color(direction: PrincipalDirection) -> Color {
    match direction {
        PrincipalDirection::X => hutspot::color::GREEN.into(),
        PrincipalDirection::Y => hutspot::color::ORANG.into(),
        PrincipalDirection::Z => hutspot::color::PURPL.into(),
    }
}

pub fn dir_to_color2(direction: PrincipalDirection, label: crate::dual::Label) -> Color {
    match (direction, label) {
        (PrincipalDirection::X, crate::dual::Label::Plus) => hutspot::color::GREEN.into(),
        (PrincipalDirection::X, crate::dual::Label::Minus) => hutspot::color::GREEN_L.into(),
        (PrincipalDirection::Y, crate::dual::Label::Plus) => hutspot::color::ORANG.into(),
        (PrincipalDirection::Y, crate::dual::Label::Minus) => hutspot::color::ORANG_L.into(),
        (PrincipalDirection::Z, crate::dual::Label::Plus) => hutspot::color::PURPL.into(),
        (PrincipalDirection::Z, crate::dual::Label::Minus) => hutspot::color::PURPL_L.into(),
    }
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
                mesh_resmut.mesh = Arc::new(match path.extension().unwrap().to_str() {
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
                });

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

                solution.dual = Dual::new(mesh_resmut.mesh.clone());

                let color_map = HashMap::new();

                let mesh = mesh_resmut.mesh.bevy(&color_map);

                let aabb = mesh.compute_aabb().unwrap();
                configuration.scale = 10. * (1. / aabb.half_extents.max_element());
                let translation = -configuration.scale * aabb.center;
                configuration.translation = Vector3D::new(
                    translation.x.into(),
                    translation.y.into(),
                    translation.z.into(),
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
                        mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge_id)) * 0.001,
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
                        Vector3D::new(0., 0., 0.),
                        configuration.translation,
                        configuration.scale,
                    );
                    for line in &lines {
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

    let mesh = match configuration.render_type {
        RenderType::Original => {
            let color_map = HashMap::new();
            mesh_resmut.mesh.bevy(&color_map)
        }
        RenderType::Polycube => {
            let mut color_map = HashMap::new();
            for (face_id, face_obj) in &solution.dual.primal.faces {
                // let dirs = face_obj
                //     .next
                //     .iter()
                //     .map(|&(_, loop_id, _, _)| solution.dual.get_loop(loop_id).unwrap().direction)
                //     .unique()
                //     .collect_tuple();

                // let color = match dirs {
                //     Some((PrincipalDirection::Y, PrincipalDirection::Z))
                //     | Some((PrincipalDirection::Z, PrincipalDirection::Y)) => {
                //         hutspot::color::ROODT.into()
                //     }
                //     Some((PrincipalDirection::X, PrincipalDirection::Z))
                //     | Some((PrincipalDirection::Z, PrincipalDirection::X)) => {
                //         hutspot::color::BLAUW.into()
                //     }
                //     Some((PrincipalDirection::X, PrincipalDirection::Y))
                //     | Some((PrincipalDirection::Y, PrincipalDirection::X)) => {
                //         hutspot::color::YELLO.into()
                //     }
                //     _ => hutspot::color::BLACK.into(),
                // };

                // color_map.insert(face_id, color);
            }
            solution.dual.primal.bevy(&color_map)
        }
    };

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

    for &(u, v, c) in &gizmos_cache.raycast {
        gizmos.line(u, v, c);
    }

    for &(u, v, c) in &gizmos_cache.debug {
        gizmos.line(u, v, c);
    }

    // Draw all loops
    // for (loop_id, l) in solution.dual.get_loops() {
    //     let color = dir_to_color(l.direction);
    //     for edge in &solution.dual.get_pairs_of_loop(loop_id) {
    //         let u = mesh_resmut.mesh.midpoint(edge[0]);
    //         let v = mesh_resmut.mesh.midpoint(edge[1]);
    //         let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0]));
    //         for line in DrawableLine::from_arrow(
    //             u,
    //             v,
    //             n,
    //             0.9,
    //             mesh_resmut.mesh.normal(mesh_resmut.mesh.face(edge[0])) * 0.001,
    //             configuration.translation,
    //             configuration.scale,
    //         ) {
    //             gizmos.line(line.u, line.v, color);
    //         }
    //     }
    // }

    // Draw all loop segments
    if configuration.render_type != RenderType::Polycube {
        let loopstruct = solution.dual.get_loop_structure();
        for (ls_id, ls) in &loopstruct.edges {
            let edges_between = &loopstruct.edges[ls_id].between;
            let pairs_between = solution.dual.get_pairs_of_sequence(
                &[vec![ls.start], edges_between.clone(), vec![ls.end]].concat(),
            );

            let dir = solution.dual.get_loop(ls.loop_id).unwrap().direction;

            let adjacent_face = loopstruct.face(ls_id);
            let centroid = hutspot::math::calculate_average_f64(
                loopstruct.faces[adjacent_face]
                    .verts
                    .iter()
                    .map(|&vert| mesh_resmut.mesh.position(vert)),
            );

            for edge in pairs_between {
                let vector0_to_centroid =
                    (centroid - mesh_resmut.mesh.midpoint(edge[0])).normalize();
                let vector1_to_centroid =
                    (centroid - mesh_resmut.mesh.midpoint(edge[1])).normalize();

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
                    gizmos.line(
                        line.u,
                        line.v,
                        dir_to_color2(dir, ls.labeling.clone().unwrap()),
                    );
                }
            }
        }
    }

    if configuration.render_type == RenderType::Polycube {
        // Draw all loop segments / faces axis aligned.

        for (face_id, face_obj) in &solution.dual.primal.faces {
            let this_centroid = solution.dual.primal.centroid(face_id);
            let this_orientation = match face_obj
                .next
                .iter()
                .map(|&(_, loop_id, _, _)| solution.dual.get_loop(loop_id).unwrap().direction)
                .unique()
                .collect_tuple()
            {
                Some((PrincipalDirection::X, PrincipalDirection::Y))
                | Some((PrincipalDirection::Y, PrincipalDirection::X)) => PrincipalDirection::Z,
                Some((PrincipalDirection::X, PrincipalDirection::Z))
                | Some((PrincipalDirection::Z, PrincipalDirection::X)) => PrincipalDirection::Y,
                Some((PrincipalDirection::Y, PrincipalDirection::Z))
                | Some((PrincipalDirection::Z, PrincipalDirection::Y)) => PrincipalDirection::X,
                _ => PrincipalDirection::X,
            };

            for &neighbor_id in &solution.dual.primal.fneighbors(face_id) {
                let neighbor_obj = &solution.dual.primal.faces[neighbor_id];

                let loop_id = face_obj
                    .next
                    .iter()
                    .find(|&&(_, _, next_id, _)| next_id == neighbor_obj.this)
                    .unwrap()
                    .1;
                let direction = solution.dual.get_loop(loop_id).unwrap().direction;

                let mut next_centroid = solution.dual.primal.centroid(neighbor_id);
                next_centroid[this_orientation as usize] = this_centroid[this_orientation as usize];

                let line = DrawableLine::from_line(
                    this_centroid,
                    next_centroid,
                    Vector3D::new(0., 0., 0.),
                    configuration.translation,
                    configuration.scale,
                );

                gizmos.line(line.u, line.v, dir_to_color(direction));
            }
        }

        for (vert_id, _) in solution.dual.primal.verts.iter() {
            for &neighbor_id in &solution.dual.primal.vneighbors(vert_id) {
                let u = mesh_resmut.mesh.position(vert_id);
                let v = mesh_resmut.mesh.position(neighbor_id);
                let line = DrawableLine::from_line(
                    u,
                    v,
                    Vector3D::new(0., 0., 0.),
                    configuration.translation,
                    configuration.scale,
                );
                gizmos.line(line.u, line.v, hutspot::color::WHITE.into());
            }
        }
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
    configuration: Res<Configuration>,
) {
    if !configuration.interactive
        && !configuration.region_selection
        && !configuration.zone_selection
    {
        gizmos_cache.raycast.clear();
        return;
    }

    if configuration.render_type != RenderType::Original {
        gizmos_cache.raycast.clear();
        return;
    }

    if let Some(cursor_ray) = **cursor_ray {
        let intersections = raycast.cast_ray(cursor_ray, &default());
        if !intersections.is_empty() {
            gizmos_cache.raycast.clear();

            let intersection = &raycast.cast_ray(cursor_ray, &default())[0].1;

            let n = intersection.normal();
            let p = intersection.position();
            let normal = Vector3D::new(n.x.into(), n.y.into(), n.z.into());
            let position_in_render = Vector3D::new(p.x.into(), p.y.into(), p.z.into());

            let t = intersection.triangle().unwrap();
            let triangle_in_render = [
                Vector3D::new(t.v0.x.into(), t.v0.y.into(), t.v0.z.into()),
                Vector3D::new(t.v1.x.into(), t.v1.y.into(), t.v1.z.into()),
                Vector3D::new(t.v2.x.into(), t.v2.y.into(), t.v2.z.into()),
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

            if configuration.region_selection {
                let selected_vert = triangle_ids[0];

                let ls = solution.dual.get_loop_structure();
                for (face_id, _) in ls.faces.iter() {
                    if !ls.faces[face_id].verts.contains(&selected_vert) {
                        continue;
                    }

                    for &vert_id in &ls.faces[face_id].verts {
                        let p = mesh_resmut.mesh.position(vert_id);
                        let n = mesh_resmut.mesh.vert_normal(vert_id);
                        let line = DrawableLine::from_vertex(
                            p,
                            n,
                            0.05,
                            configuration.translation,
                            configuration.scale,
                        );

                        gizmos_cache
                            .raycast
                            .push((line.u, line.v, ls.faces[face_id].color));
                    }
                }
            } else if configuration.zone_selection {
                let selected_vert = triangle_ids[0];

                let ls = solution.dual.get_loop_structure();
                for zone in &solution.dual.zones {
                    let dir = solution
                        .dual
                        .get_loop(ls.edges[*zone.boundary.iter().next().unwrap()].loop_id)
                        .unwrap()
                        .direction;

                    if dir != configuration.direction {
                        continue;
                    }

                    let zone_verts: HashSet<_> = zone
                        .subsurfaces
                        .iter()
                        .flat_map(|&subsurface| ls.faces[subsurface].verts.iter().copied())
                        .collect();

                    if !zone_verts.contains(&selected_vert) {
                        continue;
                    }

                    for &vert_id in &zone_verts {
                        let p = mesh_resmut.mesh.position(vert_id);
                        let n = mesh_resmut.mesh.vert_normal(vert_id);
                        let line = DrawableLine::from_vertex(
                            p,
                            n,
                            0.05,
                            configuration.translation,
                            configuration.scale,
                        );

                        gizmos_cache
                            .raycast
                            .push((line.u, line.v, dir_to_color(dir)));
                    }
                }
            } else if configuration.interactive {
                let (e1, e2) = mesh_resmut
                    .mesh
                    .edges(mesh_resmut.mesh.face_with_verts(&triangle_ids).unwrap())
                    .into_iter()
                    .filter(|&edge| {
                        let (u, v) = mesh_resmut.mesh.endpoints(edge);
                        u == triangle_ids[0] || v == triangle_ids[0]
                    })
                    .collect_tuple()
                    .unwrap();

                // filter out all edges that are already used in the solution
                let nfunction = |edgepair: [EdgeID; 2]| {
                    if solution.dual.is_occupied(edgepair).is_some() {
                        vec![]
                    } else {
                        mesh_resmut.mesh.neighbor_function_edgepairgraph()(edgepair)
                    }
                };

                let wfunction = mesh_resmut.mesh.weight_function_angle_edgepairs_aligned(
                    5,
                    5,
                    configuration.direction.to_vector(),
                );

                let cache_ref = &mut cache.cache[configuration.direction as usize];

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

                if total_path.is_empty() {
                    return;
                }

                if let Some(loop_id) = solution.dual.add_loop(Loop {
                    edges: total_path,
                    direction: configuration.direction,
                }) {
                    for intersection in &solution.dual.intersections {
                        let u = mesh_resmut.mesh.midpoint(*intersection.0);
                        let n = mesh_resmut
                            .mesh
                            .normal(mesh_resmut.mesh.face(*intersection.0));
                        let line = DrawableLine::from_vertex(
                            u,
                            n,
                            0.9,
                            configuration.translation,
                            configuration.scale,
                        );
                        gizmos_cache
                            .raycast
                            .push((line.u, line.v, hutspot::color::GRIJS.into()));

                        for &nexts in &intersection.1.next {
                            let v = mesh_resmut.mesh.midpoint(nexts.0);
                            let n = mesh_resmut.mesh.normal(mesh_resmut.mesh.face(nexts.0));
                            let line = DrawableLine::from_vertex(
                                v,
                                n,
                                0.9,
                                configuration.translation,
                                configuration.scale,
                            );
                            gizmos_cache.raycast.push((
                                line.u,
                                line.v,
                                hutspot::color::ROODT.into(),
                            ));
                        }
                    }

                    for &edge in &solution.dual.get_pairs_of_loop(loop_id) {
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
                            gizmos_cache.raycast.push((
                                line.u,
                                line.v,
                                dir_to_color(configuration.direction),
                            ));
                        }
                    }

                    let ls = solution.dual.get_loop_structure();

                    for face in &ls.faces {
                        // Get the edges of the face
                        let edges = ls.edges(face.0);
                        // All loops must be unique
                        let unique = edges.len()
                            == edges
                                .iter()
                                .map(|&edge| ls.edges[edge].loop_id)
                                .unique()
                                .count();

                        // At most 2 loops per color
                        let x_count = edges
                            .iter()
                            .map(|&edge| ls.edges[edge].loop_id)
                            .filter(|&loop_id| {
                                solution.dual.get_loop(loop_id).unwrap().direction
                                    == PrincipalDirection::X
                            })
                            .count();

                        let y_count = edges
                            .iter()
                            .map(|&edge| ls.edges[edge].loop_id)
                            .filter(|&loop_id| {
                                solution.dual.get_loop(loop_id).unwrap().direction
                                    == PrincipalDirection::Y
                            })
                            .count();

                        let z_count = edges
                            .iter()
                            .map(|&edge| ls.edges[edge].loop_id)
                            .filter(|&loop_id| {
                                solution.dual.get_loop(loop_id).unwrap().direction
                                    == PrincipalDirection::Z
                            })
                            .count();

                        // Atleast 3 loops

                        // if !(edges.len() >= 3
                        //     && unique
                        //     && x_count <= 2
                        //     && y_count <= 2
                        //     && z_count <= 2)
                        // {
                        //     println!("oopsie:::)");
                        //     solution.dual.del_loop(loop_id);
                        //     return;
                        // }

                        // TODO: same colors not allowed to intersect

                        // TODO: loop must be connected..
                    }

                    if mouse.just_pressed(MouseButton::Left)
                        || mouse.just_released(MouseButton::Left)
                    {
                        mouse.clear_just_pressed(MouseButton::Left);
                        mouse.clear_just_released(MouseButton::Left);

                        return;
                    } else {
                        solution.dual.del_loop(loop_id);
                        return;
                    }
                }
            }
        }
    }
}
