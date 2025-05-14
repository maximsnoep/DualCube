use crate::dual::Orientation;
use crate::CameraHandles;
use crate::{
    to_color, to_principal_direction, vec3_to_vector3d, vector3d_to_vec3, Configuration, InputResource, MainMesh, Perspective, Rendered, RenderedMesh,
    SolutionResource,
};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::camera::ScalingMode;
use bevy::render::render_resource::{Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages};
use douconel::douconel::Douconel;
use douconel::douconel_embedded::HasPosition;
use enum_iterator::{all, Sequence};
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use serde::{Deserialize, Serialize};
use slotmap::Key;
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController};
use std::collections::HashMap;

const BACKGROUND_COLOR_SCREENSHOT_MODE: bevy::prelude::Color = bevy::prelude::Color::srgb(255. / 255., 255. / 255., 255. / 255.);
const BACKGROUND_COLOR: bevy::prelude::Color = bevy::prelude::Color::srgb(27. / 255., 27. / 255., 27. / 255.);
const DEFAULT_CAMERA_EYE: Vec3 = Vec3::new(25.0, 25.0, 35.0);
const DEFAULT_CAMERA_TARGET: Vec3 = Vec3::new(0., 0., 0.);
const DEFAULT_CAMERA_TEXTURE_SIZE: u32 = 640;

#[derive(PartialEq, Eq, Hash, Debug, Copy, Clone, Default, Serialize, Deserialize, Sequence)]
pub enum Objects {
    InputMesh,
    #[default]
    PolycubeMap,
    QuadMesh,
}

#[derive(Clone)]
pub enum RenderAsset {
    Mesh(MeshBundle),
    Gizmo(GizmoBundle),
}

#[derive(Clone)]
pub struct MeshBundle(Mesh3d);
impl MeshBundle {
    pub const fn new(handle: Handle<Mesh>) -> Self {
        Self(Mesh3d(handle))
    }
}

#[derive(Clone)]
pub struct GizmoBundle(Gizmo);

impl GizmoBundle {
    pub fn new(handle: Handle<GizmoAsset>, width: f32, depth: f32) -> Self {
        Self(Gizmo {
            handle,
            line_config: GizmoLineConfig { width, ..Default::default() },
            depth_bias: depth,
        })
    }
}

#[derive(Clone)]
pub struct RenderFeature {
    pub label: String,
    pub visible: bool,
    pub asset: RenderAsset,
}

impl RenderFeature {
    pub fn new(label: &str, visible: bool, asset: RenderAsset) -> Self {
        Self {
            label: label.to_owned(),
            visible,
            asset,
        }
    }
}

#[derive(Clone)]
pub struct RenderObject {
    pub label: String,
    pub features: Vec<RenderFeature>,
}

impl RenderObject {
    pub fn new(label: &str) -> Self {
        Self {
            label: label.to_owned(),
            features: vec![],
        }
    }

    pub fn add(&mut self, feature: RenderFeature) -> &mut Self {
        self.features.push(feature);
        self
    }
}

#[derive(Default, Resource)]
pub struct RenderObjectStore {
    pub objects: HashMap<Objects, RenderObject>,
}

impl RenderObjectStore {
    pub fn add_object(&mut self, object: Objects, render_object: RenderObject) {
        self.objects.insert(object, render_object);
    }

    pub fn clear(&mut self) {
        self.objects.clear();
    }
}

impl From<Objects> for String {
    fn from(val: Objects) -> Self {
        match val {
            Objects::InputMesh => "input mesh",
            Objects::PolycubeMap => "polycube-map",
            Objects::QuadMesh => "quad mesh",
        }
        .to_owned()
    }
}

impl From<Objects> for Vec3 {
    fn from(val: Objects) -> Self {
        match val {
            Objects::InputMesh => Self::new(0., 0., 0.),
            Objects::PolycubeMap => Self::new(0., 1_000., 1_000.),
            Objects::QuadMesh => Self::new(1_000., 0., 1_000.),
        }
    }
}

#[derive(Component, PartialEq, Eq, Hash, Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub struct CameraFor(pub Objects);

pub fn reset(
    commands: &mut Commands,
    cameras: &Query<Entity, With<Camera>>,
    images: &mut ResMut<Assets<Image>>,
    handles: &mut ResMut<CameraHandles>,
    configuration: &ResMut<Configuration>,
) {
    for camera in cameras.iter() {
        commands.entity(camera).despawn();
    }

    // Main camera. This is the camera that the user can control.
    commands
        .spawn((
            Camera3d::default(),
            Camera {
                clear_color: ClearColorConfig::Custom(bevy::prelude::Color::srgb(
                    configuration.clear_color[0] as f32 / 255.,
                    configuration.clear_color[1] as f32 / 255.,
                    configuration.clear_color[2] as f32 / 255.,
                )),
                ..Default::default()
            },
            Tonemapping::None,
        ))
        .insert((OrbitCameraBundle::new(
            OrbitCameraController {
                mouse_rotate_sensitivity: Vec2::splat(0.08),
                mouse_translate_sensitivity: Vec2::splat(0.1),
                mouse_wheel_zoom_sensitivity: 0.2,
                smoothing_weight: 0.8,
                ..Default::default()
            },
            DEFAULT_CAMERA_EYE + Vec3::from(Objects::InputMesh),
            DEFAULT_CAMERA_TARGET + Vec3::from(Objects::InputMesh),
            Vec3::Y,
        ),))
        .insert(CameraFor(Objects::InputMesh));

    // Sub cameras. These cameras render to a texture.
    let mut image = Image {
        texture_descriptor: TextureDescriptor {
            label: None,
            size: Extent3d {
                width: DEFAULT_CAMERA_TEXTURE_SIZE,
                height: DEFAULT_CAMERA_TEXTURE_SIZE,
                ..default()
            },
            dimension: TextureDimension::D2,
            format: TextureFormat::Bgra8UnormSrgb,
            mip_level_count: 1,
            sample_count: 1,
            usage: TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST | TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        },
        ..default()
    };
    image.resize(image.texture_descriptor.size);

    for object in all::<Objects>() {
        let handle = images.add(image.clone());
        handles.map.insert(CameraFor(object), handle.clone());
        let projection = if object == Objects::PolycubeMap {
            let mut proj = OrthographicProjection::default_3d();
            proj.scaling_mode = ScalingMode::FixedVertical { viewport_height: 30. };
            Projection::Orthographic(proj)
        } else {
            bevy::prelude::Projection::default()
        };

        commands.spawn((
            Camera3d::default(),
            Camera {
                target: handle.into(),
                clear_color: ClearColorConfig::Custom(bevy::prelude::Color::srgb(
                    configuration.clear_color[0] as f32 / 255.,
                    configuration.clear_color[1] as f32 / 255.,
                    configuration.clear_color[2] as f32 / 255.,
                )),
                ..Default::default()
            },
            projection,
            Tonemapping::None,
            CameraFor(object),
        ));
    }
}

pub fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut handles: ResMut<CameraHandles>,
    cameras: Query<Entity, With<Camera>>,
    configuration: ResMut<Configuration>,
) {
    self::reset(&mut commands, &cameras, &mut images, &mut handles, &configuration);
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct MeshProperties {
    pub source: String,
    pub scale: f64,
    pub translation: Vector3D,
}

fn get_pbrbundle(
    mesh: Handle<Mesh>,
    translation: Vec3,
    scale: f32,
    material: &Handle<StandardMaterial>,
) -> (Mesh3d, MeshMaterial3d<StandardMaterial>, Transform) {
    (
        Mesh3d(mesh),
        MeshMaterial3d(material.clone()),
        Transform {
            translation,
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(scale),
        },
    )
}

fn get_mesh<VertID: Key, V: Default + HasPosition, EdgeID: Key, E: Default, FaceID: Key, F: Default>(
    dcel: &Douconel<VertID, V, EdgeID, E, FaceID, F>,
    color_map: &HashMap<FaceID, [f32; 3]>,
) -> (Mesh, Vector3D, f64) {
    println!("DEPRECATED!!!! get_mesh");
    dcel.bevy(color_map)
}

pub fn respawn_renders(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    configuration: Res<Configuration>,
    render_object_store: Res<RenderObjectStore>,
    rendered_mesh_query: Query<Entity, With<Rendered>>,
) {
    if render_object_store.is_changed() {
        info!("render_object_store has been changed.");

        info!("Despawn all objects.");
        for entity in rendered_mesh_query.iter() {
            commands.entity(entity).despawn();
        }

        let standard_material = materials.add(StandardMaterial { unlit: true, ..default() });
        let background_material = materials.add(StandardMaterial {
            base_color: bevy::prelude::Color::srgb(
                configuration.clear_color[0] as f32 / 255.,
                configuration.clear_color[1] as f32 / 255.,
                configuration.clear_color[2] as f32 / 255.,
            ),
            unlit: true,
            ..default()
        });

        // Go through render_object_store and spawn all objects (if they are visible).
        info!("Spawn all objects.");
        for (&object, render_object) in &render_object_store.objects {
            for feature in &render_object.features {
                if feature.visible {
                    match &feature.asset {
                        RenderAsset::Mesh(mesh) => {
                            commands.spawn((
                                mesh.0.clone(),
                                MeshMaterial3d(standard_material.clone()),
                                Transform {
                                    translation: Vec3::from(object),
                                    ..Default::default()
                                },
                                Rendered,
                            ));
                        }
                        RenderAsset::Gizmo(gizmo) => {
                            commands.spawn((
                                (
                                    gizmo.0.clone(),
                                    Transform {
                                        translation: Vec3::from(object),
                                        ..Default::default()
                                    },
                                ),
                                Rendered,
                            ));
                        }
                    }
                }
            }
            // Spawning covers such that the objects are view-blocked.
            commands.spawn((
                Mesh3d(meshes.add(Sphere::new(400.))),
                MeshMaterial3d(background_material.clone()),
                Transform::from_translation(Vec3::from(object)),
                Rendered,
            ));
        }
    }
}

pub fn update(
    mut meshes: ResMut<Assets<Mesh>>,
    mut gizmo_assets: ResMut<Assets<GizmoAsset>>,

    mut render_object_store: ResMut<RenderObjectStore>,

    mut mesh_resmut: ResMut<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cameras: Query<(&mut Transform, &mut Projection, &CameraFor)>,
) {
    let main_transform = cameras.iter().find(|(_, _, camera_for)| camera_for.0 == Objects::InputMesh).unwrap().0;
    let normalized_translation = main_transform.translation - Vec3::from(Objects::InputMesh);
    let normalized_rotation = main_transform.rotation;

    let distance = normalized_translation.length();

    for (mut sub_transform, mut sub_projection, sub_object) in &mut cameras {
        sub_transform.translation = normalized_translation + Vec3::from(sub_object.0);
        // println!("translate: {:?}", sub_transform.translation);
        sub_transform.rotation = normalized_rotation;
        if let Projection::Orthographic(orthographic) = sub_projection.as_mut() {
            orthographic.scaling_mode = ScalingMode::FixedVertical { viewport_height: distance };
        }
    }

    // The rest of this function function should only be called when the mesh (RenderedMesh or Solution) is changed.
    if !mesh_resmut.is_changed() && !solution.is_changed() {
        return;
    }
    info!("InputResource or SolutionResource change has been detected. Updating all objects and gizmos.");

    if mesh_resmut.mesh.faces.is_empty() {
        warn!("Current mesh is empty.");
        return;
    }

    render_object_store.clear();

    for object in all::<Objects>() {
        match object {
            // Adds the QUAD MESH to our RenderObjectStore, it has:
            // mesh with black faces
            // mesh with colored faces
            // wireframe (the quads)
            Objects::QuadMesh => {
                if let Some(quad) = &solution.current_solution.quad {
                    let mut color_map = HashMap::new();
                    for face_id in quad.quad_mesh.face_ids() {
                        let normal = quad.quad_mesh_polycube.normal(face_id);
                        let color = to_color(to_principal_direction(normal).0, Perspective::Primal, None);
                        color_map.insert(face_id, [color[0] as f32, color[1] as f32, color[2] as f32]);
                    }

                    render_object_store.add_object(
                        object,
                        RenderObject::new("Quad Mesh")
                            .add(RenderFeature::new(
                                "black",
                                false,
                                RenderAsset::Mesh(MeshBundle::new(meshes.add(get_mesh(&quad.quad_mesh, &HashMap::new()).0))),
                            ))
                            .add(RenderFeature::new(
                                "colored",
                                true,
                                RenderAsset::Mesh(MeshBundle::new(meshes.add(get_mesh(&quad.quad_mesh, &color_map).0))),
                            ))
                            .add(RenderFeature::new(
                                "wireframe",
                                true,
                                RenderAsset::Gizmo(GizmoBundle::new(gizmo_assets.add(quad.quad_mesh.gizmos(hutspot::color::GRAY)), 1., -1e-3)),
                            ))
                            .to_owned(),
                    );
                }
            }
            // Adds the POLYCUBE to our RenderObjectStore, it has:
            // mesh with black faces
            // mesh with colored faces
            // quads mapped on the polycube
            // triangles mapped on the polycube
            Objects::PolycubeMap => {
                if let Some(quad) = &solution.current_solution.quad {
                    let mut color_map = HashMap::new();
                    for face_id in quad.quad_mesh_polycube.face_ids() {
                        let normal = quad.quad_mesh_polycube.normal(face_id);
                        let color = to_color(to_principal_direction(normal).0, Perspective::Primal, None);
                        color_map.insert(face_id, [color[0] as f32, color[1] as f32, color[2] as f32]);
                    }

                    render_object_store.add_object(
                        object,
                        RenderObject::new("Polycube Map")
                            .add(RenderFeature::new(
                                "black",
                                false,
                                RenderAsset::Mesh(MeshBundle::new(meshes.add(get_mesh(&quad.quad_mesh_polycube, &HashMap::new()).0))),
                            ))
                            .add(RenderFeature::new(
                                "colored",
                                true,
                                RenderAsset::Mesh(MeshBundle::new(meshes.add(get_mesh(&quad.quad_mesh_polycube, &color_map).0))),
                            ))
                            .add(RenderFeature::new(
                                "quads",
                                true,
                                RenderAsset::Gizmo(GizmoBundle::new(
                                    gizmo_assets.add(quad.quad_mesh_polycube.gizmos(hutspot::color::GRAY)),
                                    1.,
                                    -1e-3,
                                )),
                            ))
                            .add(RenderFeature::new(
                                "triangles",
                                false,
                                RenderAsset::Gizmo(GizmoBundle::new(
                                    gizmo_assets.add(quad.triangle_mesh_polycube.gizmos(hutspot::color::GRAY)),
                                    2.,
                                    -1e-3,
                                )),
                            ))
                            .to_owned(),
                    );
                }
            }
            Objects::InputMesh => {
                let input = solution.current_solution.mesh_ref.as_ref();
                let (scale, translation) = input.scale_translation();
                let mut gizmos_loop = GizmoAsset::new();
                let mut gizmos_paths = GizmoAsset::new();
                let mut gizmos_flat_paths = GizmoAsset::new();
                let mut color_map_segmentation = HashMap::new();
                let mut color_map_alignment = HashMap::new();

                for (lewp_id, lewp) in &solution.current_solution.loops {
                    let direction = solution.current_solution.loop_to_direction(lewp_id);
                    let color = to_color(direction, Perspective::Dual, Some(Orientation::Forwards));
                    let c = Color::srgb(color[0], color[1], color[2]);

                    let edges = [lewp.edges.clone(), vec![lewp.edges[0]]].concat();

                    for [u, v] in solution.current_solution.get_pairs_of_sequence(&edges) {
                        let line = DrawableLine::from_line(
                            mesh_resmut.mesh.midpoint(u),
                            mesh_resmut.mesh.midpoint(v),
                            Vector3D::new(0., 0., 0.),
                            translation,
                            scale,
                        );
                        gizmos_loop.line(line.u, line.v, c);
                    }
                }

                if let (Ok(lay), Some(polycube)) = (&solution.current_solution.layout, &solution.current_solution.polycube) {
                    let color = hutspot::color::GRAY;
                    let c = Color::srgb(color[0], color[1], color[2]);

                    for (&pedge_id, path) in &lay.edge_to_path {
                        let f1 = polycube.structure.normal(polycube.structure.face(pedge_id));
                        let f2 = polycube.structure.normal(polycube.structure.face(polycube.structure.twin(pedge_id)));
                        for vertexpair in path.windows(2) {
                            if lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).is_none() {
                                println!("Edge between {:?} and {:?} does not exist", vertexpair[0], vertexpair[1]);
                                continue;
                            }
                            let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                            let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                            let u = lay.granulated_mesh.position(u_id);
                            let v = lay.granulated_mesh.position(v_id);
                            let line = DrawableLine::from_line(u, v, Vector3D::new(0., 0., 0.), translation, scale);
                            gizmos_flat_paths.line(line.u, line.v, c);
                            if f1 != f2 {
                                gizmos_paths.line(line.u, line.v, c);
                            }
                        }
                    }

                    for &face_id in &polycube.structure.face_ids() {
                        let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                        let (dir, side) = to_principal_direction(normal);
                        let color = to_color(dir, Perspective::Primal, Some(side));
                        for &triangle_id in &lay.face_to_patch[&face_id].faces {
                            color_map_segmentation.insert(triangle_id, color);
                        }
                    }

                    for &triangle_id in &lay.granulated_mesh.face_ids() {
                        let score = solution.current_solution.alignment_per_triangle[triangle_id];
                        let color = hutspot::color::map(score as f32, &hutspot::color::SCALE_MAGMA);
                        color_map_alignment.insert(triangle_id, color);
                    }
                }

                render_object_store.add_object(
                    object,
                    RenderObject::new("Input Mesh")
                        .add(RenderFeature::new(
                            "black",
                            true,
                            RenderAsset::Mesh(MeshBundle::new(meshes.add(get_mesh(input, &HashMap::new()).0))),
                        ))
                        .add(RenderFeature::new(
                            "segmentation",
                            false,
                            RenderAsset::Mesh(MeshBundle::new(meshes.add(get_mesh(input, &color_map_segmentation).0))),
                        ))
                        .add(RenderFeature::new(
                            "alignment",
                            false,
                            RenderAsset::Mesh(MeshBundle::new(meshes.add(get_mesh(input, &color_map_alignment).0))),
                        ))
                        .add(RenderFeature::new(
                            "wireframe",
                            false,
                            RenderAsset::Gizmo(GizmoBundle::new(gizmo_assets.add(input.gizmos(hutspot::color::GRAY)), 1., -1e-3)),
                        ))
                        .add(RenderFeature::new(
                            "loops",
                            true,
                            RenderAsset::Gizmo(GizmoBundle::new(gizmo_assets.add(gizmos_loop), 4., -2e-3)),
                        ))
                        .add(RenderFeature::new(
                            "paths",
                            false,
                            RenderAsset::Gizmo(GizmoBundle::new(gizmo_assets.add(gizmos_paths), 2., -1e-3)),
                        ))
                        .add(RenderFeature::new(
                            "flat paths",
                            false,
                            RenderAsset::Gizmo(GizmoBundle::new(gizmo_assets.add(gizmos_flat_paths), 1., -1e-3)),
                        ))
                        .to_owned(),
                );
            }
            _ => {}
        }
    }
}

#[derive(Default, Resource)]
pub struct GizmosCache {
    pub raycaster: Vec<Line>,
}

type Line = (Vec3, Vec3, hutspot::color::Color);

// Draws the gizmos. This includes all wireframes, vertices, normals, raycasts, etc.
pub fn gizmos(mut gizmos: Gizmos, gizmos_cache: Res<GizmosCache>, configuration: Res<Configuration>) {
    if configuration.interactive {
        for &(u, v, c) in &gizmos_cache.raycaster {
            gizmos.line(u, v, Color::srgb(c[0], c[1], c[2]));
        }
    }
}

pub fn add_line2(
    lines: &mut Vec<Line>,
    position_a: Vector3D,
    position_b: Vector3D,
    offset: Vector3D,
    color: hutspot::color::Color,
    translation: Vector3D,
    scale: f64,
) {
    let line = DrawableLine::from_line(position_a, position_b, offset, translation, scale);
    lines.push((line.u, line.v, color));
}
