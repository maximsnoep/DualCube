use crate::dual::{to_principal_direction, PrincipalDirection, Side};
use crate::{vec3_to_vector3d, Configuration, InputResource, MainMesh, RenderedMesh, SolutionResource, VertID, BACKGROUND_COLOR};
use crate::{CameraHandles, EmbeddedMesh};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::camera::ScalingMode;
use bevy::render::render_resource::{Extent3d, TextureDescriptor, TextureDimension, TextureFormat, TextureUsages};
use douconel::douconel::Douconel;
use douconel::douconel_embedded::HasPosition;
use enum_iterator::{all, Sequence};
use hutspot::consts::PI;
use hutspot::draw::DrawableLine;
use hutspot::geom::Vector3D;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};
use slotmap::Key;
use smooth_bevy_cameras::controllers::orbit::{OrbitCameraBundle, OrbitCameraController};
use std::cmp::Reverse;
use std::collections::HashMap;

const DEFAULT_CAMERA_EYE: Vec3 = Vec3::new(25.0, 25.0, 35.0);
const DEFAULT_CAMERA_TARGET: Vec3 = Vec3::new(0., 0., 0.);
const DEFAULT_CAMERA_TEXTURE_SIZE: u32 = 640;

#[derive(PartialEq, Eq, Hash, Debug, Copy, Clone, Default, Serialize, Deserialize, Sequence)]
pub enum Objects {
    MeshDualLoops,
    #[default]
    PolycubeDual,
    PolycubePrimal,
    MeshPolycubeLayout,
    MeshInput,
    MeshAlignmentScore,
    MeshOrthogonalityScore,
    MeshPolycubeLabeling,
    PolycubePrimalPolyhedron,
    Debug,
    Debug2,
}

impl From<Objects> for String {
    fn from(val: Objects) -> Self {
        match val {
            Objects::MeshDualLoops => "dual loops",
            Objects::PolycubeDual => "polycube (dual)",
            Objects::PolycubePrimal => "polycube (primal)",
            Objects::MeshPolycubeLayout => "embedded layout",
            Objects::MeshInput => "input mesh",
            Objects::MeshAlignmentScore => "alignment (score)",
            Objects::MeshOrthogonalityScore => "orthogonality (score)",
            Objects::MeshPolycubeLabeling => "polycube labeling",
            Objects::PolycubePrimalPolyhedron => "polycube polyhedron",
            Objects::Debug => "DEBUG!!!",
            Objects::Debug2 => "DEBUG2!!!",
        }
        .to_owned()
    }
}

impl From<Objects> for Vec3 {
    fn from(val: Objects) -> Self {
        match val {
            Objects::MeshDualLoops => Self::new(0., 0., 0.),
            Objects::PolycubeDual => Self::new(1_000., 0., 0.),
            Objects::PolycubePrimal => Self::new(1_000., 1_000., 0.),
            Objects::MeshPolycubeLayout => Self::new(1_000., 1_000., 1_000.),
            Objects::MeshInput => Self::new(1_000., 0., 1_000.),
            Objects::MeshAlignmentScore => Self::new(0., 1_000., 0.),
            Objects::MeshOrthogonalityScore => Self::new(0., 1_000., 1_000.),
            Objects::MeshPolycubeLabeling => Self::new(-1_000., 0., 0.),
            Objects::PolycubePrimalPolyhedron => Self::new(-1_000., 1_000., 0.),
            Objects::Debug => Self::new(0_000., 0_000., 1_000.),
            Objects::Debug2 => Self::new(0_000., 0_000., -1_000.),
        }
    }
}

#[derive(Component, PartialEq, Eq, Hash, Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub struct CameraFor(pub Objects);

pub fn reset(commands: &mut Commands, cameras: &Query<Entity, With<Camera>>, images: &mut ResMut<Assets<Image>>, handles: &mut ResMut<CameraHandles>) {
    for camera in cameras.iter() {
        commands.entity(camera).despawn();
    }

    // Main camera. This is the camera that the user can control.
    commands
        .spawn(Camera3dBundle {
            tonemapping: Tonemapping::None,
            ..default()
        })
        .insert((OrbitCameraBundle::new(
            OrbitCameraController::default(),
            DEFAULT_CAMERA_EYE + Vec3::from(Objects::MeshDualLoops),
            DEFAULT_CAMERA_TARGET + Vec3::from(Objects::MeshDualLoops),
            Vec3::Y,
        ),))
        .insert(CameraFor(Objects::MeshDualLoops));

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
        if object == Objects::PolycubeDual || object == Objects::PolycubePrimal {
            commands.spawn((
                Camera3dBundle {
                    camera: Camera {
                        target: handle.into(),
                        ..Default::default()
                    },
                    projection: bevy::prelude::Projection::Orthographic(OrthographicProjection {
                        // 6 world units per window height.
                        scaling_mode: ScalingMode::FixedVertical(30.0),
                        ..default()
                    }),
                    tonemapping: Tonemapping::None,
                    ..default()
                },
                CameraFor(object),
            ));
        } else {
            commands.spawn((
                Camera3dBundle {
                    camera: Camera {
                        target: handle.into(),
                        ..Default::default()
                    },
                    tonemapping: Tonemapping::None,
                    ..default()
                },
                CameraFor(object),
            ));
        }
    }
}

pub fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut handles: ResMut<CameraHandles>,
    mut config_store: ResMut<GizmoConfigStore>,
    cameras: Query<Entity, With<Camera>>,
) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    config.line_width = 3.;

    self::reset(&mut commands, &cameras, &mut images, &mut handles);
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct MeshProperties {
    pub source: String,
    pub scale: f32,
    pub translation: Vector3D,
}

fn get_pbrbundle(mesh: Handle<Mesh>, translation: Vec3, scale: f32, material: &Handle<StandardMaterial>) -> PbrBundle {
    PbrBundle {
        mesh,
        transform: Transform {
            translation,
            rotation: Quat::IDENTITY,
            scale: Vec3::splat(scale),
        },
        material: material.clone(),
        ..default()
    }
}

fn get_mesh<VertID: Key, V: Default + HasPosition, EdgeID: Key, E: Default, FaceID: Key, F: Default>(
    dcel: &Douconel<VertID, V, EdgeID, E, FaceID, F>,
    color_map: &HashMap<FaceID, [f32; 3]>,
) -> (Mesh, Vec3, f32) {
    let mesh = dcel.bevy(color_map);
    let aabb = mesh.compute_aabb().unwrap();
    let scale = 10. * (1. / aabb.half_extents.max_element());
    let translation = (-scale * aabb.center).into();
    (mesh, translation, scale)
}

pub fn update(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,

    configuration: Res<Configuration>,
    rendered_mesh_query: Query<Entity, With<RenderedMesh>>,

    mut mesh_resmut: ResMut<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut gizmos_cache: ResMut<GizmosCache>,
    mut cameras: Query<(&mut Transform, &mut Projection, &CameraFor)>,
) {
    let main_transform = cameras.iter().find(|(_, _, camera_for)| camera_for.0 == Objects::MeshDualLoops).unwrap().0;
    let normalized_translation = main_transform.translation - Vec3::from(Objects::MeshDualLoops);
    let normalized_rotation = main_transform.rotation;

    let distance = normalized_translation.length();

    for (mut sub_transform, mut sub_projection, sub_object) in &mut cameras {
        sub_transform.translation = normalized_translation + Vec3::from(sub_object.0);
        sub_transform.rotation = normalized_rotation;
        if let Projection::Orthographic(orthographic) = sub_projection.as_mut() {
            orthographic.scaling_mode = ScalingMode::FixedVertical(distance);
        }
    }

    // The rest of this function function should only be called when the mesh (RenderedMesh or Solution) is changed.
    if !mesh_resmut.is_changed() && !solution.is_changed() {
        return;
    }
    info!("InputResource or SolutionResource change has been detected. Updating all objects and gizmos.");

    for entity in rendered_mesh_query.iter() {
        commands.entity(entity).despawn();
    }
    info!("Objects despawned.");

    gizmos_cache.lines.clear();
    info!("Gizmos cache cleared.");

    if mesh_resmut.mesh.faces.is_empty() {
        warn!("Current mesh is empty.");
        return;
    }

    let standard_material = materials.add(StandardMaterial { unlit: true, ..default() });
    let background_material = materials.add(StandardMaterial {
        base_color: BACKGROUND_COLOR,
        unlit: true,
        ..default()
    });

    for object in all::<Objects>() {
        match object {
            Objects::MeshDualLoops => {
                let (mesh, translation, scale) = get_mesh(&(*mesh_resmut.mesh).clone(), &HashMap::new());
                mesh_resmut.properties.scale = scale;
                mesh_resmut.properties.translation = vec3_to_vector3d(translation);

                commands.spawn((
                    get_pbrbundle(meshes.add(mesh), translation + Vec3::from(Objects::MeshDualLoops), scale, &standard_material),
                    RenderedMesh,
                    MainMesh,
                ));

                if let Ok(dual) = &solution.current_solution.dual {
                    for segment_id in dual.loop_structure.edge_ids() {
                        let direction = dual.loop_structure.edges[segment_id].direction;
                        let side = dual.segment_to_side(segment_id, configuration.sides_mask);
                        let mut offset = Vector3D::new(0., 0., 0.);
                        let dist = 0.001 * f64::from(scale);
                        match side {
                            Side::Upper => match direction {
                                PrincipalDirection::X => offset[0] += dist,
                                PrincipalDirection::Y => offset[1] += dist,
                                PrincipalDirection::Z => offset[2] += dist,
                            },
                            Side::Lower => match direction {
                                PrincipalDirection::X => offset[0] -= dist,
                                PrincipalDirection::Y => offset[1] -= dist,
                                PrincipalDirection::Z => offset[2] -= dist,
                            },
                        };
                        let color = direction.to_dual_color_sided(side);
                        for edgepair in solution.current_solution.get_pairs_of_sequence(&dual.segment_to_edges(segment_id)) {
                            let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                            let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                            // let u = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[0]);
                            // let v = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[1]);

                            let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                offset + n * 0.01,
                                color,
                                translation + Vec3::from(Objects::MeshDualLoops),
                                scale,
                            );
                        }
                    }
                } else {
                    for loop_id in solution.current_solution.loops.keys() {
                        let color = solution.current_solution.loop_to_direction(loop_id).to_dual_color();
                        for edgepair in solution.current_solution.get_pairs_of_loop(loop_id) {
                            // let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                            // let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                            let u = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[0]);
                            let v = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[1]);

                            let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                n * 0.01,
                                color,
                                translation + Vec3::from(Objects::MeshDualLoops),
                                scale,
                            );
                        }
                    }
                }

                // for edge_id in mesh_resmut.mesh.edge_ids() {
                //     let (u_id, v_id) = mesh_resmut.mesh.endpoints(edge_id);
                //     let u = mesh_resmut.mesh.position(u_id);
                //     let v = mesh_resmut.mesh.position(v_id);
                //     let n = mesh_resmut.mesh.edge_normal(edge_id);
                //     add_line2(
                //         &mut gizmos_cache.lines,
                //         u,
                //         v,
                //         n * 0.005,
                //         hutspot::color::GRAY,
                //         translation + Vec3::from(object),
                //         scale,
                //     );
                // }
            }
            Objects::PolycubeDual => {
                if let Some(polycube) = &solution.current_solution.polycube.clone() {
                    let (mesh, translation, scale) = get_mesh(&polycube.structure.clone(), &HashMap::new());

                    commands.spawn((
                        get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                        RenderedMesh,
                    ));

                    // Draw all loop segments / faces axis aligned.
                    for &face_id in &polycube.structure.face_ids() {
                        let original_id = polycube.intersection_to_face.get_by_right(&face_id).unwrap();
                        let this_centroid = polycube.structure.centroid(face_id);

                        let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                        let orientation = to_principal_direction(normal).0;

                        for &neighbor_id in &polycube.structure.fneighbors(face_id) {
                            let next_original_id = polycube.intersection_to_face.get_by_right(&neighbor_id).unwrap();

                            let edge_between = polycube.structure.edge_between_faces(face_id, neighbor_id).unwrap().0;
                            let root = polycube.structure.root(edge_between);
                            let root_pos = polycube.structure.position(root);

                            let segment = solution
                                .current_solution
                                .dual
                                .as_ref()
                                .unwrap()
                                .loop_structure
                                .edge_between_verts(*original_id, *next_original_id)
                                .unwrap()
                                .0;

                            let direction = solution.current_solution.dual.as_ref().unwrap().loop_structure.edges[segment].direction;

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

                                let dist = 0.001 * f64::from(scale);

                                match side {
                                    Side::Upper => match direction {
                                        PrincipalDirection::X => offset[0] += dist,
                                        PrincipalDirection::Y => offset[1] += dist,
                                        PrincipalDirection::Z => offset[2] += dist,
                                    },
                                    Side::Lower => match direction {
                                        PrincipalDirection::X => offset[0] -= dist,
                                        PrincipalDirection::Y => offset[1] -= dist,
                                        PrincipalDirection::Z => offset[2] -= dist,
                                    },
                                };

                                let line = DrawableLine::from_line(
                                    this_centroid,
                                    direction_vector,
                                    offset + normal * 0.01,
                                    vec3_to_vector3d(translation + Vec3::from(object)),
                                    scale,
                                );
                                let c = direction.to_dual_color_sided(side);
                                gizmos_cache.lines.push((line.u, line.v, c));
                            }
                        }
                    }

                    // Draw the edges of the polycube.
                    for edge_id in polycube.structure.edge_ids() {
                        let endpoints = polycube.structure.endpoints(edge_id);
                        let u = polycube.structure.position(endpoints.0);
                        let v = polycube.structure.position(endpoints.1);
                        let line = DrawableLine::from_line(
                            u,
                            v,
                            polycube.structure.normal(polycube.structure.face(edge_id)) * 0.005,
                            vec3_to_vector3d(translation + Vec3::from(object)),
                            scale,
                        );
                        let c = hutspot::color::GRAY;
                        gizmos_cache.lines.push((line.u, line.v, c));
                    }
                }
            }
            Objects::PolycubePrimal => {
                if let Some(polycube) = &solution.current_solution.polycube.clone() {
                    let mut colormap = HashMap::new();
                    for &face_id in &polycube.structure.face_ids() {
                        let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                        let (dir, side) = to_principal_direction(normal);
                        let color = dir.to_primal_color_sided(side);
                        colormap.insert(face_id, color);
                    }

                    let (mesh, translation, scale) = get_mesh(&polycube.structure, &colormap);

                    commands.spawn((
                        get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                        RenderedMesh,
                    ));

                    // Draw the edges of the polycube.
                    for edge_id in polycube.structure.edge_ids() {
                        let endpoints = polycube.structure.endpoints(edge_id);
                        let u = polycube.structure.position(endpoints.0);
                        let v = polycube.structure.position(endpoints.1);
                        let line = DrawableLine::from_line(
                            u,
                            v,
                            polycube.structure.normal(polycube.structure.face(edge_id)) * 0.005,
                            vec3_to_vector3d(translation + Vec3::from(object)),
                            scale,
                        );
                        let c = hutspot::color::BLACK;
                        gizmos_cache.lines.push((line.u, line.v, c));
                    }
                }
            }
            Objects::MeshPolycubeLayout => {
                if let Some(polycube) = &solution.current_solution.polycube {
                    if let Some(Ok(lay)) = &solution.current_solution.layout {
                        let mut layout_color_map = HashMap::new();

                        for &face_id in &polycube.structure.face_ids() {
                            let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                            let (dir, side) = to_principal_direction(normal);
                            let color = dir.to_primal_color_sided(side);
                            for &triangle_id in &lay.face_to_patch[&face_id].faces {
                                layout_color_map.insert(triangle_id, color);
                            }
                        }

                        let (mesh, translation, scale) = get_mesh(&lay.granulated_mesh, &layout_color_map);

                        commands.spawn((
                            get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                            RenderedMesh,
                        ));

                        for path in lay.edge_to_path.values() {
                            for vertexpair in path.windows(2) {
                                if lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).is_none() {
                                    println!("Edge between {:?} and {:?} does not exist", vertexpair[0], vertexpair[1]);
                                    continue;
                                }
                                let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                                let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                                let u = lay.granulated_mesh.position(u_id);
                                let v = lay.granulated_mesh.position(v_id);
                                let n = lay.granulated_mesh.edge_normal(edge_id);
                                add_line2(
                                    &mut gizmos_cache.lines,
                                    u,
                                    v,
                                    n * 0.01,
                                    hutspot::color::BLACK,
                                    translation + Vec3::from(object),
                                    scale,
                                );
                            }
                        }
                    }
                }
            }
            Objects::MeshInput => {
                let (mesh, translation, scale) = get_mesh(&(*mesh_resmut.mesh).clone(), &HashMap::new());
                commands.spawn((
                    get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                    RenderedMesh,
                ));

                for edge_id in mesh_resmut.mesh.edge_ids() {
                    let (u_id, v_id) = mesh_resmut.mesh.endpoints(edge_id);
                    let u = mesh_resmut.mesh.position(u_id);
                    let v = mesh_resmut.mesh.position(v_id);
                    let n = mesh_resmut.mesh.edge_normal(edge_id);
                    add_line2(
                        &mut gizmos_cache.lines,
                        u,
                        v,
                        n * 0.005,
                        hutspot::color::WHITE,
                        translation + Vec3::from(object),
                        scale,
                    );
                }
            }
            Objects::MeshAlignmentScore => {
                if let Some(Ok(lay)) = &solution.current_solution.layout {
                    let mut layout_color_map = HashMap::new();

                    for &triangle_id in &lay.granulated_mesh.face_ids() {
                        let score = solution.current_solution.alignment_per_triangle[triangle_id];
                        let color = hutspot::color::map(score as f32, &hutspot::color::SCALE_MAGMA);
                        layout_color_map.insert(triangle_id, color);
                    }

                    let (mesh, translation, scale) = get_mesh(&lay.granulated_mesh, &layout_color_map);
                    commands.spawn((
                        get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                        RenderedMesh,
                    ));

                    for path in lay.edge_to_path.values() {
                        for vertexpair in path.windows(2) {
                            let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                            let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                            let u = lay.granulated_mesh.position(u_id);
                            let v = lay.granulated_mesh.position(v_id);
                            let n = lay.granulated_mesh.edge_normal(edge_id);
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                n * 0.005,
                                hutspot::color::BLACK,
                                translation + Vec3::from(object),
                                scale,
                            );
                        }
                    }
                }
            }
            Objects::MeshOrthogonalityScore => {
                if let Some(polycube) = &solution.current_solution.polycube {
                    if let Some(Ok(lay)) = &solution.current_solution.layout {
                        let mut layout_color_map = HashMap::new();

                        for &face_id in &polycube.structure.face_ids() {
                            let score = solution.current_solution.orthogonality_per_patch[face_id];
                            let color = hutspot::color::map(score as f32, &hutspot::color::SCALE_MAGMA);
                            for &triangle_id in &lay.face_to_patch[&face_id].faces {
                                layout_color_map.insert(triangle_id, color);
                            }
                        }

                        let (mesh, translation, scale) = get_mesh(&lay.granulated_mesh, &layout_color_map);
                        commands.spawn((
                            get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                            RenderedMesh,
                        ));

                        for path in lay.edge_to_path.values() {
                            for vertexpair in path.windows(2) {
                                let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                                let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                                let u = lay.granulated_mesh.position(u_id);
                                let v = lay.granulated_mesh.position(v_id);
                                let n = lay.granulated_mesh.edge_normal(edge_id);
                                add_line2(
                                    &mut gizmos_cache.lines,
                                    u,
                                    v,
                                    n * 0.005,
                                    hutspot::color::BLACK,
                                    translation + Vec3::from(object),
                                    scale,
                                );
                            }
                        }
                    }
                }
            }
            Objects::Debug => {
                if let Some(polycube) = &solution.current_solution.polycube {
                    if let Some(Ok(lay)) = &solution.current_solution.layout {
                        let mut layout_color_map = HashMap::new();

                        for &face_id in &polycube.structure.face_ids() {
                            let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                            let (dir, side) = to_principal_direction(normal);
                            let color = dir.to_primal_color_sided(side);
                            for &triangle_id in &lay.face_to_patch[&face_id].faces {
                                layout_color_map.insert(triangle_id, color);
                            }
                        }

                        let (mesh, translation, scale) = get_mesh(&lay.granulated_mesh, &layout_color_map);

                        commands.spawn((
                            get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                            RenderedMesh,
                        ));

                        for path_id in lay.edge_to_path.keys() {
                            // Calculate alpha for each vertex in the path.
                            // A path is geodesic if alpha is > 180 degrees (pi) for each vertex
                            // Any vertex with alpha < 180 degrees is a candidate for smoothing
                            let mut wedges = priority_queue::PriorityQueue::new();

                            // Every vertex is defined by a wedge of 3 vertices
                            let path = lay.edge_to_path.get(&path_id).unwrap();
                            for pair in path.windows(3) {
                                // First we find the correct side (shortest angle/alpha) of the wedge
                                // There are two wedges between a, b, c
                                let (a, b, c) = (pair[0], pair[1], pair[2]);
                                let (wedge1, wedge2) = lay.granulated_mesh.wedges(a, b, c);

                                if lay.granulated_mesh.distance(a, b) < 1e-6 || lay.granulated_mesh.distance(b, c) < 1e-6 {
                                    continue;
                                }

                                let alpha_wedge1 = wedge1
                                    .clone()
                                    .into_iter()
                                    .tuple_windows()
                                    .map(|(v1, v2)| lay.granulated_mesh.vertex_angle(v1, b, v2))
                                    .sum::<f64>();
                                let alpha_wedge2 = wedge2
                                    .clone()
                                    .into_iter()
                                    .tuple_windows()
                                    .map(|(v1, v2)| lay.granulated_mesh.vertex_angle(v1, b, v2))
                                    .sum::<f64>();

                                if alpha_wedge1 < alpha_wedge2 {
                                    wedges.push((wedge1, b, false), Reverse(OrderedFloat(alpha_wedge1)));
                                } else {
                                    wedges.push((wedge2, b, true), Reverse(OrderedFloat(alpha_wedge2)));
                                }
                            }

                            let color = if let Some(worst_wedge) = wedges.peek() {
                                println!("Worst wedge: {:?}", worst_wedge);

                                if worst_wedge.1 .0.abs() < PI * 0.9 {
                                    add_line(
                                        &mut gizmos_cache.lines,
                                        lay.granulated_mesh.position(worst_wedge.0 .1),
                                        lay.granulated_mesh.vert_normal(worst_wedge.0 .1),
                                        0.1,
                                        hutspot::color::RED,
                                        translation + Vec3::from(object),
                                        scale,
                                    );

                                    hutspot::color::ORANGE
                                } else {
                                    hutspot::color::BLACK
                                }
                            } else {
                                hutspot::color::BLACK
                            };

                            for vertexpair in path.windows(2) {
                                if lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).is_none() {
                                    println!("Edge between {:?} and {:?} does not exist", vertexpair[0], vertexpair[1]);
                                    continue;
                                }
                                let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                                let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                                let u = lay.granulated_mesh.position(u_id);
                                let v = lay.granulated_mesh.position(v_id);
                                let n = lay.granulated_mesh.edge_normal(edge_id);
                                add_line2(&mut gizmos_cache.lines, u, v, n * 0.01, color, translation + Vec3::from(object), scale);
                            }

                            if color == hutspot::color::ORANGE {
                                break;
                            }
                        }

                        // for path in lay.edge_to_path.values() {
                        //     for vertexpair in path.windows(2) {
                        //         if lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).is_none() {
                        //             println!("Edge between {:?} and {:?} does not exist", vertexpair[0], vertexpair[1]);
                        //             continue;
                        //         }
                        //         let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                        //         let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                        //         let u = lay.granulated_mesh.position(u_id);
                        //         let v = lay.granulated_mesh.position(v_id);
                        //         let n = lay.granulated_mesh.edge_normal(edge_id);
                        //         add_line2(
                        //             &mut gizmos_cache.lines,
                        //             u,
                        //             v,
                        //             n * 0.01,
                        //             hutspot::color::BLACK,
                        //             translation + Vec3::from(object),
                        //             scale,
                        //         );
                        //     }
                        // }

                        for edge_id in lay.granulated_mesh.edge_ids() {
                            let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                            let u = lay.granulated_mesh.position(u_id);
                            let v = lay.granulated_mesh.position(v_id);
                            let n = lay.granulated_mesh.edge_normal(edge_id);
                            add_line2(
                                &mut gizmos_cache.lines,
                                u,
                                v,
                                n * 0.005,
                                hutspot::color::LIGHT_GRAY,
                                translation + Vec3::from(object),
                                scale,
                            );
                        }

                        // for edge_id in mesh_resmut.mesh.edge_ids() {
                        //     let (u_id, v_id) = mesh_resmut.mesh.endpoints(edge_id);
                        //     let u = mesh_resmut.mesh.position(u_id);
                        //     let v = mesh_resmut.mesh.position(v_id);
                        //     let n = mesh_resmut.mesh.edge_normal(edge_id);
                        //     add_line2(
                        //         &mut gizmos_cache.lines,
                        //         u,
                        //         v,
                        //         n * 0.008,
                        //         hutspot::color::LIGHT_GRAY,
                        //         translation + Vec3::from(object),
                        //         scale,
                        //     );
                        // }
                    }
                }
            }
            Objects::Debug2 => {
                let mut color_map = HashMap::new();

                if let Ok(dual) = &solution.current_solution.dual {
                    for (zone_id, zone) in &dual.zones {
                        let dir = zone.direction;
                        if dir != configuration.direction {
                            continue;
                        }
                        let color = dir.to_dual_color();
                        let rand1 = (rand::random::<f32>() - 0.5) / 10.;
                        let rand2 = (rand::random::<f32>() - 0.5) / 10.;
                        let rand3 = (rand::random::<f32>() - 0.5) / 10.;
                        let rand_color = [color[0] + rand1, color[1] + rand2, color[2] + rand3];

                        for &region_id in &zone.regions {
                            let region = &dual.loop_structure.faces[region_id];
                            let verts_in_region = &region.verts;

                            for vert_id in verts_in_region {
                                for face_id in mesh_resmut.mesh.star(*vert_id) {
                                    color_map.insert(face_id, rand_color);
                                }
                            }
                        }
                    }
                }

                let (mesh, translation, scale) = get_mesh(&(*mesh_resmut.mesh).clone(), &color_map);
                mesh_resmut.properties.scale = scale;
                mesh_resmut.properties.translation = vec3_to_vector3d(translation);

                commands.spawn((
                    get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                    RenderedMesh,
                ));

                if let Ok(dual) = &solution.current_solution.dual {
                    for segment_id in dual.loop_structure.edge_ids() {
                        let direction = dual.loop_structure.edges[segment_id].direction;
                        let side = dual.segment_to_side(segment_id, configuration.sides_mask);
                        let mut offset = Vector3D::new(0., 0., 0.);
                        let dist = 0.001 * f64::from(scale);
                        match side {
                            Side::Upper => match direction {
                                PrincipalDirection::X => offset[0] += dist,
                                PrincipalDirection::Y => offset[1] += dist,
                                PrincipalDirection::Z => offset[2] += dist,
                            },
                            Side::Lower => match direction {
                                PrincipalDirection::X => offset[0] -= dist,
                                PrincipalDirection::Y => offset[1] -= dist,
                                PrincipalDirection::Z => offset[2] -= dist,
                            },
                        };
                        let color = direction.to_dual_color_sided(side);
                        for edgepair in solution.current_solution.get_pairs_of_sequence(&dual.segment_to_edges(segment_id)) {
                            let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                            let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                            // let u = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[0]);
                            // let v = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[1]);

                            let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                            add_line2(&mut gizmos_cache.lines, u, v, offset + n * 0.01, color, translation + Vec3::from(object), scale);
                        }
                    }
                } else {
                    for loop_id in solution.current_solution.loops.keys() {
                        let color = solution.current_solution.loop_to_direction(loop_id).to_dual_color();
                        for edgepair in solution.current_solution.get_pairs_of_loop(loop_id) {
                            // let u = mesh_resmut.mesh.midpoint(edgepair[0]);
                            // let v = mesh_resmut.mesh.midpoint(edgepair[1]);
                            let u = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[0]);
                            let v = solution.current_solution.get_coordinates_of_loop_in_edge(loop_id, edgepair[1]);

                            let n = mesh_resmut.mesh.edge_normal(edgepair[0]);
                            add_line2(&mut gizmos_cache.lines, u, v, n * 0.005, color, translation + Vec3::from(object), scale);
                        }
                    }
                }
            }
            Objects::MeshPolycubeLabeling => {
                if let Some(polycube) = &solution.current_solution.polycube {
                    if let Some(Ok(lay)) = &solution.current_solution.layout {
                        let mut layout_color_map = HashMap::new();

                        for &face_id in &polycube.structure.face_ids() {
                            let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                            let (dir, side) = to_principal_direction(normal);
                            let color = dir.to_primal_color_sided(side);
                            for &triangle_id in &lay.face_to_patch[&face_id].faces {
                                layout_color_map.insert(triangle_id, color);
                            }
                        }

                        let (mesh, translation, scale) = get_mesh(&lay.granulated_mesh, &layout_color_map);

                        commands.spawn((
                            get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                            RenderedMesh,
                        ));

                        let path_ids = lay
                            .edge_to_path
                            .keys()
                            .filter(|&&path_id| {
                                let twin_id = lay.polycube_ref.structure.twin(path_id).to_owned();
                                path_id < twin_id
                            })
                            .copied()
                            .collect_vec();

                        for path_id in path_ids {
                            let path = lay.edge_to_path.get(&path_id).unwrap();

                            let face_id = lay.polycube_ref.structure.face(path_id);

                            let twin_path_id = lay.polycube_ref.structure.twin(path_id).to_owned();
                            let twin_face_id = lay.polycube_ref.structure.face(twin_path_id);

                            let normal = lay.polycube_ref.structure.normal(face_id).normalize();
                            let twin_normal = lay.polycube_ref.structure.normal(twin_face_id).normalize();
                            if normal != twin_normal {
                                for vertexpair in path.windows(2) {
                                    if lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).is_none() {
                                        println!("Edge between {:?} and {:?} does not exist", vertexpair[0], vertexpair[1]);
                                        continue;
                                    }
                                    let edge_id = lay.granulated_mesh.edge_between_verts(vertexpair[0], vertexpair[1]).unwrap().0;
                                    let (u_id, v_id) = lay.granulated_mesh.endpoints(edge_id);
                                    let u = lay.granulated_mesh.position(u_id);
                                    let v = lay.granulated_mesh.position(v_id);
                                    let n = lay.granulated_mesh.edge_normal(edge_id);
                                    add_line2(
                                        &mut gizmos_cache.lines,
                                        u,
                                        v,
                                        n * 0.01,
                                        hutspot::color::BLACK,
                                        translation + Vec3::from(object),
                                        scale,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            Objects::PolycubePrimalPolyhedron => {
                if let Some(polycube) = &solution.current_solution.polycube.clone() {
                    let mut colormap = HashMap::new();
                    for &face_id in &polycube.structure.face_ids() {
                        let normal = (polycube.structure.normal(face_id) as Vector3D).normalize();
                        let (dir, side) = to_principal_direction(normal);
                        let color = dir.to_primal_color_sided(side);
                        colormap.insert(face_id, color);
                    }

                    let (mesh, translation, scale) = get_mesh(&polycube.structure, &colormap);

                    commands.spawn((
                        get_pbrbundle(meshes.add(mesh), translation + Vec3::from(object), scale, &standard_material),
                        RenderedMesh,
                    ));

                    // Draw the edges of the polycube.
                    for edge_id in polycube.structure.edge_ids() {
                        let face1 = polycube.structure.face(edge_id);
                        let face2 = polycube.structure.face(polycube.structure.twin(edge_id));
                        let normal1 = polycube.structure.normal(face1);
                        let normal2 = polycube.structure.normal(face2);
                        if normal1 != normal2 {
                            let endpoints = polycube.structure.endpoints(edge_id);
                            let u = polycube.structure.position(endpoints.0);
                            let v = polycube.structure.position(endpoints.1);
                            let line = DrawableLine::from_line(
                                u,
                                v,
                                polycube.structure.normal(polycube.structure.face(edge_id)) * 0.005,
                                vec3_to_vector3d(translation + Vec3::from(object)),
                                scale,
                            );
                            let c = hutspot::color::BLACK;
                            gizmos_cache.lines.push((line.u, line.v, c));
                        }
                    }
                }
            }
        }

        // Spawning covers such that the objects are view-blocked.
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(400.)),
                transform: Transform {
                    translation: Vec3::from(object),
                    ..default()
                },
                material: background_material.clone(),
                ..default()
            },
            RenderedMesh,
        ));
    }
}

#[derive(Default, Resource)]
pub struct GizmosCache {
    pub lines: Vec<Line>,
    pub raycaster: Vec<Line>,
}

type Line = (Vec3, Vec3, hutspot::color::Color);

// Draws the gizmos. This includes all wireframes, vertices, normals, raycasts, etc.
pub fn gizmos(mut gizmos: Gizmos, gizmos_cache: Res<GizmosCache>, configuration: Res<Configuration>) {
    for &(u, v, c) in &gizmos_cache.lines {
        gizmos.line(u, v, Color::srgb(c[0], c[1], c[2]));
    }

    if configuration.interactive {
        for &(u, v, c) in &gizmos_cache.raycaster {
            gizmos.line(u, v, Color::srgb(c[0], c[1], c[2]));
        }
    }
}

pub fn add_line(lines: &mut Vec<Line>, position: Vector3D, normal: Vector3D, length: f32, color: hutspot::color::Color, translation: Vec3, scale: f32) {
    let line = DrawableLine::from_vertex(position, normal, length, vec3_to_vector3d(translation), scale);
    lines.push((line.u, line.v, color));
}

pub fn add_line2(
    lines: &mut Vec<Line>,
    position_a: Vector3D,
    position_b: Vector3D,
    offset: Vector3D,
    color: hutspot::color::Color,
    translation: Vec3,
    scale: f32,
) {
    let line = DrawableLine::from_line(position_a, position_b, offset, vec3_to_vector3d(translation), scale);
    lines.push((line.u, line.v, color));
}
