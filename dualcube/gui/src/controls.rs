use crate::colors;
use crate::jobs::{Job, JobRequest};
use crate::render::gizmos::{vector3d_to_vec3, view_to_world, world_to_view, PerpetualGizmos};
use crate::render::store::MainMesh;
use crate::resources::{Configuration, InputResource, SolutionResource};
use bevy::picking::backend::ray::RayMap;
use bevy::prelude::*;
use dualcube::prelude::*;
use dualcube::solutions::Loop;
use itertools::Itertools;
use mehsh::prelude::*;
use ordered_float::OrderedFloat;
use std::collections::HashMap;

/// Registers the interactive mouse/keyboard controls.
pub struct ControlsPlugin;

impl Plugin for ControlsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CacheResource>()
            .add_systems(Update, system);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InteractiveMode {
    None,
    LoopModification,
    SegmentationModification,
}

/// Cached interactive loop preview.
#[derive(Default, Resource)]
pub struct CacheResource {
    pub cache: [HashMap<[EdgeID; 2], Vec<([EdgeID; 2], OrderedFloat<f64>)>>; 3],
    pub loop_preview_key: Option<LoopPreviewKey>,
    pub loop_preview: Option<(Vec<EdgeID>, f64)>,
    pub loop_preview_segments: Vec<(Vec<EdgeID>, f64)>,
    pub locked_loop_segments: Vec<(Vec<EdgeID>, f64)>,
    pub locked_loop_direction: Option<PrincipalDirection>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoopPreviewKey {
    pub direction: PrincipalDirection,
    pub hover: [EdgeID; 2],
    pub anchors: Vec<[EdgeID; 2]>,
}

pub fn segmentation_modification_system(
    mouse: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos: Gizmos<PerpetualGizmos>,
    mut configuration: ResMut<Configuration>,
    mut jobs: MessageWriter<JobRequest>,
    position: Vector3D,
    nearest_face: FaceID,
) -> Result<(), BevyError> {
    if mesh_resmut.mesh.nr_verts() == 0 {
        return Ok(());
    }

    if let Some(layout) = &solution.current_solution.layout {
        let granulated_vert_lookup = layout.granulated_mesh.kdtree();
        let nearest_granulated_vert = granulated_vert_lookup.nearest(&position.into()).1;

        // Look for nearest segmentation corner
        let modification = if (solution.selected_corner).is_none() {
            let (current_polycube_corner, current_segmentation_corner) =
                layout
                    .vert_to_corner
                    .iter()
                    .min_by_key(|(_, &corner)| {
                        OrderedFloat(layout.granulated_mesh.position(corner).metric_distance(
                            &layout.granulated_mesh.position(nearest_granulated_vert),
                        ))
                    })
                    .map(|(&poly_vert, &seg_vert)| (poly_vert, seg_vert))
                    .unwrap();

            // Highlight this vertex
            let v = layout.granulated_mesh.position(current_segmentation_corner);
            let v_transformed = world_to_view(
                v,
                mesh_resmut.properties.translation,
                mesh_resmut.properties.scale,
            );
            let n = vector3d_to_vec3(layout.granulated_mesh.normal(current_segmentation_corner));

            let isometry = Isometry3d::new(
                v_transformed,
                Quat::from_rotation_arc(Vec3::Z, n.normalize()),
            );
            gizmos.line(
                v_transformed,
                v_transformed + n,
                colors::to_bevy(colors::DARK_GRAY),
            );
            gizmos.circle(isometry, 0.2, colors::to_bevy(colors::DARK_GRAY));

            Some(current_polycube_corner)
        } else {
            None
        };

        if let Some(corner_poly) = solution.selected_corner {
            // Highlight selected corner
            let corner_poly_vert = layout
                .vert_to_corner
                .get_by_left(&corner_poly)
                .unwrap()
                .to_owned();
            let v = layout.granulated_mesh.position(corner_poly_vert);
            let v_transformed = world_to_view(
                v,
                mesh_resmut.properties.translation,
                mesh_resmut.properties.scale,
            );
            let n = vector3d_to_vec3(layout.granulated_mesh.normal(corner_poly_vert));

            let isometry = Isometry3d::new(
                v_transformed,
                Quat::from_rotation_arc(Vec3::Z, n.normalize()),
            );
            gizmos.line(
                v_transformed,
                v_transformed + n,
                colors::to_bevy(colors::BLACK),
            );
            gizmos.circle(isometry, 0.1, colors::to_bevy(colors::BLACK));

            // Highlight the current position (where the vertex would be moved)
            let v1 = layout.granulated_mesh.position(nearest_granulated_vert);
            let v1_transformed = world_to_view(
                v1,
                mesh_resmut.properties.translation,
                mesh_resmut.properties.scale,
            );
            let n1 = vector3d_to_vec3(layout.granulated_mesh.normal(nearest_granulated_vert));

            let isometry1 = Isometry3d::new(
                v1_transformed,
                Quat::from_rotation_arc(Vec3::Z, n1.normalize()),
            );
            gizmos.line(
                v1_transformed,
                v1_transformed + n1,
                colors::to_bevy(colors::BLACK),
            );
            gizmos.circle(isometry1, 0.1, colors::to_bevy(colors::BLACK));

            gizmos.arrow(
                v_transformed + 0.2 * n,
                v1_transformed + 0.1 * n1,
                colors::to_bevy(colors::BLACK),
            );
        }

        // CONTROLS
        //
        // Action1:  Select segmentation corner to be moved (ALT+LMB)
        // Action2:  Move segmentation corner to new location (LMB) (if a corner is selected)
        // Action3:  Deselect segmentation corner (ESCAPE)

        let esc = keyboard.pressed(KeyCode::Escape);
        let lmb = mouse.pressed(MouseButton::Left);
        let alt = keyboard.pressed(KeyCode::AltLeft);

        // Controls
        match (lmb, alt, esc) {
            // Action1:
            (true, true, false) => {
                if let Some(corner_poly) = modification {
                    solution.selected_corner = Some(corner_poly);
                }
            }
            // Action2:
            (true, false, false) => {
                if let Some(corner_poly) = solution.selected_corner {
                    jobs.write(JobRequest::Run(Job::move_corner(
                        solution.current_solution.clone(),
                        configuration.clone(),
                        corner_poly,
                        nearest_granulated_vert,
                    )));
                    solution.selected_corner = None;
                }
            }
            // Action3:
            (_, _, true) => {
                solution.selected_corner = None;
            }
            _ => {}
        }
    }

    Ok(())
}

pub fn loop_modification_system(
    mouse: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos: Gizmos<PerpetualGizmos>,
    mut configuration: ResMut<Configuration>,
    mut jobs: MessageWriter<JobRequest>,

    position: Vector3D,
    nearest_face: FaceID,
) -> Result<(), BevyError> {
    if mesh_resmut.mesh.nr_verts() == 0 {
        return Ok(());
    }

    // Get the nearest vertex in the hovered face and use its opposite edge pair
    // as the interactive loop anchor/seed.
    let nearest_vert = mesh_resmut
        .mesh
        .vertices(nearest_face)
        .min_by_key(|&v| OrderedFloat(position.metric_distance(&mesh_resmut.mesh.position(v))))
        .unwrap()
        .to_owned();

    let edgepair = mesh_resmut
        .mesh
        .edges_in_face_with_vert(nearest_face, nearest_vert)
        .unwrap();

    solution.current_solution.prepare_flow();

    let direction = configuration.direction;
    if cache
        .locked_loop_direction
        .is_some_and(|locked| locked != direction)
    {
        cache.locked_loop_segments.clear();
        cache.locked_loop_direction = None;
        cache.loop_preview = None;
        cache.loop_preview_key = None;
        cache.loop_preview_segments.clear();
    }
    let anchor_color = colors::to_bevy(colors::from_direction(
        direction,
        Some(Perspective::Dual),
        None,
    ));
    let preview_color = colors::from_direction(
        direction,
        Some(Perspective::Dual),
        Some(Orientation::Backwards),
    );

    // Draw already selected anchors.
    for &anchor in &configuration.loop_anchors {
        draw_edgepair_arrow(&mesh_resmut, &mut gizmos, anchor, anchor_color);
    }

    // Draw hovered anchor candidate.
    draw_edgepair_arrow(&mesh_resmut, &mut gizmos, edgepair, anchor_color);

    let mut preview_anchors = configuration.loop_anchors.clone();
    preview_anchors.push(edgepair);
    let preview_key = LoopPreviewKey {
        direction,
        hover: edgepair,
        anchors: configuration.loop_anchors.clone(),
    };

    if cache.loop_preview_key.as_ref() != Some(&preview_key) {
        if let Some((edges, cost, segments)) = solution
            .current_solution
            .construct_loop_with_anchors_and_locked_segments(
                &preview_anchors,
                direction,
                &cache.locked_loop_segments,
                |a: f64| OrderedFloat(a.powi(3)),
            )
        {
            cache.loop_preview = Some((edges, cost));
            cache.loop_preview_segments = segments;
        } else {
            cache.loop_preview = None;
            cache.loop_preview_segments.clear();
        }
        cache.loop_preview_key = Some(preview_key);
    }

    if let Some((edges, _)) = &cache.loop_preview {
        draw_loop_gradient(&mesh_resmut, &mut gizmos, edges, preview_color);
    }

    let lmb = mouse.just_pressed(MouseButton::Left);
    let enter = keyboard.just_pressed(KeyCode::Enter);
    let delete = keyboard.just_pressed(KeyCode::Delete);
    let force = keyboard.pressed(KeyCode::ShiftLeft);

    if keyboard.just_pressed(KeyCode::Escape) {
        configuration.loop_anchors.clear();
        cache.loop_preview = None;
        cache.loop_preview_key = None;
        cache.loop_preview_segments.clear();
        cache.locked_loop_segments.clear();
        cache.locked_loop_direction = None;
    }

    if lmb {
        configuration.loop_anchors.push(edgepair);
        cache.locked_loop_segments = cache.loop_preview_segments.clone();
        cache.locked_loop_segments.pop();
        cache.locked_loop_direction = Some(direction);
        cache.loop_preview = None;
        cache.loop_preview_key = None;
        cache.loop_preview_segments.clear();
        debug!("loop anchors now: {:?}", configuration.loop_anchors);
    }

    if enter {
        if let Some((edges, _)) = cache.loop_preview.clone() {
            solution
                .current_solution
                .add_loop(Loop { edges, direction });
            configuration.loop_anchors.clear();
            solution.next[0].clear();
            solution.next[1].clear();
            solution.next[2].clear();
            cache.cache[0].clear();
            cache.cache[1].clear();
            cache.cache[2].clear();
            cache.loop_preview = None;
            cache.loop_preview_key = None;
            cache.loop_preview_segments.clear();
            cache.locked_loop_segments.clear();
            cache.locked_loop_direction = None;
        }
    }

    if delete {
        let option_a = [edgepair[0], edgepair[1]];
        let option_b = [edgepair[1], edgepair[0]];

        if let Some(loop_id) = solution.current_solution.loops.keys().find(|&loop_id| {
            let edges = solution.current_solution.get_pairs_of_loop(loop_id);
            edges.contains(&option_a) || edges.contains(&option_b)
        }) {
            jobs.write(JobRequest::Run(Job::remove_loop(
                solution.current_solution.clone(),
                loop_id,
                force,
                configuration.clone(),
            )));
        }
    }

    Ok(())
}

fn draw_edgepair_arrow(
    mesh_resmut: &InputResource,
    gizmos: &mut Gizmos<PerpetualGizmos>,
    edgepair: [EdgeID; 2],
    color: bevy::prelude::Color,
) {
    let u = mesh_resmut.mesh.position(edgepair[0]);
    let v = mesh_resmut.mesh.position(edgepair[1]);
    gizmos.arrow(
        world_to_view(
            u,
            mesh_resmut.properties.translation,
            mesh_resmut.properties.scale,
        ),
        world_to_view(
            v,
            mesh_resmut.properties.translation,
            mesh_resmut.properties.scale,
        ),
        color,
    );
}

fn draw_loop_gradient(
    mesh_resmut: &InputResource,
    gizmos: &mut Gizmos<PerpetualGizmos>,
    edges: &[EdgeID],
    color: colors::Color,
) {
    if edges.len() < 2 {
        return;
    }

    let edge_pairs = Solution::cycled_windows(edges);
    let last = edge_pairs.len().saturating_sub(1).max(1) as f32;

    for (i, [from, to]) in edge_pairs.into_iter().enumerate() {
        let t = i as f32 / last;
        let alpha = 1.0 - t;
        let segment_color = bevy::color::Color::srgba(color[0], color[1], color[2], alpha);
        let u = mesh_resmut.mesh.position(from);
        let v = mesh_resmut.mesh.position(to);
        gizmos.line(
            world_to_view(
                u,
                mesh_resmut.properties.translation,
                mesh_resmut.properties.scale,
            ),
            world_to_view(
                v,
                mesh_resmut.properties.translation,
                mesh_resmut.properties.scale,
            ),
            segment_color,
        );
    }
}

pub fn system(
    ray_map: Res<RayMap>,
    mut ray_cast: MeshRayCast,
    foo_query: Query<(), With<MainMesh>>,
    mouse: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos: Gizmos<PerpetualGizmos>,
    mut configuration: ResMut<Configuration>,
    mut jobs: MessageWriter<JobRequest>,
) -> Result<(), BevyError> {
    configuration.raycasted = None;
    configuration.selected = None;

    if keyboard.pressed(KeyCode::ControlLeft) || mouse.pressed(MouseButton::Right) {
        return Ok(());
    }

    if configuration.interactive_mode == InteractiveMode::None {
        return Ok(());
    }

    // Only ray cast against entities with the `Foo` component.
    let filter = |entity| foo_query.contains(entity);
    let settings = MeshRayCastSettings::default().with_filter(&filter);

    let Some(&(_, intersection)) = ray_map
        .iter()
        .filter_map(|(_, ray)| {
            let (_, hit) = ray_cast.cast_ray(*ray, &settings).first()?;
            Some((*ray, hit.point))
        })
        .collect_vec()
        .first()
    else {
        return Ok(());
    };

    let position = view_to_world(
        intersection,
        mesh_resmut.properties.translation,
        mesh_resmut.properties.scale,
    );
    let nearest_face = mesh_resmut.triangle_lookup.nearest(&position.into());

    // Draw the ray !
    let isometry1 = Isometry3d::new(
        intersection,
        Quat::from_rotation_arc(
            Vec3::Z,
            vector3d_to_vec3(mesh_resmut.mesh.normal(nearest_face)).normalize(),
        ),
    );
    gizmos.circle(isometry1, 0.1, colors::to_bevy(colors::BLACK));

    match configuration.interactive_mode {
        InteractiveMode::None => Ok(()),
        InteractiveMode::LoopModification => loop_modification_system(
            mouse,
            keyboard,
            mesh_resmut,
            solution,
            cache,
            gizmos,
            configuration,
            jobs,
            position,
            nearest_face,
        ),
        InteractiveMode::SegmentationModification => segmentation_modification_system(
            mouse,
            keyboard,
            mesh_resmut,
            solution,
            cache,
            gizmos,
            configuration,
            jobs,
            position,
            nearest_face,
        ),
    }
}
