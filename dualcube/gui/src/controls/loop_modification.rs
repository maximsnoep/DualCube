use super::shared::{draw_edgepair_arrow, draw_loop_gradient, CacheResource, LoopPreviewKey};
use crate::colors;
use crate::jobs::Job;
use crate::render::gizmos::PerpetualGizmos;
use crate::resources::{Configuration, InputResource, SolutionResource};
use bevy::prelude::*;
use dualcube::prelude::*;
use mehsh::prelude::*;
use ordered_float::OrderedFloat;

pub fn loop_modification_system(
    mouse: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mesh_resmut: Res<InputResource>,
    mut solution: ResMut<SolutionResource>,
    mut cache: ResMut<CacheResource>,
    mut gizmos: Gizmos<PerpetualGizmos>,
    mut configuration: ResMut<Configuration>,
    mut jobs: MessageWriter<Job>,
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
    let preview_color =
        colors::from_direction(direction, Some(Perspective::Dual), Some(Sign::Negative));

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
            jobs.write(Job::add_loop(
                solution.current_solution.clone(),
                Loop { edges, direction },
                configuration.clone(),
            ));
        }
    }

    if delete {
        let option_a = [edgepair[0], edgepair[1]];
        let option_b = [edgepair[1], edgepair[0]];

        if let Some(loop_id) = solution.current_solution.loops.keys().find(|&loop_id| {
            let edges = solution.current_solution.get_pairs_of_loop(loop_id);
            edges.contains(&option_a) || edges.contains(&option_b)
        }) {
            jobs.write(Job::remove_loop(
                solution.current_solution.clone(),
                loop_id,
                force,
                configuration.clone(),
            ));
        }
    }

    Ok(())
}
