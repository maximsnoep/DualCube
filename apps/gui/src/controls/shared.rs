use crate::colors;
use crate::render::gizmos::{PerpetualGizmos, world_to_view};
use crate::resources::InputResource;
use bevy::prelude::*;
use dualcube::prelude::*;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InteractiveMode {
    None,
    LoopModification,
    SegmentationModification,
}

/// Cached interactive control previews.
#[derive(Default, Resource)]
pub struct CacheResource {
    pub cache: [HashMap<[EdgeID; 2], Vec<([EdgeID; 2], OrderedFloat<f64>)>>; 3],
    pub loop_preview_key: Option<LoopPreviewKey>,
    pub loop_preview: Option<(Vec<EdgeID>, f64)>,
    pub loop_preview_segments: Vec<(Vec<EdgeID>, f64)>,
    pub locked_loop_segments: Vec<(Vec<EdgeID>, f64)>,
    pub locked_loop_direction: Option<Direction>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoopPreviewKey {
    pub direction: Direction,
    pub hover: [EdgeID; 2],
    pub anchors: Vec<[EdgeID; 2]>,
}

#[allow(unused_qualifications)]
pub fn draw_edgepair_arrow(
    mesh_resmut: &InputResource,
    gizmos: &mut Gizmos<'_, '_, PerpetualGizmos>,
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

#[allow(unused_qualifications)]
pub fn draw_loop_gradient(
    mesh_resmut: &InputResource,
    gizmos: &mut Gizmos<'_, '_, PerpetualGizmos>,
    edges: &[EdgeID],
    color: colors::Kolor,
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
