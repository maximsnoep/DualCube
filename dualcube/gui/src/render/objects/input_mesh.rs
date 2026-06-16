//! Render object for the input (triangle) mesh.

use super::super::gizmos::{
    flow_graph_gizmos, lambert_color_map, layout_path_gizmos, segmentation_color_map,
    uniform_color_map, world_to_view, DirectionalGizmos,
};
use super::super::store::RenderObject;
use crate::colors;
use bevy::prelude::*;
use dualcube::prelude::*;
use mehsh::prelude::*;
use std::collections::HashMap;

const PRINCIPAL_DIRECTIONS: [Direction; 3] = DIRECTIONS;

/// The INPUT MESH render object, it has:
/// - meshes with gray / black / lambert-shaded faces
/// - segmentation and alignment colorings (if a layout exists)
/// - wireframes (input and granulated mesh)
/// - the dual loops, layout paths, and vector fields
pub(in crate::render) fn build(solution: &Solution) -> Option<RenderObject> {
    let input = solution.mesh_ref.as_ref();
    let (scale, translation) = input.scale_translation();

    // The dual loops, per principal direction.
    let mut loops = DirectionalGizmos::default();
    for (lewp_id, lewp) in &solution.loops {
        let direction = solution.loop_to_direction(lewp_id);
        let positions = [lewp.edges.clone(), vec![lewp.edges[0]], vec![lewp.edges[1]]]
            .concat()
            .into_iter()
            .map(|edge_id| world_to_view(input.position(edge_id), translation, scale))
            .collect::<Vec<_>>();
        let c = colors::to_bevy(colors::from_direction(
            direction,
            Some(Perspective::Dual),
            Some(Sign::Positive),
        ));
        loops.get_mut(direction).linestrip(positions, c);
    }

    // Layout-dependent features.
    let empty_mesh = mehsh::prelude::Mesh::<INPUT>::default();
    let mut granulated_mesh = &empty_mesh;
    let mut granulated_mesh_gizmos = GizmoAsset::new();
    let mut gizmos_paths = GizmoAsset::new();
    let mut gizmos_flat_paths = GizmoAsset::new();
    let mut color_map_segmentation = HashMap::new();
    let mut color_map_alignment = HashMap::new();

    if let (Some(layout), Some(polycube)) = (&solution.layout, &solution.polycube) {
        granulated_mesh = &layout.granulated_mesh;
        granulated_mesh_gizmos = granulated_mesh.gizmos(colors::GRAY);
        (gizmos_paths, gizmos_flat_paths) =
            layout_path_gizmos(layout, polycube, granulated_mesh, translation, scale);
        color_map_segmentation = segmentation_color_map(layout, polycube);

        for triangle_id in granulated_mesh.face_ids() {
            let color = layout
                .alignment_per_triangle
                .get(&triangle_id)
                .map_or(colors::SNOEP_YELLOW, |&score| {
                    colors::map(score as f32, &colors::SCALE_MAGMA)
                });
            color_map_alignment.insert(triangle_id, color);
        }
    }

    // The vector fields, per principal direction.
    let mut field_gizmos = DirectionalGizmos::default();
    if let Some(fields) = &solution.fields {
        // Arrow length relative to the mesh bounding-box diagonal.
        let vert_ids = input.vert_ids();
        let field_scale = 0.015
            * vert_ids.first().map_or(1.0, |&first_id| {
                let first = input.position(first_id);
                let (mut min_p, mut max_p) = (first, first);
                for &vert_id in &vert_ids {
                    let p = input.position(vert_id);
                    min_p = Vector3D::new(min_p.x.min(p.x), min_p.y.min(p.y), min_p.z.min(p.z));
                    max_p = Vector3D::new(max_p.x.max(p.x), max_p.y.max(p.y), max_p.z.max(p.z));
                }
                (max_p - min_p).norm().max(1e-12)
            });

        for (field, dir) in [&fields.field_x, &fields.field_y, &fields.field_z]
            .into_iter()
            .zip(PRINCIPAL_DIRECTIONS)
        {
            let color = colors::to_bevy(colors::from_direction(dir, None, None));
            let gizmos = field_gizmos.get_mut(dir);
            for (&vert_id, &vector_id) in &field.map {
                let Some(v) = field.vectors.get(vector_id) else {
                    continue;
                };
                if v.norm() <= 1e-12 {
                    continue;
                }
                let p = input.position(vert_id);
                gizmos.arrow(
                    world_to_view(p, translation, scale),
                    world_to_view(p + *v * field_scale, translation, scale),
                    color,
                );
            }
        }
    }

    // The flow graphs, per principal direction: edge-to-edge transition weights.
    // Bright = low weight (good), fading to black/transparent = high weight (bad).
    let mut flow_graph_dir = DirectionalGizmos::default();
    if let Some(flow_graphs) = &solution.flow_graphs {
        for (graph, dir) in flow_graphs.iter().zip(PRINCIPAL_DIRECTIONS) {
            let color = colors::from_direction(dir, None, None);
            *flow_graph_dir.get_mut(dir) =
                flow_graph_gizmos(&graph.edges(), input, color, translation, scale, 100.0);
        }
    }

    Some(
        RenderObject::default()
            .mesh(input, &uniform_color_map(input, colors::LIGHT_GRAY), "gray")
            .mesh(input, &uniform_color_map(input, colors::BLACK), "black")
            .mesh(input, &lambert_color_map(input), "lambert")
            .mesh(granulated_mesh, &color_map_segmentation, "segmentation")
            .mesh(granulated_mesh, &color_map_alignment, "alignment")
            .gizmo(input.gizmos(colors::GRAY), 0.5, -0.00001, "wireframe")
            .gizmo(loops.x, 3., -0.0001, "x-loops")
            .gizmo(loops.y, 3., -0.00011, "y-loops")
            .gizmo(loops.z, 3., -0.000111, "z-loops")
            .gizmo(gizmos_paths, 3., -0.0001, "paths")
            .gizmo(gizmos_flat_paths, 1., -0.00011, "flat paths")
            .gizmo(granulated_mesh_gizmos, 0.5, -0.00001, "refined wireframe")
            .gizmo(field_gizmos.x, 1., -0.0010, "x-field")
            .gizmo(field_gizmos.y, 1., -0.0011, "y-field")
            .gizmo(field_gizmos.z, 1., -0.0012, "z-field")
            .gizmo(flow_graph_dir.x, 1., -0.0001, "x-flow-graph")
            .gizmo(flow_graph_dir.y, 1., -0.00011, "y-flow-graph")
            .gizmo(flow_graph_dir.z, 1., -0.000111, "z-flow-graph")
            .to_owned(),
    )
}
