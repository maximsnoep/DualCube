//! Render object for the input (triangle) mesh.

use super::gizmos::{
    edge_endpoints_view, lambert_color_map, layout_path_gizmos, segmentation_color_map,
    uniform_color_map, world_to_view, DirectionalGizmos,
};
use super::store::RenderObject;
use crate::colors;
use bevy::prelude::*;
use dualcube::prelude::*;
use mehsh::prelude::*;
use std::collections::HashMap;

const PRINCIPAL_DIRECTIONS: [PrincipalDirection; 3] = [
    PrincipalDirection::X,
    PrincipalDirection::Y,
    PrincipalDirection::Z,
];

/// The INPUT MESH render object, it has:
/// - meshes with gray / black / lambert-shaded faces
/// - segmentation and alignment colorings (if a layout exists)
/// - wireframes (input and granulated mesh)
/// - the dual loops, layout paths, vector fields, feature edges,
///   principal-curvature glyphs, and elastica polylines
pub(super) fn build(solution: &Solution) -> Option<RenderObject> {
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
            Some(Orientation::Forwards),
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

    // Sharp feature edges, colored per principal direction.
    let features = dualcube::feature::feature_extraction(input, std::f64::consts::FRAC_PI_3, 1);
    let mut gizmos_features = GizmoAsset::new();
    for (feature_edges, dir) in features.iter().zip(PRINCIPAL_DIRECTIONS) {
        let color = colors::to_bevy(colors::from_direction(dir, None, None));
        for &edge_id in feature_edges {
            let (u, v) = edge_endpoints_view(input, edge_id, translation, scale);
            gizmos_features.line(u, v, color);
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
                    world_to_view(p + v.normalize() * field_scale, translation, scale),
                    color,
                );
            }
        }
    }

    // Principal curvature glyphs (direction + resolution-robust magnitude).
    let mut gizmos_curvature_max = GizmoAsset::new();
    let mut gizmos_curvature_min = GizmoAsset::new();
    let curvature_color = colors::to_bevy(colors::from_direction(
        PrincipalDirection::X,
        Some(Perspective::Dual),
        None,
    ));
    // Tunables for glyph sizing
    let s: f64 = 2.0; // sensitivity of length to curvature (dimensionless)
    let base_frac: f64 = 0.5; // glyph base length as fraction of local edge scale

    for vert_id in input.vert_ids() {
        let v = input.position(vert_id);

        // Local scale h(v): mean 1-ring edge length (resolution proxy).
        let mut h_sum = 0.0;
        let mut h_cnt = 0.0;
        for neighbor_id in input.neighbors(vert_id) {
            h_sum += (input.position(neighbor_id) - v).norm();
            h_cnt += 1.0;
        }
        if h_cnt < 1.0 {
            continue;
        }
        let h = (h_sum / h_cnt).max(1e-9);

        let (k_min, k_max, dir_min, dir_max) =
            dualcube::elastica::estimate_vertex_principal_frame(input, vert_id);

        // Sanity checks (debug-friendly).
        let n = input.tangent_frame(vert_id).2.normalize();
        debug_assert!((dir_max.norm() - 1.0).abs() < 1e-5);
        debug_assert!((dir_min.norm() - 1.0).abs() < 1e-5);
        debug_assert!(dir_max.dot(&n).abs() < 1e-6);
        debug_assert!(dir_min.dot(&n).abs() < 1e-6);
        debug_assert!(dir_max.dot(&dir_min).abs() < 1e-5);

        let v_view = world_to_view(v, translation, scale);
        for (k, dir, gizmo) in [
            (k_max, dir_max, &mut gizmos_curvature_max),
            (k_min, dir_min, &mut gizmos_curvature_min),
        ] {
            // Curvature k has units 1/length, so |k| * h is a dimensionless
            // "bending per step", mapped to a saturating glyph length in world
            // units that is tied to the local resolution.
            let len = base_frac * h * (s * (k.abs() * h)).tanh();
            for endpoint in [v + dir * len, v - dir * len] {
                gizmo.line(
                    v_view,
                    world_to_view(endpoint, translation, scale),
                    curvature_color,
                );
            }
        }
    }

    // Elastica derivative polylines, per principal direction.
    let mut elastica_gizmos = DirectionalGizmos::default();
    for dir in PRINCIPAL_DIRECTIONS {
        let polylines = solution.elastica_graph.derivative_edge_polylines(dir);
        let max_weight = polylines
            .iter()
            .map(|&(_, _, _, weight)| weight)
            .fold(0.0_f64, f64::max);
        debug!("Max elastica weight for {dir}: {max_weight}");

        let c = colors::to_bevy(colors::from_direction(dir, None, None));
        let gizmos = elastica_gizmos.get_mut(dir);
        for &(v0, v1, v2, weight) in &polylines {
            if weight > 0.2 {
                continue;
            }
            let [p0, p1, p2] =
                [v0, v1, v2].map(|v| world_to_view(input.position(v), translation, scale));
            gizmos.line(p0, p1, c);
            gizmos.line(p1, p2, c);
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
            .gizmo(field_gizmos.x, 1., -0.0001, "x-field")
            .gizmo(field_gizmos.y, 1., -0.00011, "y-field")
            .gizmo(field_gizmos.z, 1., -0.000111, "z-field")
            // .gizmo(gizmos_features, 1., -0.0001, "features")
            // .gizmo(gizmos_curvature_max, 1., -0.0001, "max curvature")
            // .gizmo(gizmos_curvature_min, 1., -0.00011, "min curvature")
            // .gizmo(elastica_gizmos.x, 1., -0.0001, "x-elastica")
            // .gizmo(elastica_gizmos.y, 1., -0.00011, "y-elastica")
            // .gizmo(elastica_gizmos.z, 1., -0.000111, "z-elastica")
            .to_owned(),
    )
}
