//! Render object for the polycube-map (the input mesh mapped onto the polycube).

use super::gizmos::{
    lambert_color_map, layout_path_gizmos, segmentation_color_map, uniform_color_map,
};
use super::store::RenderObject;
use crate::colors;
use dualcube::prelude::*;

/// The POLYCUBE-MAP render object, it has:
/// - meshes with black / gray / colored / lambert-shaded faces
/// - quads and triangles mapped on the polycube
/// - the layout paths mapped on the polycube (flat and non-flat)
pub(super) fn build(solution: &Solution) -> Option<RenderObject> {
    let (quad, layout, polycube) = (
        solution.quad.as_ref()?,
        solution.layout.as_ref()?,
        solution.polycube.as_ref()?,
    );

    let mesh = &quad.triangle_mesh_polycube;
    let (scale, translation) = mesh.scale_translation();
    let (gizmos_paths, gizmos_flat_paths) =
        layout_path_gizmos(layout, polycube, mesh, translation, scale);

    Some(
        RenderObject::default()
            .mesh(mesh, &uniform_color_map(mesh, colors::BLACK), "black")
            .mesh(mesh, &uniform_color_map(mesh, colors::LIGHT_GRAY), "gray")
            .mesh(mesh, &segmentation_color_map(layout, polycube), "colored")
            // Shading uses the normals of the (world-space) granulated mesh,
            // whose face ids correspond to the polycube-mapped triangle mesh.
            .mesh(mesh, &lambert_color_map(&layout.granulated_mesh), "lambert")
            .gizmo(
                quad.quad_mesh_polycube.gizmos(colors::GRAY),
                2.,
                -0.01,
                "quads",
            )
            .gizmo(mesh.gizmos(colors::GRAY), 2., -0.01, "triangles")
            .gizmo(gizmos_paths, 5., -0.001, "paths")
            .gizmo(gizmos_flat_paths, 3., -0.0011, "flat paths")
            .to_owned(),
    )
}
