//! Render object for the polycube.

use super::gizmos::{edge_endpoints_view, uniform_color_map, world_to_view, DirectionalGizmos};
use super::store::RenderObject;
use crate::colors;
use bevy::prelude::*;
use dualcube::prelude::*;
use itertools::Itertools;
use mehsh::prelude::*;
use std::collections::HashMap;

/// The POLYCUBE render object, it has:
/// - meshes with black / gray / colored faces
/// - the dual loops per principal direction
/// - the polycube edges (flat and non-flat)
pub(super) fn build(solution: &Solution) -> Option<RenderObject> {
    let polycube = solution.polycube.as_ref()?;

    let structure = &polycube.structure;
    let (scale, translation) = structure.scale_translation();

    let mut colored_color_map = HashMap::new();
    let mut loops = DirectionalGizmos::default();

    for face_id in structure.face_ids() {
        colored_color_map.insert(
            face_id,
            colors::from_direction(
                to_principal_direction(structure.normal(face_id)).0,
                Some(Perspective::Primal),
                None,
            ),
        );

        // Each quad face carries two loop segments, connecting the midpoints of
        // opposite edges, colored by the dual direction they run along.
        let Some(edges) = structure.edges(face_id).collect_array::<4>() else {
            panic!("Expected four edges for face {face_id:?}");
        };
        let positions = edges.map(|edge_id| structure.position(edge_id));
        let views = positions.map(|position| world_to_view(position, translation, scale));

        for (from, to, axis) in [
            (0, 2, positions[1] - positions[3]),
            (1, 3, positions[0] - positions[2]),
        ] {
            let dir = to_principal_direction(axis).0;
            let c = colors::to_bevy(colors::from_direction(dir, Some(Perspective::Dual), None));
            loops.get_mut(dir).line(views[from], views[to], c);
        }
    }

    // Polycube edges; edges between faces with equal normals are flat.
    let mut gizmos_paths = GizmoAsset::new();
    let mut gizmos_flat_paths = GizmoAsset::new();
    let c = colors::to_bevy(colors::GRAY);
    for pedge_id in structure.edge_ids() {
        let n1 = structure.normal(structure.face(pedge_id));
        let n2 = structure.normal(structure.face(structure.twin(pedge_id)));
        let (u, v) = edge_endpoints_view(structure, pedge_id, translation, scale);
        gizmos_flat_paths.line(u, v, c);
        if n1 != n2 {
            gizmos_paths.line(u, v, c);
        }
    }

    Some(
        RenderObject::default()
            .mesh(
                structure,
                &uniform_color_map(structure, colors::BLACK),
                "black",
            )
            .mesh(
                structure,
                &uniform_color_map(structure, colors::LIGHT_GRAY),
                "gray",
            )
            .mesh(structure, &colored_color_map, "colored")
            .gizmo(loops.x, 6., -0.001, "x-loops")
            .gizmo(loops.y, 6., -0.0011, "y-loops")
            .gizmo(loops.z, 6., -0.00111, "z-loops")
            .gizmo(gizmos_paths, 5., -0.001, "paths")
            .gizmo(gizmos_flat_paths, 3., -0.0011, "flat paths")
            .to_owned(),
    )
}
