//! Generic gizmo and color-map helpers shared by the render objects.

use crate::colors;
use bevy::prelude::*;
use dualcube::layout::Layout;
use dualcube::prelude::*;
use itertools::Itertools;
use mehsh::prelude::*;
use std::collections::HashMap;

/// Gizmo config group for the always-visible (perpetual) gizmos.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct PerpetualGizmos {}

/// Setup for gizmo rendering
pub fn setup(mut config_store: ResMut<GizmoConfigStore>) {
    let (perp_gizmos, _) = config_store.config_mut::<PerpetualGizmos>();
    perp_gizmos.depth_bias = -1.0;
}

/// Converts a [`Vector3D`] into a Bevy [`Vec3`].
#[inline]
#[must_use]
pub fn vector3d_to_vec3(v: Vector3D) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Transforms a world-space position into view space: `(p * s) + t`.
#[must_use]
pub fn world_to_view(v: Vector3D, translation: Vector3D, scale: f64) -> Vec3 {
    let vt = v * scale + translation;
    Vec3::new(vt.x as f32, vt.y as f32, vt.z as f32)
}

/// Inverse of [`world_to_view`]: `(p' - t) / s`.
#[must_use]
pub fn view_to_world(v: Vec3, translation: Vector3D, scale: f64) -> Vector3D {
    (Vector3D::new(f64::from(v.x), f64::from(v.y), f64::from(v.z)) - translation) / scale
}

/// One gizmo asset per principal direction.
pub struct DirectionalGizmos {
    pub x: GizmoAsset,
    pub y: GizmoAsset,
    pub z: GizmoAsset,
}

impl Default for DirectionalGizmos {
    fn default() -> Self {
        Self {
            x: GizmoAsset::new(),
            y: GizmoAsset::new(),
            z: GizmoAsset::new(),
        }
    }
}

impl DirectionalGizmos {
    pub fn get_mut(&mut self, direction: PrincipalDirection) -> &mut GizmoAsset {
        match direction {
            PrincipalDirection::X => &mut self.x,
            PrincipalDirection::Y => &mut self.y,
            PrincipalDirection::Z => &mut self.z,
        }
    }
}

/// Color map assigning the same color to every face of the mesh.
#[must_use]
pub fn uniform_color_map<M: Tag>(
    mesh: &mehsh::prelude::Mesh<M>,
    color: colors::Color,
) -> HashMap<FaceKey<M>, colors::Color> {
    mesh.face_ids()
        .into_iter()
        .map(|face_id| (face_id, color))
        .collect()
}

/// Color map with a Lambert-style grayscale shade per face, based on its normal.
#[must_use]
pub fn lambert_color_map<M: Tag>(
    mesh: &mehsh::prelude::Mesh<M>,
) -> HashMap<FaceKey<M>, colors::Color> {
    let light_dir = Vector3D::new(-25.0, 25.0, 25.0).normalize();
    let wrap = 0.5;

    mesh.face_ids()
        .into_iter()
        .map(|face_id| {
            let n = mesh.normal(face_id).normalize();
            let ndl = n.dot(&light_dir) as f32;
            let diffuse = ((ndl + wrap) / (1.0 + wrap)).clamp(0.0, 1.0);
            let hemi = 0.2 + 0.15 * ((n.y as f32) * 0.5 + 0.5);
            let shade = (0.75 * diffuse + 0.25 * hemi).clamp(0.0, 1.0);
            (face_id, [shade, shade, shade])
        })
        .collect()
}

/// Color map assigning each input triangle the color of the polycube patch it belongs to.
#[must_use]
pub fn segmentation_color_map(
    layout: &Layout,
    polycube: &Polycube,
) -> HashMap<FaceID, colors::Color> {
    let mut color_map = HashMap::new();
    for &patch_id in &polycube.structure.face_ids() {
        let normal = (polycube.structure.normal(patch_id) as Vector3D).normalize();
        let (dir, side) = to_principal_direction(normal);
        let color = colors::from_direction(dir, Some(Perspective::Primal), Some(side));
        for &triangle_id in &layout.face_to_patch[&patch_id].faces {
            color_map.insert(triangle_id, color);
        }
    }
    color_map
}

/// View-space endpoints of an edge in the given mesh.
#[must_use]
pub fn edge_endpoints_view<M: Tag>(
    mesh: &mehsh::prelude::Mesh<M>,
    edge_id: EdgeKey<M>,
    translation: Vector3D,
    scale: f64,
) -> (Vec3, Vec3) {
    let Some([v1, v2]) = mesh.vertices(edge_id).collect_array::<2>() else {
        panic!("Expected two vertices for edge {edge_id:?}");
    };
    (
        world_to_view(mesh.position(v1), translation, scale),
        world_to_view(mesh.position(v2), translation, scale),
    )
}

/// Gizmos for the layout paths, drawn on `position_mesh` (which must share edge
/// ids with the layout's granulated mesh). Returns `(paths, flat_paths)`:
/// `flat_paths` contains all path segments, `paths` only those between polycube
/// faces with different normals.
#[must_use]
pub fn layout_path_gizmos(
    layout: &Layout,
    polycube: &Polycube,
    position_mesh: &mehsh::prelude::Mesh<INPUT>,
    translation: Vector3D,
    scale: f64,
) -> (GizmoAsset, GizmoAsset) {
    let mut paths = GizmoAsset::new();
    let mut flat_paths = GizmoAsset::new();
    let c = colors::to_bevy(colors::GRAY);
    let structure = &polycube.structure;

    for (&pedge_id, path) in &layout.edge_to_path {
        let n1 = structure.normal(structure.face(pedge_id));
        let n2 = structure.normal(structure.face(structure.twin(pedge_id)));
        for vertexpair in path.windows(2) {
            let Some((edge_id, _)) = layout
                .granulated_mesh
                .edge_between_verts(vertexpair[0], vertexpair[1])
            else {
                warn!(
                    "Edge between {:?} and {:?} does not exist; skipping path segment",
                    vertexpair[0], vertexpair[1]
                );
                continue;
            };
            let (u, v) = edge_endpoints_view(position_mesh, edge_id, translation, scale);
            flat_paths.line(u, v, c);
            if n1 != n2 {
                paths.line(u, v, c);
            }
        }
    }

    (paths, flat_paths)
}
