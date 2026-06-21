use crate::colors;
use crate::colors::Kolor;
use bevy::prelude::*;
use dualcube::prelude::*;
use std::collections::HashMap;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct PerpetualGizmos {}

pub fn setup(mut config_store: ResMut<'_, GizmoConfigStore>) {
    let (perp_gizmos, _) = config_store.config_mut::<PerpetualGizmos>();
    perp_gizmos.depth_bias = -1.0;
}

#[inline]
#[must_use]
pub fn vector3d_to_vec3(v: Vector3D) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

#[must_use]
pub fn world_to_view(v: Vector3D, translation: Vector3D, scale: f64) -> Vec3 {
    vector3d_to_vec3(v * scale + translation)
}

#[must_use]
pub fn view_to_world(v: Vec3, translation: Vector3D, scale: f64) -> Vector3D {
    (Vector3D::new(f64::from(v.x), f64::from(v.y), f64::from(v.z)) - translation) / scale
}

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
    pub fn get_mut(&mut self, direction: Direction) -> &mut GizmoAsset {
        match direction {
            Direction::X => &mut self.x,
            Direction::Y => &mut self.y,
            Direction::Z => &mut self.z,
        }
    }
}

#[must_use]
pub fn uniform_color_map<M: Tag>(
    mesh: &mehsh::prelude::Mesh<M>,
    color: Kolor,
) -> HashMap<FaceKey<M>, Kolor> {
    mesh.face_ids()
        .into_iter()
        .map(|face_id| (face_id, color))
        .collect()
}

#[must_use]
pub fn lambert_color_map<M: Tag>(mesh: &mehsh::prelude::Mesh<M>) -> HashMap<FaceKey<M>, Kolor> {
    let light_dir = Vector3D::new(-25.0, 25.0, 25.0).normalize();
    let wrap = 0.5;

    mesh.face_ids()
        .into_iter()
        .map(|face_id| {
            let shade = lambert_shade(mesh.normal(face_id).normalize(), light_dir, wrap);
            (face_id, [shade, shade, shade])
        })
        .collect()
}

#[must_use]
pub fn segmentation_color_map(layout: &Layout, polycube: &Polycube) -> HashMap<FaceID, Kolor> {
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

#[must_use]
pub fn flow_graph_gizmos(
    edges: &[(EdgeID, EdgeID, f64)],
    mesh: &mehsh::prelude::Mesh<INPUT>,
    base_color: Kolor,
    translation: Vector3D,
    scale: f64,
    top_percent: f32,
) -> GizmoAsset {
    let mut gizmos = GizmoAsset::new();

    let Some(threshold) = flow_weight_threshold(edges, top_percent) else {
        return gizmos;
    };

    let color = colors::to_bevy(base_color);

    for &(from, to, weight) in edges {
        if weight <= 1e-9 || weight > threshold {
            continue;
        }

        gizmos.arrow(
            world_to_view(mesh.position(from), translation, scale),
            world_to_view(mesh.position(to), translation, scale),
            color,
        );
    }

    gizmos
}

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

    // `position_mesh` must share edge ids with the granulated layout mesh.
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

fn lambert_shade(normal: Vector3D, light_dir: Vector3D, wrap: f64) -> f32 {
    let diffuse =
        ((normal.dot(&light_dir) as f32 + wrap as f32) / (1.0 + wrap as f32)).clamp(0.0, 1.0);
    let hemi = 0.2 + 0.15 * ((normal.y as f32) * 0.5 + 0.5);
    (0.75 * diffuse + 0.25 * hemi).clamp(0.0, 1.0)
}

fn flow_weight_threshold(edges: &[(EdgeID, EdgeID, f64)], top_percent: f32) -> Option<f64> {
    let mut weights = edges
        .iter()
        .filter_map(|&(_, _, weight)| (weight > 1e-9).then_some(weight))
        .collect::<Vec<_>>();

    weights.sort_by(f64::total_cmp);
    let keep_count = (weights.len() as f32 * top_percent.clamp(0.0, 100.0) / 100.0).ceil() as usize;
    (keep_count > 0).then(|| weights[keep_count.saturating_sub(1).min(weights.len() - 1)])
}
