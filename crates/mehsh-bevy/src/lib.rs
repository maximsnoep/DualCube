use mehsh::prelude::*;
use orx_parallel::*;

#[derive(Default)]
pub struct MeshBuilder {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    uvs: Vec<[f32; 2]>,
}

impl MeshBuilder {
    #[must_use]
    pub fn new() -> Self {
        Self {
            positions: Vec::new(),
            normals: Vec::new(),
            colors: Vec::new(),
            uvs: Vec::new(),
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn normalize(&mut self, scale: f64, translation: Vector3D) {
        for position in &mut self.positions {
            *position = [
                position[0].mul_add(scale as f32, translation.x as f32),
                position[1].mul_add(scale as f32, translation.y as f32),
                position[2].mul_add(scale as f32, translation.z as f32),
            ];
        }
    }

    #[must_use]
    pub fn build(self) -> bevy::mesh::Mesh {
        bevy::mesh::Mesh::new(
            bevy::mesh::PrimitiveTopology::TriangleList,
            bevy::asset::RenderAssetUsages::RENDER_WORLD
                | bevy::asset::RenderAssetUsages::MAIN_WORLD,
        )
        .with_inserted_indices(bevy::mesh::Indices::U32(
            (0..self.positions.len() as u32).collect::<Vec<_>>(),
        ))
        .with_inserted_attribute(bevy::mesh::Mesh::ATTRIBUTE_POSITION, self.positions)
        .with_inserted_attribute(bevy::mesh::Mesh::ATTRIBUTE_NORMAL, self.normals)
        .with_inserted_attribute(bevy::mesh::Mesh::ATTRIBUTE_COLOR, self.colors)
        .with_inserted_attribute(bevy::mesh::Mesh::ATTRIBUTE_UV_0, self.uvs)
    }
}

/// Construct a Bevy mesh object (one that can be rendered using Bevy).
/// Requires a `color_map` to assign colors to faces. If no color is assigned to a face, it will default to black.
#[must_use]
pub fn to_bevy<M: Tag>(
    mesh: &Mesh<M>,
    color_map: &HashMap<FaceKey<M>, [f32; 3]>,
) -> (bevy::mesh::Mesh, Vector3D, f64) {
    if mesh.faces.is_empty() {
        return (MeshBuilder::new().build(), Vector3D::new(0., 0., 0.), 1.);
    }
    let mut bevy_mesh_builder = bevy_builder(mesh, color_map);
    let (scale, translation) = mesh.scale_translation();
    bevy_mesh_builder.normalize(scale, translation);
    (bevy_mesh_builder.build(), translation, scale)
}

fn bevy_builder<M: Tag>(mesh: &Mesh<M>, color_map: &HashMap<FaceKey<M>, [f32; 3]>) -> MeshBuilder {
    let triangulated_faces = mesh
        .face_ids()
        .par()
        .map(|&id| match mesh.vertices(id).collect_vec().as_slice() {
            &[v0, v1, v2] => {
                let (p0, p1, p2) = (
                    v3d_to_slice(mesh.position(v0)),
                    v3d_to_slice(mesh.position(v1)),
                    v3d_to_slice(mesh.position(v2)),
                );
                let (n0, n1, n2) = (
                    v3d_to_slice(mesh.normal(v0)),
                    v3d_to_slice(mesh.normal(v1)),
                    v3d_to_slice(mesh.normal(v2)),
                );
                let c = srgb_to_linear(color_map.get(&id).copied().unwrap_or([0., 0., 0.]));
                (vec![p0, p1, p2], vec![n0, n1, n2], vec![c; 3])
            }
            &[v0, v1, v2, v3] => {
                let (p0, p1, p2, p3) = (
                    v3d_to_slice(mesh.position(v0)),
                    v3d_to_slice(mesh.position(v1)),
                    v3d_to_slice(mesh.position(v2)),
                    v3d_to_slice(mesh.position(v3)),
                );
                let (n0, n1, n2, n3) = (
                    v3d_to_slice(mesh.normal(v0)),
                    v3d_to_slice(mesh.normal(v1)),
                    v3d_to_slice(mesh.normal(v2)),
                    v3d_to_slice(mesh.normal(v3)),
                );
                let c = srgb_to_linear(color_map.get(&id).copied().unwrap_or([0., 0., 0.]));
                (
                    vec![p3, p0, p1, p1, p2, p3],
                    vec![n3, n0, n1, n1, n2, n3],
                    vec![c; 6],
                )
            }
            _ => (vec![], vec![], vec![]),
        })
        .collect::<Vec<_>>();

    let total_len: usize = triangulated_faces.iter().map(|(p, _, _)| p.len()).sum();
    info!("Building mesh for Bevy with {total_len} vertices.",);

    let mut positions = Vec::with_capacity(total_len);
    let mut normals = Vec::with_capacity(total_len);
    let mut colors = Vec::with_capacity(total_len);

    for (p, n, c) in triangulated_faces {
        positions.extend(p);
        normals.extend(n);
        colors.extend(c);
    }
    let uvs = vec![[0., 0.]; positions.len()];

    MeshBuilder {
        positions,
        normals,
        colors,
        uvs,
    }
}

// Construct a Bevy gizmos object of the wireframe (one that can be rendered using Bevy)
#[must_use]
pub fn gizmos<M: Tag>(mesh: &Mesh<M>, color: [f32; 3]) -> bevy::gizmos::GizmoAsset {
    let mut gizmo = bevy::gizmos::GizmoAsset::new();
    let (scale, translation) = mesh.scale_translation();
    for e in mesh.edge_ids_iter() {
        if let &[u, v] = mesh
            .vertices(e)
            .map(|id| mesh.position(id))
            .collect::<Vec<_>>()
            .as_slice()
        {
            gizmo.line(
                v3d_to_bevy(&(u * scale + translation)),
                v3d_to_bevy(&(v * scale + translation)),
                srgb_to_bevy(color),
            );
        }
    }
    gizmo
}

fn v3d_to_slice(vec: Vector3D) -> [f32; 3] {
    [vec.x as f32, vec.y as f32, vec.z as f32]
}

fn srgb_to_linear(color: [f32; 3]) -> [f32; 4] {
    bevy::color::ColorToComponents::to_f32_array(
        bevy::color::Color::srgb_from_array(color).to_linear(),
    )
}

fn v3d_to_bevy(vec: &Vector3D) -> bevy::math::Vec3 {
    bevy::math::Vec3::new(vec.x as f32, vec.y as f32, vec.z as f32)
}

fn srgb_to_bevy(color: [f32; 3]) -> bevy::color::Color {
    bevy::color::Color::srgb_from_array(color)
}
