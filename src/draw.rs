use douconel::{
    douconel::{Douconel, EdgeID, FaceID, VertID},
    douconel_embedded::HasPosition,
};
use hutspot::{draw::DrawableLine, geom::Vector3D};

use crate::{GizmoType, Line};

pub fn add_face_normal<V, E, F>(lines: &mut Vec<Line>, face_id: FaceID, mesh: &Douconel<V, E, F>, translation: Vector3D, scale: f32)
where
    E: std::default::Default,
    F: std::default::Default,
    V: std::default::Default + HasPosition,
{
    let p = mesh.centroid(face_id);
    let n = mesh.normal(face_id);
    for line in DrawableLine::from_arrow(p, p + n, p.cross(&n).normalize(), 0.05, Vector3D::new(0., 0., 0.), translation, scale) {
        lines.push((line.u, line.v, hutspot::color::ROODT.into(), GizmoType::Normal));
    }
}

pub fn add_vertex<V, E, F>(lines: &mut Vec<Line>, vert_id: VertID, mesh: &Douconel<V, E, F>, translation: Vector3D, scale: f32)
where
    E: std::default::Default,
    F: std::default::Default,
    V: std::default::Default + HasPosition,
{
    let p = mesh.position(vert_id);
    let n = mesh.vert_normal(vert_id);
    let line = DrawableLine::from_vertex(p, n, 0.01, translation, scale);
    lines.push((line.u, line.v, hutspot::color::GRIJS.into(), GizmoType::Vertex));
}

pub fn add_edge<V, E, F>(lines: &mut Vec<Line>, edge_id: EdgeID, mesh: &Douconel<V, E, F>, translation: Vector3D, scale: f32)
where
    E: std::default::Default,
    F: std::default::Default,
    V: std::default::Default + HasPosition,
{
    let endpoints = mesh.endpoints(edge_id);
    let u = mesh.position(endpoints.0);
    let v = mesh.position(endpoints.1);
    let line = DrawableLine::from_line(u, v, mesh.normal(mesh.face(edge_id)) * 0.001, translation, scale);
    lines.push((line.u, line.v, hutspot::color::GRIJS.into(), GizmoType::Wireframe));
}
