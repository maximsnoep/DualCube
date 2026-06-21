//! Flow graphs: per-axis weighted edge graphs derived from the flow fields.

use crate::flowfield::Field;
use dualcube_types::prelude::*;

#[derive(Debug, Clone)]
pub struct GraphParams {
    pub alignment_weight: f64,
    pub confidence_weight: f64,
}

impl Default for GraphParams {
    fn default() -> Self {
        Self {
            alignment_weight: 10.0,
            confidence_weight: 1.0,
        }
    }
}

pub fn build_flow_graphs<T: Tag>(
    mesh: &Mesh<T>,
    fields: &crate::flowfield::Fields<T>,
    params: GraphParams,
) -> [grapff::fixed::FixedGraph<ids::Key<EDGE, T>, f64>; 3] {
    let params = &params;
    let nodes = mesh.edge_ids();
    let neighbors = mesh.neighbor_function_edgegraph();

    let mut flow_graphs = [
        grapff::fixed::FixedGraph::default(),
        grapff::fixed::FixedGraph::default(),
        grapff::fixed::FixedGraph::default(),
    ];

    for (field, axis) in [
        (&fields.field_x, Direction::X),
        (&fields.field_y, Direction::Y),
        (&fields.field_z, Direction::Z),
    ] {
        let edges = nodes
            .iter()
            .copied()
            .flat_map(|edge| {
                neighbors(edge)
                    .into_iter()
                    .map(move |next| (edge, next, get_edge_weight(mesh, field, edge, next, params)))
            })
            .collect::<Vec<_>>();
        flow_graphs[axis as usize] = grapff::fixed::FixedGraph::from(nodes.clone(), edges);
    }

    flow_graphs
}

fn get_edge_weight<T: Tag>(
    mesh: &Mesh<T>,
    field: &Field<T>,
    e0: ids::Key<EDGE, T>,
    e1: ids::Key<EDGE, T>,
    params: &GraphParams,
) -> f64 {
    if mesh.twin(e0) == e1 {
        return 0.0;
    }
    assert!(mesh.face(e0) == mesh.face(e1));
    let flow_vector = get_flow_from_edge_to_edge(mesh, field, e0, e1);
    let flow_magnitude = flow_vector.norm().clamp(0.0, 1.0);
    let confidence = 1.0 - flow_magnitude;

    if flow_magnitude < 1e-12 {
        return confidence * params.confidence_weight;
    }

    let flow_direction = flow_vector.normalize();

    let edge_vector = mesh.position(e1) - mesh.position(e0);
    let edge_direction = edge_vector.normalize();

    Vector3D::angle(&flow_direction, &edge_direction).powf(params.alignment_weight)
        + confidence * params.confidence_weight
}

fn get_flow_from_edge_to_edge<T: Tag>(
    mesh: &Mesh<T>,
    field: &Field<T>,
    edge: ids::Key<EDGE, T>,
    next: ids::Key<EDGE, T>,
) -> Vector3D {
    let flow_edge = get_flow_at_edge(mesh, field, edge);
    let flow_next = get_flow_at_edge(mesh, field, next);
    0.5 * (flow_edge + flow_next)
}

fn get_flow_at_edge<T: Tag>(mesh: &Mesh<T>, field: &Field<T>, edge: ids::Key<EDGE, T>) -> Vector3D {
    let (Some(f0), Some(f1)) = (
        field.vector_at(mesh.root(edge)),
        field.vector_at(mesh.toor(edge)),
    ) else {
        return Vector3D::zeros();
    };
    0.5 * (f0 + f1)
}
