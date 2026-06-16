//! Flow graphs: per-axis weighted edge graphs derived from the flow fields.
//!
use crate::prelude::*;
use mehsh::prelude::*;

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

impl Solution {
    /// Build the three per-axis flow graphs from the current flow fields.
    pub fn set_flow_graphs(&mut self, params: GraphParams) {
        // Early return if vector fields are missing
        let Some(fields) = &self.fields else {
            log::warn!("Cannot build flow graphs: vector fields are missing.");
            self.flow_graphs = None;
            return;
        };

        // Graph is based on edge to edge "connectivity"
        // A node for each edge, connected to its neighbors; two edges are connected if they share a vertex.
        // In a triangle mesh, this is the two edges in the same face, and the "twin" edge (on the opposite face).
        let mesh = &self.mesh_ref;
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
                    neighbors(edge).into_iter().map(move |next| {
                        (
                            edge,
                            next,
                            Self::get_edge_weight(mesh, field, edge, next, params),
                        )
                    })
                })
                .collect::<Vec<_>>();
            flow_graphs[axis as usize] = grapff::fixed::FixedGraph::from(nodes.clone(), edges);
        }

        self.flow_graphs = Some(flow_graphs);

        log::info!("flow graphs set");
    }

    // Compute an graph edge weight based on flow.
    fn get_edge_weight<T: Tag>(
        mesh: &Mesh<T>,
        field: &Field<T>,
        e0: ids::Key<EDGE, T>,
        e1: ids::Key<EDGE, T>,
        params: &GraphParams,
    ) -> f64 {
        // If the twin edge is the next edge, the weight is zero.
        if mesh.twin(e0) == e1 {
            return 0.0;
        }
        assert!(mesh.face(e0) == mesh.face(e1));
        let flow_vector = Self::get_flow_from_edge_to_edge(mesh, field, e0, e1);
        let flow_magnitude = flow_vector.norm().clamp(0.0, 1.0);
        let confidence = 1.0 - flow_magnitude;
        let flow_direction = flow_vector.normalize();

        let edge_vector = mesh.position(e1) - mesh.position(e0);
        let edge_direction = edge_vector.normalize();

        // Total weight calculation,
        // For a low weight (good weight) we want low angle and high confidence
        Vector3D::angle(&flow_direction, &edge_direction).powf(params.alignment_weight)
            + confidence * params.confidence_weight
    }

    // The flow (from flow field) from one edge to another edge.
    fn get_flow_from_edge_to_edge<T: Tag>(
        mesh: &Mesh<T>,
        field: &Field<T>,
        edge: ids::Key<EDGE, T>,
        next: ids::Key<EDGE, T>,
    ) -> Vector3D {
        let flow_edge = Self::get_flow_at_edge(mesh, field, edge);
        let flow_next = Self::get_flow_at_edge(mesh, field, next);
        0.5 * (flow_edge + flow_next)
    }

    // The flow (from flow field) at the midpoint of a half-edge.
    fn get_flow_at_edge<T: Tag>(
        mesh: &Mesh<T>,
        field: &Field<T>,
        edge: ids::Key<EDGE, T>,
    ) -> Vector3D {
        let (Some(f0), Some(f1)) = (
            field.vector_at(mesh.root(edge)),
            field.vector_at(mesh.toor(edge)),
        ) else {
            return Vector3D::zeros();
        };
        0.5 * (f0 + f1)
    }
}
