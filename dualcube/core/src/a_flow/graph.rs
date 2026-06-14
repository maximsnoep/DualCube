//! Flow graphs: per-axis weighted edge graphs derived from the flow fields.
//!
//! For each principal direction we build a graph over the half-edges of the
//! input mesh, where the weight of moving from one half-edge to a neighbor
//! penalizes misalignment with the corresponding flow field. Loops are later
//! traced as cheap paths through these graphs.

use crate::flow::Field;
use crate::prelude::*;
use crate::solutions::Solution;
use mehsh::prelude::*;
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct GraphParams {
    pub alignment_penalty_weight: f64,
    pub length_penalty_weight: f64,
    pub combined_penalty_weight: f64,
}

impl Default for GraphParams {
    fn default() -> Self {
        Self {
            alignment_penalty_weight: 100.0,
            length_penalty_weight: 120.0,
            combined_penalty_weight: 180.0,
        }
    }
}

impl Solution {
    /// Build the three per-axis flow graphs from the current flow fields.
    pub fn set_flow_graphs(&mut self) {
        self.set_flow_graphs_with_params(GraphParams::default());
    }

    pub fn set_flow_graphs_with_params(&mut self, params: GraphParams) {
        let Some(fields) = &self.fields else {
            log::warn!("Cannot build flow graphs: vector fields are missing.");
            self.flow_graphs = None;
            return;
        };

        let mesh = &self.mesh_ref;
        let nodes = mesh.edge_ids();
        let neighbors = mesh.neighbor_function_edgegraph();
        let fields = [&fields.field_x, &fields.field_y, &fields.field_z];

        let mut flow_graphs = [
            grapff::fixed::FixedGraph::default(),
            grapff::fixed::FixedGraph::default(),
            grapff::fixed::FixedGraph::default(),
        ];

        for axis in [
            PrincipalDirection::X,
            PrincipalDirection::Y,
            PrincipalDirection::Z,
        ] {
            let field = fields[axis as usize];

            let edge_fields: HashMap<_, _> = nodes
                .iter()
                .copied()
                .map(|edge| (edge, Self::edge_field(mesh, field, edge)))
                .collect();

            let edges = nodes
                .iter()
                .copied()
                .flat_map(|edge| {
                    let edge_fields = &edge_fields;
                    let params = &params;
                    neighbors(edge).into_iter().map(move |next| {
                        (
                            edge,
                            next,
                            Self::flow_weight(mesh, edge_fields, edge, next, params),
                        )
                    })
                })
                .collect::<Vec<_>>();

            log::info!("axis: {:?}, edges: {}", axis, edges.len());

            flow_graphs[axis as usize] = grapff::fixed::FixedGraph::from(nodes.clone(), edges);
        }

        self.flow_graphs = Some(flow_graphs);
        *self
            .available_flow_graphs
            .write()
            .expect("available flow graph cache lock poisoned") = None;

        log::info!("flow graphs set");
    }

    /// Weight of traversing from half-edge `edge` to its neighbor `next`.
    ///
    /// A low weight requires both:
    /// - good directional alignment with the local flow field, and
    /// - a long/confident local flow vector.
    ///
    /// Short field vectors mean uncertain/bad flow, so they make the edge expensive
    /// even if the remaining direction happens to align.
    fn flow_weight<T: Tag>(
        mesh: &Mesh<T>,
        edge_fields: &HashMap<ids::Key<EDGE, T>, Vector3D>,
        edge: ids::Key<EDGE, T>,
        next: ids::Key<EDGE, T>,
        params: &GraphParams,
    ) -> f64 {
        if mesh.face(edge) != mesh.face(next) {
            assert_eq!(mesh.twin(edge), next);
            return 0.0;
        }

        let d = mesh.position(next) - mesh.position(edge);
        let len = d.norm();
        if len <= 1e-12 {
            return 0.0;
        }
        let d = d / len;

        let f = 0.5 * (edge_fields[&edge] + edge_fields[&next]);
        let field_length = f.norm().clamp(0.0, 1.0);
        let field_direction = unit_or(f, -d);

        let alignment_quality =
            ((1.0 + d.dot(&field_direction).clamp(-1.0, 1.0)) * 0.5).clamp(0.0, 1.0);
        let alignment_penalty = 1.0 - alignment_quality;
        let length_penalty = 1.0 - field_length;
        let combined_penalty = 1.0 - alignment_quality * field_length;

        len * (1.0
            + params.alignment_penalty_weight * alignment_penalty.powi(4)
            + params.length_penalty_weight * length_penalty.powi(4)
            + params.combined_penalty_weight * combined_penalty.powi(4))
    }

    /// The flow field vector at the midpoint of a half-edge.
    ///
    /// Keeps vector length intact because length is the field confidence used by
    /// `flow_weight`.
    fn edge_field<T: Tag>(mesh: &Mesh<T>, field: &Field<T>, edge: ids::Key<EDGE, T>) -> Vector3D {
        let f0 = field.vector_at(mesh.root(edge)).unwrap();
        let f1 = field.vector_at(mesh.toor(edge)).unwrap();
        0.5 * (f0 + f1)
    }
}

fn unit_or(v: Vector3D, fallback: Vector3D) -> Vector3D {
    if v.norm() > 1e-12 {
        v.normalize()
    } else {
        fallback
    }
}
