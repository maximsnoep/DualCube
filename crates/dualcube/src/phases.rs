use crate::prelude::*;
use slotmap::SlotMap;
use std::sync::Arc;

#[derive(Clone, Debug)]
pub struct InputPhase {
    pub mesh_ref: Arc<Mesh<INPUT>>,
}

impl InputPhase {
    #[must_use]
    pub fn new(mesh_ref: Arc<Mesh<INPUT>>) -> Self {
        Self { mesh_ref }
    }

    #[must_use]
    pub fn compute_flow(&self, field_params: FieldParams, graph_params: GraphParams) -> FlowPhase {
        let fields = Fields::new(&self.mesh_ref, field_params);
        let flow_graphs = FlowPhase::build_graphs(&self.mesh_ref, &fields, graph_params);
        FlowPhase {
            input: self.clone(),
            fields,
            flow_graphs,
        }
    }
}

#[derive(Clone, Debug)]
pub struct FlowPhase {
    pub input: InputPhase,
    pub fields: Fields<INPUT>,
    pub flow_graphs: Arc<[grapff::fixed::FixedGraph<EdgeID, f64>; 3]>,
}

impl FlowPhase {
    #[must_use]
    pub fn build_graphs(
        mesh_ref: &Mesh<INPUT>,
        fields: &Fields<INPUT>,
        graph_params: GraphParams,
    ) -> Arc<[grapff::fixed::FixedGraph<EdgeID, f64>; 3]> {
        Arc::new(build_flow_graphs(mesh_ref, fields, graph_params))
    }

    pub fn compute_dual(
        &self,
        loops: SlotMap<LoopID, Loop>,
    ) -> Result<DualPhase, PropertyViolationError> {
        DualPhase::from_loops(self.input.clone(), loops)
    }
}

#[derive(Clone, Debug)]
pub struct DualPhase {
    pub input: InputPhase,
    pub loops: SlotMap<LoopID, Loop>,
    pub dual: Dual,
}

impl DualPhase {
    pub fn from_loops(
        input: InputPhase,
        loops: SlotMap<LoopID, Loop>,
    ) -> Result<Self, PropertyViolationError> {
        let dual = Dual::from(input.mesh_ref.clone(), &loops)?;
        Ok(Self { input, loops, dual })
    }

    #[must_use]
    pub fn compute_polycube(&self) -> Polycube {
        Polycube::from_dual(&self.dual)
    }

    pub fn compute_primal(&self, unit: bool) -> Result<PrimalPhase, SolutionError> {
        let polycube = self.compute_polycube();
        validate_polycube(&polycube)?;

        let mut layout = None;
        let mut layout_attempts = 0;
        for _ in 0..10 {
            layout_attempts += 1;
            match Layout::embed(&self.dual, &polycube) {
                Ok(ok_layout) => {
                    layout = Some(ok_layout);
                    break;
                }
                Err(err) => {
                    info!("compute_primal: layout attempt {layout_attempts} failed: {err:?}");
                }
            }
        }

        let Some(layout) = layout else {
            return Err(SolutionError::NoPrimal);
        };

        let mut primal = PrimalPhase {
            dual: self.clone(),
            polycube,
            layout,
        };
        primal.resize_polycube(unit)?;
        Ok(primal)
    }
}

#[derive(Clone, Debug)]
pub struct PrimalPhase {
    pub dual: DualPhase,
    pub polycube: Polycube,
    pub layout: Layout,
}

impl PrimalPhase {
    pub fn resize_polycube(&mut self, unit: bool) -> Result<(), SolutionError> {
        if unit {
            self.polycube.resize(&self.dual.dual, None);
        } else {
            self.polycube.resize(&self.dual.dual, Some(&self.layout));
        }
        Ok(())
    }

    #[must_use]
    pub fn quality(&self, loop_count: usize) -> Option<f64> {
        let beta = 0.001;
        if let (Some(alignment), Some(orthogonality)) =
            (self.layout.alignment, self.layout.orthogonality)
        {
            Some(alignment + orthogonality - beta * loop_count as f64)
        } else {
            None
        }
    }
}

pub fn validate_polycube(polycube: &Polycube) -> Result<(), SolutionError> {
    for face in polycube.structure.face_ids() {
        let normal = polycube.structure.normal(face);
        if normal.x.is_nan() || normal.y.is_nan() || normal.z.is_nan() {
            return Err(SolutionError::NoPolycube);
        }
    }
    Ok(())
}
