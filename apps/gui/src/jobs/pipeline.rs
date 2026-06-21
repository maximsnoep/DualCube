//! The solution pipeline: loops → dual → corners → layout → polycube → quad.
//!
//! Each stage clones the solution, applies one operation, and reports back as
//! a [`JobResult::StageCompleted`]; [`Stage::next_job`] then decides whether
//! the pipeline continues or stops with a refresh of the renders.

use super::{Job, JobResult};
use crate::render;
use crate::resources::{Configuration, Phase};
use bevy::prelude::*;
use dualcube::prelude::*;
#[cfg(feature = "hex")]
use dualcube_hex::HexExt;
#[cfg(feature = "quad")]
use dualcube_quad::QuadExt;
use mehsh::prelude::VertKey;

/// A completed stage of the pipeline.
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub(super) enum Stage {
    Field,
    Graph,
    Loops,
    Dual,
    Corners,
    Layout,
    Polycube,
    Quad,
    Hex,
}

impl Stage {
    /// The job that follows this stage. The pipeline stops (with a refresh of
    /// the renders) when the configured stop phase has been reached.
    pub(super) fn next_job(self, solution: Solution, configuration: Configuration) -> Job {
        let stop_phase = match self {
            Self::Loops => Some(Phase::Loops),
            Self::Dual => Some(Phase::Dual),
            Self::Layout => Some(Phase::Layout),
            Self::Polycube => Some(Phase::Polycube),
            // These stages never stop the pipeline themselves.
            Self::Field | Self::Graph | Self::Corners | Self::Quad | Self::Hex => None,
        };
        if stop_phase == Some(configuration.stop.clone()) {
            return Job::refresh(solution, configuration);
        }

        match self {
            Self::Field => Job::compute_graph(solution, configuration),
            Self::Graph => Job::refresh(solution, configuration),
            Self::Loops => Job::compute_dual(solution, configuration),
            Self::Dual => Job::place_corners(solution, configuration),
            Self::Corners => Job::place_paths(solution, configuration),
            Self::Layout => Job::compute_polycube(solution, configuration),
            Self::Polycube => Job::refresh(solution, configuration),
            Self::Quad => Job::refresh(solution, configuration),
            Self::Hex => Job::refresh(solution, configuration),
        }
    }
}

/// Clones the solution and applies `operation` to it.
/// Logs a warning and returns `None` if the operation fails.
fn try_step<E: std::fmt::Debug>(
    solution: &Solution,
    operation: &str,
    f: impl FnOnce(&mut Solution) -> Result<(), E>,
) -> Option<Solution> {
    let mut modified = solution.clone();
    match f(&mut modified) {
        Ok(()) => Some(modified),
        Err(err) => {
            warn!("Failed to {operation}: {err:?}");
            None
        }
    }
}

fn completed(stage: Stage, solution: Solution, configuration: &Configuration) -> Option<JobResult> {
    Some(JobResult::StageCompleted {
        stage,
        solution,
        configuration: configuration.clone(),
    })
}

#[allow(dead_code)]
impl Job {
    pub fn initialize_loops(solution: Solution, configuration: Configuration) -> Self {
        Self::new("initializing loops", move || {
            let mut initialized = Solution::new(solution.mesh_ref.clone());
            initialized.initialize();
            completed(Stage::Loops, initialized, &configuration)
        })
    }

    pub fn evolve(solution: Solution, configuration: Configuration) -> Self {
        Self::new("evolving", move || {
            match solution.evolve(
                configuration.iterations,
                configuration.pool1,
                configuration.pool2,
            ) {
                Ok(evolved) => return completed(Stage::Loops, evolved, &configuration),
                Err(e) => {
                    warn!("Failed to evolve solution. {e}");
                    return None;
                }
            };
        })
    }

    pub fn compute_fields(solution: Solution, configuration: Configuration) -> Self {
        Self::new("computing fields", move || {
            let mut solution = solution.clone();
            solution.fields = Some(Fields::new(
                &solution.mesh_ref,
                configuration.fields_params.clone(),
            ));
            completed(Stage::Field, solution, &configuration)
        })
    }

    pub fn compute_graph(solution: Solution, configuration: Configuration) -> Self {
        Self::new("computing flow graphs", move || {
            let mut solution = solution.clone();
            solution.set_flow_graphs(configuration.graph_params.clone());
            completed(Stage::Graph, solution, &configuration)
        })
    }

    pub fn compute_dual(solution: Solution, configuration: Configuration) -> Self {
        Self::new("computing dual", move || {
            match try_step(&solution, "construct dual and polycube", |s| {
                s.construct_dual_and_polycube()
            }) {
                Some(modified) => completed(Stage::Dual, modified, &configuration),
                // On failure still refresh, so the user sees the (unchanged) solution.
                None => Some(JobResult::Refreshed(render::refresh(
                    &solution,
                    &configuration,
                ))),
            }
        })
    }

    pub fn place_corners(solution: Solution, configuration: Configuration) -> Self {
        Self::new("placing corners", move || {
            let modified = try_step(&solution, "place corners and paths", |s| s.place_corners())?;
            completed(Stage::Corners, modified, &configuration)
        })
    }

    pub fn move_corner(
        solution: Solution,
        configuration: Configuration,
        corner: VertKey<POLYCUBE>,
        new_vertex: VertID,
    ) -> Self {
        Self::new("moving corner", move || {
            let modified = try_step(&solution, "move corner", |s| {
                s.move_corner_to(corner, new_vertex)
            })?;
            completed(Stage::Layout, modified, &configuration)
        })
    }

    pub fn place_paths(solution: Solution, configuration: Configuration) -> Self {
        Self::new("placing paths", move || {
            let modified = try_step(&solution, "place paths", |s| s.place_paths())?;
            completed(Stage::Layout, modified, &configuration)
        })
    }

    pub fn smoothen_layout(solution: Solution, configuration: Configuration) -> Self {
        Self::new("smoothening layout", move || {
            let modified = try_step(&solution, "optimize corners", |s| s.optimize_corners())?;
            completed(Stage::Layout, modified, &configuration)
        })
    }

    pub fn compute_polycube(solution: Solution, configuration: Configuration) -> Self {
        Self::new("computing polycube", move || {
            let modified = try_step(&solution, "resize polycube", |s| {
                s.resize_polycube(configuration.unit)
            })?;
            completed(Stage::Polycube, modified, &configuration)
        })
    }

    #[allow(unused_variables)]
    pub fn compute_quad(solution: Solution, configuration: Configuration) -> Self {
        #[cfg(feature = "quad")]
        {
            Self::new("computing quad", move || {
                let modified = try_step(&solution, "construct quad", |s| {
                    s.construct_quad(configuration.omega)
                })?;
                completed(Stage::Quad, modified, &configuration)
            })
        }
        #[cfg(not(feature = "quad"))]
        {
            Self::new("computing quad", move || {
                warn!(
                    "Quad computation is disabled. Rebuild gui with `--features quad` to enable it."
                );
                None
            })
        }
    }

    #[allow(unused_variables)]
    pub fn compute_hex(solution: Solution, configuration: Configuration) -> Self {
        #[cfg(feature = "hex")]
        {
            Self::new("computing hex", move || {
                let modified = try_step(&solution, "construct hex", |s| s.construct_hex())?;
                completed(Stage::Hex, modified, &configuration)
            })
        }
        #[cfg(not(feature = "hex"))]
        {
            Self::new("computing hex", move || {
                warn!(
                    "Hex computation is disabled. Rebuild gui with `--features hex` to enable it."
                );
                None
            })
        }
    }

    pub fn refresh(solution: Solution, configuration: Configuration) -> Self {
        Self::new("refreshing", move || {
            Some(JobResult::Refreshed(render::refresh(
                &solution,
                &configuration,
            )))
        })
    }
}
