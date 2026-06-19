//! Jobs that read and write solutions from/to disk.

use crate::jobs::{Job, JobResult};
use dualcube::prelude::*;
use std::path::PathBuf;

impl Job {
    pub fn import(path: PathBuf) -> Self {
        Self::new("importing", move || {
            info!("Importing solution from {}", path.display());
            let solution = io::import_solution(&path);
            Some(JobResult::Imported { solution })
        })
    }

    pub fn export(solution: Solution, path: PathBuf) -> Self {
        Self::new("exporting", move || {
            if solution.mesh_ref.vert_ids().is_empty() {
                warn!("Nothing to export: the mesh is empty");
                return None;
            }
            info!("Exporting solution to {}", path.display());
            io::export_solution(&solution, &path);
            None
        })
    }
}
