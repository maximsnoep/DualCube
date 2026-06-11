//! Jobs that read and write solutions from/to disk.
//!
//! To add a new format: add one constructor here (use [`log_export`] for
//! result reporting).

use super::{Job, JobResult};
use bevy::prelude::*;
use dualcube::prelude::Solution;
use io::Export;
use std::path::PathBuf;

/// Logs the outcome of exporting a file.
fn log_export<E: std::fmt::Debug>(format: &str, path: &PathBuf, result: Result<(), E>) {
    match result {
        Ok(()) => info!("Exported {format} file to {}", path.display()),
        Err(err) => warn!(
            "Failed to export {format} file to {}: {err:?}",
            path.display()
        ),
    }
}

impl Job {
    pub fn import(path: PathBuf) -> Self {
        Self::new("importing", move || {
            info!("Importing solution from {}", path.display());
            Some(JobResult::Imported {
                solution: io::import_solution(path.clone()),
            })
        })
    }

    /// Exports the solution as Dsol + OBJ + Flag files.
    pub fn export(solution: Solution, path: PathBuf) -> Self {
        Self::new("exporting", move || {
            if solution.mesh_ref.vert_ids().is_empty() {
                warn!("Nothing to export: the mesh is empty");
                return None;
            }
            log_export("Dc", &path, io::Dc::export(&solution, &path));
            log_export("OBJ", &path, io::OBJ::export(&solution, &path));
            log_export("Flag", &path, io::Flag::export(&solution, &path));
            None
        })
    }

    pub fn export_nlr(solution: Solution, path: PathBuf) -> Self {
        Self::new("exporting (NLR)", move || {
            if solution.mesh_ref.vert_ids().is_empty() {
                warn!("Nothing to export: the mesh is empty");
                return None;
            }
            log_export("NLR", &path, io::NLR::export(&solution, &path));
            None
        })
    }

    pub fn export_dotgraph(solution: Solution, path: PathBuf) -> Self {
        Self::new("exporting (Dotgraph)", move || {
            log_export("APG", &path, io::APG::export(&solution, &path));
            None
        })
    }

    pub fn export_hex(solution: Solution, path: PathBuf) -> Self {
        Self::new("hexing", move || {
            log_export("HEX", &path, io::HEX::export(&solution, &path));
            None
        })
    }
}
