//! Experimental path straightening via the external `flip_geodesics` tool.
//!
//! The granulated mesh and its paths are written to an OBJ+ file, the tool
//! straightens the paths, and the (possibly split) edges are read back in.

use super::pipeline::Stage;
use super::{Job, JobResult};
use crate::resources::Configuration;
use bevy::prelude::*;
use dualcube::prelude::*;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;

const FLIP_GEODESICS_EXE: &str =
    "C:\\Users\\20182085\\Documents\\flip-geodesics-demo\\build\\bin\\Release\\flip_geodesics.exe";
const TEMP_OBJ: &str =
    "C:\\Users\\20182085\\Documents\\flip-geodesics-demo\\build\\bin\\Release\\temp.obj";
const TEMP_LINES: &str =
    "C:\\Users\\20182085\\Documents\\flip-geodesics-demo\\build\\bin\\Release\\temp.lines";

impl Job {
    pub fn path_straightening(solution: Solution, configuration: Configuration) -> Self {
        Self::new("path straightening", move || {
            straighten(&solution, &configuration)
        })
    }
}

fn straighten(solution: &Solution, configuration: &Configuration) -> Option<JobResult> {
    let Some(layout) = &solution.layout else {
        warn!("Cannot straighten paths: no layout available");
        return None;
    };

    let mut solution_clone = solution.clone();
    for _ in 0..3 {
        let input_path = PathBuf::from(TEMP_OBJ);
        let output_path = PathBuf::from(TEMP_LINES);

        // Write the granulated mesh and its paths as an OBJ+ file.
        info!("Writing OBJ+ file to {}", input_path.display());
        let vertex_map = match layout.granulated_mesh.to_obj(&input_path) {
            Ok(vertex_map) => vertex_map,
            Err(err) => {
                warn!("Failed to write {}: {err:?}", input_path.display());
                return None;
            }
        };
        let mut file = match OpenOptions::new().append(true).open(&input_path) {
            Ok(file) => file,
            Err(err) => {
                warn!("Failed to open {}: {err:?}", input_path.display());
                return None;
            }
        };
        for path in layout.edge_to_path.values() {
            let line = path
                .iter()
                .map(|vert_id| format!("{}", vertex_map.id(vert_id).unwrap()))
                .join(" ");
            if let Err(err) = writeln!(file, "l {}", line) {
                warn!("Failed to write to {}: {err:?}", input_path.display());
                return None;
            }
        }

        // Run the path straightening tool.
        let status = match std::process::Command::new(FLIP_GEODESICS_EXE)
            .arg(&input_path)
            .arg(&output_path)
            .status()
        {
            Ok(status) => status,
            Err(err) => {
                warn!("Failed to run {FLIP_GEODESICS_EXE}: {err:?}");
                return None;
            }
        };
        if status.success() {
            info!("Path straightening succeeded");
        } else {
            warn!("Path straightening failed (exit status {status})");
        }

        // Read the straightened paths back in.
        let paths = match std::fs::read_to_string(&output_path) {
            Ok(paths) => paths,
            Err(err) => {
                warn!("Failed to read {}: {err:?}", output_path.display());
                return None;
            }
        };
        let mut lines = paths.lines();

        let mut mesh = solution_clone
            .layout
            .as_ref()
            .unwrap()
            .granulated_mesh
            .clone();

        while let Some(line) = lines.next() {
            // Skip empty line
            if line.trim().is_empty() {
                continue;
            }
            // Next line should end with an integer: the number of path entries.
            let Some(n) = line
                .split_whitespace()
                .last()
                .and_then(|s| s.parse::<usize>().ok())
            else {
                warn!("Malformed path header in straightening output: {line:?}");
                return None;
            };

            for _ in 0..n {
                let Some(line) = lines.next() else {
                    continue;
                };
                // Entries are formatted either as:
                // - v INDEX_A
                // - e INDEX_A INDEX_B T_VALUE, positioned at T_VALUE from INDEX_A to INDEX_B
                let parts: Vec<&str> = line.split_whitespace().collect();
                match parts.as_slice() {
                    ["v", _index] => {
                        // The vertex already exists in the mesh; nothing to do.
                    }
                    ["e", start, end, t_value] => {
                        let (Ok(start), Ok(end), Ok(t_value)) = (
                            start.parse::<usize>(),
                            end.parse::<usize>(),
                            t_value.parse::<f64>(),
                        ) else {
                            warn!("Malformed edge entry in straightening output: {line:?}");
                            continue;
                        };
                        let (start, end) = (start + 1, end + 1);
                        if t_value < 0.001 || t_value > 0.999 {
                            continue;
                        }
                        if let (Some(&start_vert), Some(&end_vert)) =
                            (vertex_map.key(start), vertex_map.key(end))
                        {
                            let start_pos = mesh.position(start_vert);
                            let end_pos = mesh.position(end_vert);
                            let position = start_pos.lerp(&end_pos, t_value);

                            // Split the edge at value t.
                            if let Some((edge_id, _)) =
                                mesh.edge_between_verts(start_vert, end_vert)
                            {
                                let new_vert_id = mesh.split_edge(edge_id).0;
                                mesh.set_position(new_vert_id, position);
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        let layout = solution_clone.layout.as_mut().unwrap();
        debug!(
            "Granulated mesh: {} verts before, {} after straightening",
            layout.granulated_mesh.nr_verts(),
            mesh.nr_verts()
        );
        layout.granulated_mesh = mesh;

        // Re-place the paths and patches on the updated mesh.
        loop {
            if let Err(err) = layout.place_all_paths() {
                warn!("Failed to place paths, retrying: {err:?}");
                continue;
            }
            if let Err(err) = layout.assign_all_patches() {
                warn!("Failed to assign patches, retrying: {err:?}");
                continue;
            }
            break;
        }
    }

    Some(JobResult::StageCompleted {
        stage: Stage::Layout,
        solution: solution_clone,
        configuration: configuration.clone(),
    })
}
