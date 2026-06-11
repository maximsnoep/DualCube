//! Jobs for manual loop edits (adding and removing dual loops).

use super::{Job, JobResult};
use crate::resources::Configuration;
use bevy::prelude::*;
use dualcube::prelude::*;
use dualcube::solutions::{Loop, LoopID};
use ordered_float::OrderedFloat;

impl Job {
    /// Computes a candidate solution containing a new loop through `anchors`.
    /// The candidate is `None` if no valid loop (or solution) could be found.
    pub fn add_loop(
        solution: Solution,
        anchors: Vec<[EdgeID; 2]>,
        direction: PrincipalDirection,
        flowgraph: grapff::fixed::FixedGraph<EdgeID, f64>,
    ) -> Self {
        Self::new("adding loop", move || {
            let candidate = solution
                .construct_loop_with_anchors(&anchors, direction, &flowgraph, |a: f64| {
                    OrderedFloat(a.powi(3))
                })
                .and_then(|(edges, _)| {
                    let mut candidate = solution.clone();
                    candidate.add_loop(Loop { edges, direction });
                    // Once enough loops are present, the dual must be reconstructable.
                    if candidate.loops.len() >= 14 {
                        if let Err(err) = candidate.construct_dual_and_polycube() {
                            warn!("Failed to reconstruct solution: {err:?}");
                            return None;
                        }
                    }
                    Some(candidate)
                });

            Some(JobResult::AddedLoop {
                anchors: anchors.clone(),
                direction,
                solution: candidate,
            })
        })
    }

    /// Removes a loop. `_force` is reserved: force removal even if the
    /// solution cannot be reconstructed.
    pub fn remove_loop(
        solution: Solution,
        loop_id: LoopID,
        _force: bool,
        configuration: Configuration,
    ) -> Self {
        Self::new("removing loop", move || {
            let mut candidate = solution.clone();
            candidate.del_loop(loop_id);
            Some(JobResult::RemovedLoop {
                solution: candidate,
                configuration: configuration.clone(),
            })
        })
    }
}
