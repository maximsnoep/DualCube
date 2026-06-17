//! Jobs for manual loop edits (adding and removing dual loops).

use super::{Job, JobResult};
use crate::resources::Configuration;
use bevy::prelude::*;
use dualcube::prelude::*;
use ordered_float::OrderedFloat;

impl Job {
    /// Computes a candidate solution containing a new loop through `anchors`.
    /// The candidate is `None` if no valid loop (or solution) could be found.
    pub fn add_loop(solution: Solution, anchors: Vec<[EdgeID; 2]>, direction: Direction) -> Self {
        Self::new("adding loop", move || {
            let candidate = solution
                .construct_loop_with_anchors(&anchors, direction, |a: f64| OrderedFloat(a.powi(3)))
                .and_then(|(edges, _)| {
                    let mut candidate = solution.clone();
                    candidate.add_loop(Loop { edges, direction });
                    if candidate.reconstruct_solution(true, 0).is_ok() {
                        Some(candidate)
                    } else {
                        None
                    }
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
