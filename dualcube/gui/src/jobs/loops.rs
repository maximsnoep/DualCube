//! Jobs for manual loop edits (adding and removing dual loops).

use super::{Job, JobResult};
use crate::resources::Configuration;

use dualcube::prelude::*;

impl Job {
    /// Computes a candidate solution containing a new loop through `anchors`.
    /// The candidate is `None` if no valid loop (or solution) could be found.
    pub fn add_loop(solution: Solution, loop_to_add: Loop, configuration: Configuration) -> Self {
        Self::new("adding loop", move || {
            let mut candidate = solution.clone();
            candidate.add_loop(loop_to_add.clone());
            if candidate.dual_is_ok() {
                Some(JobResult::AddedLoop {
                    solution: candidate,
                    configuration: configuration.clone(),
                })
            } else {
                None
            }
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
