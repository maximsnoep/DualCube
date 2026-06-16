//! Background jobs: definitions, submission, execution, and result handling.
//!
//! To add a new job, write a single constructor in the matching submodule
//! (or a new one):
//!
//! ```ignore
//! impl Job {
//!     pub fn my_job(solution: Solution) -> Self {
//!         Self::new("doing my thing", move || {
//!             // ... work with the captured data ...
//!             None // or Some(JobResult::...)
//!         })
//!     }
//! }
//! ```
//!
//! Only if it produces a new *kind* of result do you also add a [`JobResult`]
//! variant and handle it in [`poll_jobs`].

mod io;
mod loops;
mod path_straightening;
mod pipeline;

use crate::render::store::RenderObjectStore;
use crate::resources::{Configuration, InputResource, SolutionResource};
use bevy::prelude::*;
use bevy::tasks::futures_lite::future;
use bevy::tasks::{AsyncComputeTaskPool, Task};
use dualcube::prelude::*;
use pipeline::Stage;
use std::sync::Arc;

pub struct JobPlugin;

impl Plugin for JobPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JobState>()
            .add_message::<JobRequest>()
            .add_systems(
                Update,
                (
                    submit_jobs,
                    poll_jobs.run_if(bevy::time::common_conditions::on_timer(
                        std::time::Duration::from_millis(10),
                    )),
                ),
            );
    }
}

#[derive(Message)]
pub enum JobRequest {
    Run(Job),
    /// Reserved for cancelling the running job (not emitted anywhere yet).
    #[allow(dead_code)]
    Cancel,
}

/// Singleton job state. At most one job runs at a time; `request` holds the
/// description of the running job (and doubles as the busy flag).
#[derive(Resource, Default)]
pub struct JobState {
    pub request: Option<&'static str>,
    current: Option<Task<Option<JobResult>>>,
}

/// A unit of background work: a description (shown in the UI while running)
/// and the closure that does the work on the worker thread.
///
/// Jobs are created through the constructors in the submodules, e.g.
/// [`Job::import`], [`Job::compute_dual`], [`Job::add_loop`].
#[derive(Clone)]
pub struct Job {
    description: &'static str,
    run: Arc<dyn Fn() -> Option<JobResult> + Send + Sync>,
}

impl Job {
    fn new(
        description: &'static str,
        run: impl Fn() -> Option<JobResult> + Send + Sync + 'static,
    ) -> Self {
        Self {
            description,
            run: Arc::new(run),
        }
    }
}

/// What a finished job hands back to [`poll_jobs`].
enum JobResult {
    /// A solution was imported; resets the input resources.
    Imported { solution: Solution },
    /// A pipeline stage finished; the stage decides which job runs next.
    StageCompleted {
        stage: Stage,
        solution: Solution,
        configuration: Configuration,
    },
    /// New render objects are ready to be displayed.
    Refreshed(RenderObjectStore),
    /// A candidate loop was computed (or failed: `solution` is `None`).
    #[allow(dead_code)]
    AddedLoop {
        anchors: Vec<[EdgeID; 2]>,
        direction: Direction,
        solution: Option<Solution>,
    },
    /// A loop was removed.
    RemovedLoop {
        solution: Solution,
        configuration: Configuration,
    },
}

/// Submits jobs to the worker thread (only if idle).
fn submit_jobs(mut ev_reader: MessageReader<JobRequest>, mut job_state: ResMut<JobState>) {
    for ev in ev_reader.read() {
        match (ev, job_state.request) {
            (JobRequest::Run(job), None) => {
                info!("Starting job: {}", job.description);

                job_state.request = Some(job.description);
                let job = job.clone();
                let task = AsyncComputeTaskPool::get().spawn(async move { (job.run)() });
                job_state.current = Some(task);
            }
            (JobRequest::Cancel, Some(job)) => {
                info!("Cancelling job: {}", job);
                job_state.request = None;
                // Cancel the thread
                if let Some(task) = job_state.current.take() {
                    let future = task.cancel();
                }
            }
            _ => {}
        }
    }
}

/// Polls the current job for completion and applies its result.
fn poll_jobs(
    mut job_state: ResMut<JobState>,
    mut jobs: MessageWriter<JobRequest>,
    mut input_resource: ResMut<InputResource>,
    mut solution_resource: ResMut<SolutionResource>,
    mut render_object_store: ResMut<RenderObjectStore>,
    configuration: Res<Configuration>,
) {
    let (Some(request), Some(mut task)) = (job_state.request.take(), job_state.current.take())
    else {
        return;
    };

    let Some(result) = future::block_on(future::poll_once(&mut task)) else {
        // Not finished yet; put the job back.
        job_state.request = Some(request);
        job_state.current = Some(task);
        return;
    };

    info!("Finished job: {request}");

    let Some(result) = result else {
        // Normal for jobs without a result (e.g. exports); failures have
        // already been logged by the job itself.
        debug!("Job '{request}' produced no result to apply");
        return;
    };

    match result {
        JobResult::StageCompleted {
            stage,
            solution,
            configuration,
        } => {
            solution_resource.current_solution = solution.clone();
            jobs.write(JobRequest::Run(stage.next_job(solution, configuration)));
        }

        JobResult::Imported { solution } => {
            *input_resource = InputResource::new(solution.mesh_ref.clone());
            solution_resource.current_solution = solution;
            for next in &mut solution_resource.next {
                next.clear();
            }
            jobs.write(JobRequest::Run(Job::refresh(
                solution_resource.current_solution.clone(),
                configuration.clone(),
            )));
        }

        JobResult::AddedLoop {
            anchors,
            direction,
            solution,
        } => {
            for seed in anchors {
                solution_resource.next[direction as usize].insert(seed, solution.clone());
            }
        }

        JobResult::RemovedLoop {
            solution,
            configuration,
        } => {
            solution_resource.current_solution = solution;
            for next in &mut solution_resource.next {
                next.clear();
            }
            jobs.write(JobRequest::Run(Job::compute_dual(
                solution_resource.current_solution.clone(),
                configuration,
            )));
        }

        JobResult::Refreshed(new_render_object_store) => {
            *render_object_store = new_render_object_store;
        }
    }
}
