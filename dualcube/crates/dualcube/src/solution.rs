//! The [`Solution`] type: a compatibility facade over the explicit pipeline
//! phases.
//!
//! A solution owns the current editable state used by the UI/IO layer. The
//! actual pipeline data model is represented by [`InputPhase`], [`FlowPhase`],
//! [`DualPhase`], and [`PrimalPhase`]. A solution lazily derives the downstream
//! representations:
//!
//! 1. flow fields and graphs ([`crate::flow`]),
//! 2. dual loops ([`crate::loops`]),
//! 3. the dual structure ([`crate::dual`]),
//! 4. the layout / embedding ([`crate::layout`]),
//! 5. the polycube ([`crate::polycube`]),
//! 6. the quad mesh ([`crate::quad`]),
//! 7. the hex mesh ([`crate::hex`]).
//!
//! The loop bookkeeping and tracing methods live in [`crate::loops`]; the flow
//! graph construction lives in [`crate::flow::graph`].

use crate::prelude::*;
use rand::seq::IteratorRandom;
use serde::{Deserialize, Serialize};
use slotmap::SlotMap;
use std::sync::Arc;
use std::time::Instant;
use thiserror::Error;

#[derive(Error, Debug, Clone, Serialize, Deserialize)]
pub enum SolutionError {
    #[error("Something is wrong with the DUAL representation: {0}")]
    DualError(#[from] PropertyViolationError),
    #[error("Something is wrong with the PRIMAL representation: {0}")]
    PrimalError(#[from] LayoutError),
    #[error("The DUAL representation is not initialized and can therefore not be modified.")]
    NoDual,
    #[error("The PRIMAL representation is not initialized and can therefore not be modified.")]
    NoPrimal,
    #[error("The POLYCUBE representation is not initialized and can therefore not be modified.")]
    NoPolycube,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolutionPersistence {
    pub mesh_ref: Arc<Mesh<INPUT>>,
    pub loops: SlotMap<LoopID, Loop>,
    pub dual: Result<Dual, PropertyViolationError>,

    #[serde(default)]
    pub polycube: Option<Polycube>,

    #[serde(default)]
    pub layout: Option<Layout>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Solution {
    pub mesh_ref: Arc<Mesh<INPUT>>,
    pub loops: SlotMap<LoopID, Loop>,

    #[serde(skip)]
    pub(crate) occupied: ids::SecMap<EDGE, INPUT, Vec<LoopID>>,
    #[serde(skip)]
    pub last_loop: Option<LoopID>,

    pub dual: Result<Dual, PropertyViolationError>,
    pub polycube: Option<Polycube>,
    pub layout: Option<Layout>,
    pub quad: Option<Quad>,

    #[serde(skip)]
    pub flow_graphs: Option<Arc<[grapff::fixed::FixedGraph<EdgeID, f64>; 3]>>,

    #[serde(skip)]
    pub fields: Option<Fields<INPUT>>,
}

impl Clone for Solution {
    fn clone(&self) -> Self {
        Self {
            mesh_ref: self.mesh_ref.clone(),
            loops: self.loops.clone(),
            occupied: self.occupied.clone(),
            last_loop: self.last_loop,
            dual: self.dual.clone(),
            polycube: self.polycube.clone(),
            layout: self.layout.clone(),
            quad: self.quad.clone(),
            flow_graphs: self.flow_graphs.clone(),
            fields: self.fields.clone(),
        }
    }
}

impl Solution {
    pub fn to_persistence(&self) -> SolutionPersistence {
        SolutionPersistence {
            mesh_ref: self.mesh_ref.clone(),
            loops: self.loops.clone(),
            dual: self.dual.clone(),
            polycube: self.polycube.clone(),
            layout: self.layout.clone(),
        }
    }

    pub fn from_persistence(data: SolutionPersistence) -> Self {
        Self {
            fields: None,

            occupied: Loop::occupied(&data.loops),
            loops: data.loops,
            last_loop: None,

            dual: data.dual,

            polycube: data.polycube,
            layout: data.layout,

            quad: None,

            flow_graphs: None,

            mesh_ref: data.mesh_ref,
        }
    }

    pub fn clear(&mut self) {
        self.dual = Err(PropertyViolationError::default());
        self.polycube = None;
        self.layout = None;
        self.quad = None;
    }

    // ***
    // STEPS OF A SOLUTION
    // ***

    /// Create a new (empty) solution from an input mesh.
    pub fn new(mesh_ref: Arc<Mesh<INPUT>>) -> Self {
        Self {
            mesh_ref: mesh_ref.clone(),
            loops: SlotMap::with_key(),
            occupied: ids::SecMap::new(),
            dual: Err(PropertyViolationError::default()),
            polycube: None,
            layout: None,
            quad: None,
            last_loop: None,
            // Flow fields and graphs are computed lazily (see `prepare_flow`), so we
            // don't waste work building them for solutions that are only loaded or
            // reconstructed rather than initialized from scratch.
            fields: None,
            flow_graphs: None,
        }
    }

    /// Ensure the flow fields and per-axis flow graphs exist.
    ///
    /// Loop sampling traces loops through the flow graphs, which are derived from
    /// the flow fields, so these must be computed before any loop can be found.
    /// The fields are axis-guided so that the X/Y/Z directions get a consistent
    /// global meaning, which is what we want for polycube construction.
    pub fn prepare_flow(&mut self) {
        if self.flow_graphs.is_some() {
            return;
        }
        let input = InputPhase::new(self.mesh_ref.clone());
        let flow = input.compute_flow(FieldParams::default(), GraphParams::default());
        self.fields = Some(flow.fields);
        self.flow_graphs = Some(flow.flow_graphs);
    }

    /// Build the three per-axis flow graphs from the current flow fields.
    pub fn set_flow_graphs(&mut self, params: GraphParams) {
        let Some(fields) = &self.fields else {
            warn!("Cannot build flow graphs: vector fields are missing.");
            self.flow_graphs = None;
            return;
        };

        self.flow_graphs = Some(Arc::new(build_flow_graphs(&self.mesh_ref, fields, params)));
        info!("flow graphs set");
    }

    /// Initialize the loop structure by sampling per-axis loops and keeping the
    /// best valid combination.
    pub fn initialize(&mut self) {
        // Loops are sampled from the flow graphs, so make sure they are ready.
        self.prepare_flow();

        let m = |b: f64| OrderedFloat(b.powi(10));
        let s = |(p, _): (&[EdgeID], f64)| -(p.len() as f64);

        let samples = 3;
        let x_loops = self.sample_loops(samples, Direction::X, m, s);
        let y_loops = self.sample_loops(samples, Direction::Y, m, s);
        let z_loops = self.sample_loops(samples, Direction::Z, m, s);

        // Compute all n^3 combinations
        let combinations = x_loops
            .into_iter()
            .cartesian_product(y_loops)
            .cartesian_product(z_loops)
            .map(|((x, y), z)| (x, y, z))
            .collect_vec();

        let candidate_solutions = combinations
            .into_par()
            .filter_map(|(x_loop, y_loop, z_loop)| {
                let mut solution = self.clone();
                solution.add_loop(Loop {
                    edges: x_loop,
                    direction: Direction::X,
                });
                solution.add_loop(Loop {
                    edges: y_loop,
                    direction: Direction::Y,
                });
                solution.add_loop(Loop {
                    edges: z_loop,
                    direction: Direction::Z,
                });
                if solution.reconstruct_solution(false, 1).is_err() {
                    None
                } else {
                    Some(solution)
                }
            })
            .collect::<Vec<_>>();

        // Get the best solution based on quality
        if let Some(best_solution) = candidate_solutions
            .into_iter()
            .max_by_key(|solution| OrderedFloat(solution.get_quality().unwrap()))
        {
            *self = best_solution;
        }
    }

    /// Evolve the loop structure with a simple population-based search.
    pub fn evolve(
        &self,
        iterations: usize,
        pool1_size: usize,
        pool2_size: usize,
    ) -> Result<Self, SolutionError> {
        let Some(initial_quality) = self.get_quality() else {
            return Err(SolutionError::NoPrimal);
        };
        if pool1_size == 0 {
            warn!("evolve: pool1_size is 0; cannot evolve");
            return Ok(self.clone());
        }

        let started_at = Instant::now();
        let survivor_count = pool1_size.min(5);
        let mut seed = self.clone();
        seed.prepare_flow();
        let mut pool1 = vec![(seed, initial_quality); pool1_size];

        info!(
            "evolve: starting with iterations={iterations}, pool1_size={pool1_size}, pool2_size={pool2_size}, initial_quality={initial_quality}"
        );

        for iteration in 0..iterations {
            let iteration_started_at = Instant::now();

            let raw_mutations = (0..pool2_size)
                .into_par()
                .filter_map(|_| {
                    let index = rand::random_range(0..pool1.len());
                    let (sol, _) = &pool1[index];
                    sol.mutation()
                })
                .collect::<Vec<_>>();

            let raw_generated = raw_mutations.len();
            let dedup_timer = Instant::now();
            let mut seen = HashSet::new();
            let unique_mutations = raw_mutations
                .into_iter()
                .filter(|solution| seen.insert(solution.loop_signature()))
                .collect::<Vec<_>>();
            let unique_generated = unique_mutations.len();
            let dedup_ms = dedup_timer.elapsed();

            let mut pool2 = unique_mutations
                .into_par()
                .filter_map(|mut mutation| {
                    if mutation.reconstruct_solution_inner(true, 1, false).is_err() {
                        return None;
                    }
                    let quality = mutation.get_quality().unwrap_or(0.0);
                    Some((mutation, quality))
                })
                .collect::<Vec<_>>();

            let generated = pool2.len();
            let best_mutation = pool2.iter().map(|(_, q)| *q).reduce(f64::max);
            let avg_mutation = if generated == 0 {
                None
            } else {
                Some(pool2.iter().map(|(_, q)| *q).sum::<f64>() / generated as f64)
            };

            pool1.append(&mut pool2);
            pool1.sort_unstable_by(|(_, a), (_, b)| b.total_cmp(a));
            pool1.truncate(survivor_count);

            let best_in_pool = pool1.first().map(|(_, q)| *q).unwrap_or(f64::MIN);
            let worst_in_pool = pool1.last().map(|(_, q)| *q).unwrap_or(f64::MIN);

            info!(
                "evolve: iteration {}/{} raw={} unique={} valid={}/{} dedup={:?} elapsed={:?}; best_mutation={best_mutation:?}, avg_mutation={avg_mutation:?}, pool_len={}, best={best_in_pool}, worst={worst_in_pool}",
                iteration + 1,
                iterations,
                raw_generated,
                unique_generated,
                generated,
                pool2_size,
                dedup_ms,
                iteration_started_at.elapsed(),
                pool1.len()
            );
        }

        let Some((sol, quality)) = pool1.into_iter().next() else {
            return Ok(self.clone());
        };
        info!(
            "evolve: picked best solution with quality {quality} after {:?}",
            started_at.elapsed()
        );

        let mut combined = self.clone();
        combined.loops = sol.loops.clone();
        combined.occupied = sol.occupied.clone();
        combined.reconstruct_solution(false, 0)?;
        Ok(combined)
    }

    /// Construct the dual structure and the polycube from the current loops.
    pub fn construct_dual_and_polycube(&mut self) -> Result<(), PropertyViolationError> {
        self.layout = None;
        self.polycube = None;

        let dual_phase =
            DualPhase::from_loops(InputPhase::new(self.mesh_ref.clone()), self.loops.clone())?;
        let polycube = dual_phase.compute_polycube();

        if let Err(err) = validate_polycube(&polycube) {
            warn!("construct_dual_and_polycube: invalid polycube: {err:?}");
            return Err(PropertyViolationError::UnknownError);
        }

        self.dual = Ok(dual_phase.dual);
        self.polycube = Some(polycube);

        Ok(())
    }

    /// Place all polycube corners on the input mesh.
    pub fn place_corners(&mut self) -> Result<(), SolutionError> {
        if self.dual.is_err() {
            return Err(SolutionError::NoDual);
        }
        if self.polycube.is_none() {
            return Err(SolutionError::NoPolycube);
        }

        let mut layout = Layout::new(self.dual.as_ref().unwrap(), self.polycube.as_ref().unwrap());
        layout.place_all_corners();
        self.layout = Some(layout);
        Ok(())
    }

    /// Move a single corner to a new mesh vertex.
    pub fn move_corner_to(
        &mut self,
        corner: VertKey<POLYCUBE>,
        new_vertex: VertID,
    ) -> Result<(), SolutionError> {
        if self.layout.is_none() {
            return Err(SolutionError::NoPrimal);
        }

        let mut layout = self.layout.clone().unwrap();
        layout.move_corner(corner, new_vertex)?;
        self.layout = Some(layout);

        Ok(())
    }

    /// Place all polycube edge paths and assign the resulting patches.
    pub fn place_paths(&mut self) -> Result<(), SolutionError> {
        if self.layout.is_none() {
            return Err(SolutionError::NoPrimal);
        }
        let layout = self.layout.as_mut().unwrap();
        layout.place_all_paths()?;
        layout.assign_all_patches()?;
        Ok(())
    }

    /// Optimize corner placements by laplacian-shooting each corner and keeping
    /// the improved layout.
    pub fn optimize_corners(&mut self) -> Result<(), SolutionError> {
        if self.dual.is_err() {
            return Err(SolutionError::NoDual);
        }
        let dual = self.layout.as_ref().unwrap().dual_ref.clone();
        let polycube = self.layout.as_ref().unwrap().polycube_ref.clone();

        let polycube_vertices = dual
            .loop_structure
            .face_ids()
            .iter()
            .map(|f| polycube.region_to_vertex.get_by_left(f).unwrap().to_owned())
            .collect_vec();

        let mut solution_backup = self.clone();
        let mut solution_clone = self.clone();

        let vert_lookup = self.layout.as_ref().unwrap().granulated_mesh.kdtree();

        for &polycube_vertex in &polycube_vertices {
            let layout = solution_clone.layout.as_mut().unwrap();

            if layout
                .laplacian_corner_shoot(polycube_vertex, &vert_lookup)
                .is_err()
            {
                solution_clone = solution_backup.clone();
                continue;
            }

            let quality = solution_clone.get_quality().unwrap();
            solution_backup = solution_clone.clone();
            info!("optimize_corners: shot corner {polycube_vertex:?}, new quality {quality:?}");
        }

        *self = solution_clone;

        Ok(())
    }

    /// Whether the current loops induce a valid dual structure.
    pub fn dual_is_ok(&self) -> bool {
        Dual::from(self.mesh_ref.clone(), &self.loops).is_ok()
    }

    /// Rebuild the full chain (dual, polycube, layout, quad) from the loops.
    pub fn reconstruct_solution(&mut self, unit: bool, omega: usize) -> Result<(), SolutionError> {
        self.reconstruct_solution_inner(unit, omega, false)
    }

    fn reconstruct_solution_inner(
        &mut self,
        unit: bool,
        _omega: usize,
        _construct_quad: bool,
    ) -> Result<(), SolutionError> {
        let started_at = Instant::now();
        self.clear();

        if self.loops.len() < 3 {
            return Ok(());
        }

        let dual_timer = Instant::now();
        let dual_phase =
            DualPhase::from_loops(InputPhase::new(self.mesh_ref.clone()), self.loops.clone())?;
        let dual_ms = dual_timer.elapsed();

        let primal_timer = Instant::now();
        let primal = dual_phase.compute_primal(unit)?;
        let primal_ms = primal_timer.elapsed();

        self.dual = Ok(primal.dual.dual);
        self.polycube = Some(primal.polycube);
        self.layout = Some(primal.layout);
        let polycube_ms = primal_ms;
        let layout_ms = primal_ms;
        let resize_ms = std::time::Duration::ZERO;
        let layout_attempts = 0;

        info!(
            "reconstruct_solution: quality={:?} loops={} dual={:?} polycube={:?} layout={:?} layout_attempts={} resize={:?} total_before_quad={:?}",
            self.get_quality(),
            self.loops.len(),
            dual_ms,
            polycube_ms,
            layout_ms,
            layout_attempts,
            resize_ms,
            started_at.elapsed()
        );

        Ok(())
    }

    pub fn resize_polycube(&mut self, unit: bool) -> Result<(), SolutionError> {
        if self.dual.is_err() {
            return Err(SolutionError::NoDual);
        }
        let dual = self.dual.as_ref().unwrap();
        if self.polycube.is_none() {
            return Err(SolutionError::NoPolycube);
        }
        let polycube = self.polycube.as_mut().unwrap();

        if unit {
            polycube.resize(dual, None);
            Ok(())
        } else {
            if self.layout.is_none() {
                return Err(SolutionError::NoPrimal);
            }
            let layout = self.layout.as_ref().unwrap();
            polycube.resize(dual, Some(layout));
            Ok(())
        }
    }

    pub fn get_quality(&self) -> Option<f64> {
        let layout = self.layout.as_ref()?;
        let beta = 0.001;
        if let (Some(alignment), Some(orthogonality)) = (layout.alignment, layout.orthogonality) {
            Some(alignment + orthogonality - beta * self.loops.len() as f64)
        } else {
            None
        }
    }

    fn clone_loop_state(&self) -> Self {
        // Evolution mutates only loop bookkeeping. Avoid cloning any derived
        // representations; they are rebuilt once the candidate needs scoring.
        // `flow_graphs` is Arc-backed, so this only bumps a refcount and lets
        // evolved candidates keep sampling loops in later iterations.
        Self {
            mesh_ref: self.mesh_ref.clone(),
            loops: self.loops.clone(),
            occupied: self.occupied.clone(),
            last_loop: self.last_loop,
            dual: Err(PropertyViolationError::default()),
            polycube: None,
            layout: None,
            quad: None,
            flow_graphs: self.flow_graphs.clone(),
            fields: None,
        }
    }

    fn loop_signature(&self) -> Vec<(usize, Vec<EdgeID>)> {
        self.loops
            .values()
            .map(|loop_| (loop_.direction as usize, loop_.edges.clone()))
            .sorted_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.len().cmp(&b.1.len())))
            .collect_vec()
    }

    /// Produce a mutated loop-state copy of this solution by adding or removing loops.
    pub fn mutation(&self) -> Option<Self> {
        // Two types of mutation:
        // 1. Add loop(s)
        // 2. Remove loop(s)

        let m = |b: f64| OrderedFloat(b.powi(10));
        let s = |(_, s): (&[EdgeID], f64)| s;

        let timer = Instant::now();
        let mut mutated_solution = self.clone_loop_state();
        let clone_ms = timer.elapsed();
        let mut case = (rand::random::<u8>() % 2) + 1;
        let mut operation = "remove";
        let mut sampled_loops = 0usize;
        let mut sample_ms = std::time::Duration::ZERO;

        if self.loops.is_empty() {
            return None;
        }

        if self.loops.len() < 5 {
            case = 1;
        }

        match case {
            1 => {
                // Add a small batch before validating. Most mutations stay cheap,
                // but some explore coupled loops that only become valid/useful together.
                let add_count = match rand::random_range(0..10) {
                    0 => 3,
                    1..=3 => 2,
                    _ => 1,
                };

                operation = "add";
                let max_attempts = add_count * 4;
                let mut attempts = 0;
                while sampled_loops < add_count && attempts < max_attempts {
                    attempts += 1;
                    let axis = DIRECTIONS[rand::random_range(0..DIRECTIONS.len())];
                    let sample_timer = Instant::now();
                    let maybe_loop = self.sample_loops(1, axis, m, s).into_iter().next();
                    sample_ms += sample_timer.elapsed();

                    let Some(lewp) = maybe_loop else {
                        continue;
                    };

                    sampled_loops += 1;
                    mutated_solution.add_loop(Loop {
                        edges: lewp,
                        direction: axis,
                    });
                }

                if sampled_loops == 0 {
                    return None;
                }
            }
            2 => {
                // Remove loop(s)
                let loop_id = self.loops.keys().choose(&mut rand::rng()).unwrap();
                mutated_solution.del_loop(loop_id);
            }
            _ => unreachable!(),
        };

        info!(
            "mutation: operation={operation} parent_loops={} child_loops={} sampled_loops={} clone={:?} sample={:?} total={:?}",
            self.loops.len(),
            mutated_solution.loops.len(),
            sampled_loops,
            clone_ms,
            sample_ms,
            timer.elapsed()
        );

        Some(mutated_solution)
    }
}
