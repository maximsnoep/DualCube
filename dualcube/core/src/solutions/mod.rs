//! The [`Solution`] type: the orchestrator that ties together every stage of
//! the polycube segmentation pipeline.
//!
//! A solution owns the input mesh and the dual loops, and lazily derives the
//! downstream representations:
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

use crate::dual::{Dual, PropertyViolationError};
use crate::layout::{Layout, LayoutError};
use crate::polycube::{Polycube, POLYCUBE};
use crate::prelude::*;
use crate::quad::Quad;
use itertools::Itertools;
use mehsh::prelude::*;
use ordered_float::OrderedFloat;
use orx_parallel::*;
use rand::seq::IteratorRandom;
use serde::{Deserialize, Serialize};
use slotmap::SlotMap;
use std::sync::{Arc, RwLock};
use thiserror::Error;

pub use crate::loops::{wrap_pairs, Loop, LoopID, NodeCopy};

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
    pub flow_graphs: Option<[grapff::fixed::FixedGraph<EdgeID, f64>; 3]>,

    #[serde(skip)]
    pub(crate) available_flow_graphs:
        Arc<RwLock<Option<Arc<[grapff::fixed::FixedGraph<EdgeID, f64>; 3]>>>>,

    #[serde(skip)]
    pub fields: Option<crate::flow::Fields<INPUT>>,
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
            available_flow_graphs: Arc::new(RwLock::new(None)),
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
            available_flow_graphs: Arc::new(RwLock::new(None)),

            mesh_ref: data.mesh_ref,
        }
    }

    pub fn clear(&mut self) {
        self.dual = Err(PropertyViolationError::default());
        self.polycube = None;
        self.layout = None;
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
            available_flow_graphs: Arc::new(RwLock::new(None)),
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
        if self.fields.is_none() {
            self.fields = Some(crate::flow::Fields::from_mesh(&self.mesh_ref));
        }
        self.set_flow_graphs();
    }

    /// Initialize the loop structure by sampling per-axis loops and keeping the
    /// best valid combination.
    pub fn initialize(&mut self) {
        // Loops are sampled from the flow graphs, so make sure they are ready.
        self.prepare_flow();

        let m = |b: f64| OrderedFloat(b.powi(10));
        let s = |(p, _): (&[EdgeID], f64)| -(p.len() as f64);

        let samples = 3;
        let x_loops = self.sample_loops(samples, PrincipalDirection::X, m, s);
        let y_loops = self.sample_loops(samples, PrincipalDirection::Y, m, s);
        let z_loops = self.sample_loops(samples, PrincipalDirection::Z, m, s);

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
                    direction: PrincipalDirection::X,
                });
                solution.add_loop(Loop {
                    edges: y_loop,
                    direction: PrincipalDirection::Y,
                });
                solution.add_loop(Loop {
                    edges: z_loop,
                    direction: PrincipalDirection::Z,
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
    pub fn evolve(&self, iterations: usize, pool1_size: usize, pool2_size: usize) -> Option<Self> {
        let mut pool1 = vec![(self.clone(), self.get_quality().unwrap()); pool1_size];
        for _ in 0..iterations {
            let pool2 = (0..pool2_size)
                .into_par()
                .map(|_| {
                    // Grab a random solution from pool1
                    let index = rand::random_range(0..pool1.len());
                    pool1[index].clone()
                })
                .filter_map(|(sol, _)| {
                    // Mutate the solution
                    sol.mutation().map_or_else(
                        || None,
                        |mutation| {
                            let quality = mutation.get_quality().unwrap_or(0.0);
                            // Compute quality
                            Some((mutation, quality))
                        },
                    )
                })
                .collect::<Vec<_>>()
                .into_iter()
                .sorted_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .rev();

            for (_, quality) in pool2.clone() {
                log::debug!("evolve: mutated candidate with quality {quality}");
            }

            // overwrite pool1 with top 5 solutions of pool1 and top 5 solutions of pool2
            pool1 = pool1
                .into_iter()
                .take(5)
                .chain(pool2.take(5))
                .sorted_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .rev()
                .collect();

            let best_in_pool = pool1.iter().map(|(_, q)| *q).fold(f64::MIN, f64::max);
            log::info!(
                "evolve: pool of {} solutions, best quality so far {best_in_pool}",
                pool1.len()
            );
        }

        if pool1.is_empty() {
            log::warn!("evolve: no valid solutions generated");
            return None;
        }

        // Grab the best solution
        let (sol, quality) = pool1
            .into_iter()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();
        log::info!("evolve: picked best solution with quality {quality}");
        Some(sol)
    }

    /// Construct the dual structure and the polycube from the current loops.
    pub fn construct_dual_and_polycube(&mut self) -> Result<(), PropertyViolationError> {
        self.layout = None;
        self.polycube = None;

        self.dual = Dual::from(self.mesh_ref.clone(), &self.loops);
        if let Err(e) = &self.dual {
            return Err(e.clone());
        }

        self.polycube = Some(Polycube::from_dual(self.dual.as_ref().unwrap()));

        // check all faces of the polycube have a normal
        for face in self.polycube.as_ref().unwrap().structure.face_ids() {
            let normal = self.polycube.as_ref().unwrap().structure.normal(face);
            if normal.x.is_nan() || normal.y.is_nan() || normal.z.is_nan() {
                log::error!(
                    "construct_dual_and_polycube: polycube face {face:?} has invalid (NaN) normal {normal}"
                );
                return Err(PropertyViolationError::UnknownError);
            }
        }

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
            log::debug!(
                "optimize_corners: shot corner {polycube_vertex:?}, new quality {quality:?}"
            );
        }

        *self = solution_clone;

        Ok(())
    }

    /// Construct the quad mesh from the current layout.
    pub fn construct_quad(&mut self, omega: usize) -> Result<(), PropertyViolationError> {
        self.quad = Quad::from_layout(self.layout.as_ref().unwrap(), omega);
        Ok(())
    }

    /// Whether the current loops induce a valid dual structure.
    pub fn dual_is_ok(&self) -> bool {
        Dual::from(self.mesh_ref.clone(), &self.loops).is_ok()
    }

    /// Rebuild the full chain (dual, polycube, layout, quad) from the loops.
    pub fn reconstruct_solution(&mut self, unit: bool, omega: usize) -> Result<(), SolutionError> {
        self.clear();

        if self.loops.len() < 3 {
            return Ok(());
        }

        log::info!(
            "Reconstructing solution ({} loops) with unit: {}, omega: {}.",
            self.loops.len(),
            unit,
            omega
        );

        let dual = Dual::from(self.mesh_ref.clone(), &self.loops)?;
        let polycube = Polycube::from_dual(&dual);

        // check all faces of the polycube have a normal
        for face in polycube.structure.face_ids() {
            let normal = polycube.structure.normal(face);
            if normal.x.is_nan() || normal.y.is_nan() || normal.z.is_nan() {
                return Err(SolutionError::NoPolycube);
            }
        }

        'outer: for _ in 0..10 {
            let layout = Layout::embed(&dual, &polycube);
            if let Ok(ok_layout) = layout {
                self.layout = Some(ok_layout);
                break 'outer;
            }
        }

        if self.layout.is_none() {
            return Err(SolutionError::NoPrimal);
        }
        self.polycube = Some(polycube);
        self.dual = Ok(dual);

        self.resize_polycube(unit)?;

        log::info!(
            "The constructed solution has quality: {:?}",
            self.get_quality()
        );

        self.quad = Quad::from_layout(self.layout.as_ref().unwrap(), omega);

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

    /// Produce a mutated copy of this solution by adding or removing loops.
    pub fn mutation(&self) -> Option<Self> {
        // Two types of mutation:
        // 1. Add loop(s)
        // 2. Remove loop(s)

        let m = |b: f64| OrderedFloat(b.powi(10));
        let s = |(_, s): (&[EdgeID], f64)| s;

        let mut mutated = false;
        let mut mutated_solution = self.clone();
        let mut case = (rand::random::<u8>() % 2) + 1;

        if self.loops.is_empty() {
            return None;
        }

        if self.loops.len() < 5 {
            case = 1;
        }

        match case {
            1 => {
                // Add loop(s)
                let x = rand::random::<u8>() % 3;
                let y = rand::random::<u8>() % 3;
                let z = rand::random::<u8>() % 3;
                if x + y + z == 0 {
                    return None;
                }

                let x_loops = self
                    .sample_loops(x as usize, PrincipalDirection::X, m, s)
                    .into_iter()
                    .map(|x| (x, PrincipalDirection::X))
                    .collect_vec();
                let y_loops = self
                    .sample_loops(y as usize, PrincipalDirection::Y, m, s)
                    .into_iter()
                    .map(|y| (y, PrincipalDirection::Y))
                    .collect_vec();
                let z_loops = self
                    .sample_loops(z as usize, PrincipalDirection::Z, m, s)
                    .into_iter()
                    .map(|z| (z, PrincipalDirection::Z))
                    .collect_vec();

                // Iteratively add the loops, save result if result is valid.
                for (lewp, axis) in x_loops.into_iter().chain(y_loops).chain(z_loops) {
                    let mut candidate_solution = mutated_solution.clone();
                    candidate_solution.add_loop(Loop {
                        edges: lewp,
                        direction: axis,
                    });
                    // Check solution
                    if candidate_solution.dual_is_ok() {
                        mutated_solution = candidate_solution;
                        mutated = true;
                    }
                }
            }
            2 => {
                // Remove loop(s)
                let loop_id = self.loops.keys().choose(&mut rand::rng()).unwrap();
                let mut candidate_solution = mutated_solution.clone();
                candidate_solution.del_loop(loop_id);

                // Check solution
                if candidate_solution.dual_is_ok() {
                    mutated_solution = candidate_solution;
                    mutated = true;
                }
            }
            _ => unreachable!(),
        };

        if !mutated {
            return None;
        }

        if mutated_solution.reconstruct_solution(true, 1).is_err() {
            return None;
        }

        Some(mutated_solution)
    }
}
