use std::sync::Arc;

use log::{info, warn};
use mehsh::prelude::Mesh;
use petgraph::graph::NodeIndex;
use serde::{Deserialize, Serialize};

use crate::{
    prelude::{INPUT, Polycube},
    quad::Quad,
    skeleton::{
        connectivity_surgery::{FailedSurgeryDiagnostic, extract_skeleton},
        contraction::{CONTRACTION, contract_mesh},
        curve_skeleton::{CurveSkeleton, CurveSkeletonSpatial},
        manipulation::remove_skeleton_node,
        orthogonalize::{
            LabeledCurveSkeleton, backtracking_orthogonalization,
            backtracking_orthogonalization_with_subdivisions, greedy_orthogonalization,
        },
        simplify::{convexify, embeddability::make_embedding_possible, simplify_skeleton},
        voxelize::generate_polycube,
    },
};

pub mod curve_skeleton;
pub mod orthogonalize;
pub mod generate_loops;

mod boundary_loop;
mod connectivity_surgery;
mod contraction;
mod geometry;
mod manipulation;
mod patch;
mod simplify;
mod voxelize;

/// Holds all relevant information for skeleton-based polycube initialization.
///
/// Fields will be gradually filled as computation proceeds.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SkeletonData {
    /// A contracted version of the input mesh.
    contraction_mesh: Arc<Mesh<CONTRACTION>>,

    /// The extracted curve skeleton for the input mesh, with induced surface patches,
    /// directly from connectivity surgery.
    raw_curve_skeleton: Option<CurveSkeleton>,

    /// Simplified version of the raw curve skeleton.
    cleaned_skeleton: Option<CurveSkeleton>,

    /// The orthogonalized and labeled curve skeleton:
    ///  - Each node has an unique integer location,
    ///  - Each edge has a direction and length.
    labeled_skeleton: Option<LabeledCurveSkeleton>,

    /// A skeleton isomorphic to `labeled_skeleton`, but with node positions and patch vertices updated to match the polycube structure.
    polycube_skeleton: Option<LabeledCurveSkeleton>,

    /// Diagnostic bundle produced when connectivity surgery couldn't reduce
    /// away every face. Contains both the partial skeleton (mangled — empty
    /// boundary loops) and the world-space triangle positions of every face
    /// still in the stuck-state simplicial complex. Set only on failure;
    /// intended purely for visual inspection so the user can see what
    /// surgery got stuck on.
    failed_surgery: Option<FailedSurgeryDiagnostic>,

    /// Genus of the input surface, as derived by connectivity surgery's
    /// topology preprocessing (`target_g`). `None` when preprocessing
    /// rejected the input (non-manifold / disconnected) so no genus was
    /// computed.
    #[serde(default)]
    genus: Option<usize>,
}

impl SkeletonData {
    /// Returns a reference to the contracted mesh.
    pub fn contraction_mesh(&self) -> &Mesh<CONTRACTION> {
        &self.contraction_mesh
    }

    /// Returns a reference to the curve skeleton if it has been computed.
    pub fn curve_skeleton(&self) -> Option<&CurveSkeleton> {
        self.raw_curve_skeleton.as_ref()
    }

    /// Returns a reference to the cleaned skeleton if it has been computed.
    pub fn cleaned_skeleton(&self) -> Option<&CurveSkeleton> {
        self.cleaned_skeleton.as_ref()
    }

    /// Returns a reference to the labeled skeleton if it has been computed.
    pub fn labeled_skeleton(&self) -> Option<&LabeledCurveSkeleton> {
        self.labeled_skeleton.as_ref()
    }

    /// Returns a reference to the polycube skeleton if it has been computed.
    pub fn polycube_skeleton(&self) -> Option<&LabeledCurveSkeleton> {
        self.polycube_skeleton.as_ref()
    }

    /// Returns the failed-surgery diagnostic bundle (partial skeleton + the
    /// world-space positions of every face still left in the stuck state).
    /// `Some` only when the most recent surgery couldn't reduce away every
    /// face; intended purely for diagnostic visualization.
    pub fn failed_surgery(&self) -> Option<&FailedSurgeryDiagnostic> {
        self.failed_surgery.as_ref()
    }

    /// Genus of the input surface as derived by connectivity surgery, or
    /// `None` if topology preprocessing rejected the input.
    pub fn genus(&self) -> Option<usize> {
        self.genus
    }

    pub fn update_convexity(
        &mut self,
        mesh: Arc<Mesh<INPUT>>,
        convexity_threshold: f64,
        convexity_merge_threshold: f64,
        omega: usize,
        refine_embedding: bool,
    ) -> (Option<Polycube>, Option<Quad>) {
        // Reuse contraction
        let (curve_skeleton, mut cleaned_skeleton, failed_surgery, genus) =
            surgery_and_simplification(&mesh, &self.contraction_mesh, refine_embedding);

        // Reuse pipeline post simplifcation
        let (labeled, polycube_and_skeleton) = post_simplification_stage(
            mesh,
            convexity_threshold,
            convexity_merge_threshold,
            &mut cleaned_skeleton,
            omega,
            refine_embedding,
        );

        let (polycube, polycube_skeleton, quad) = match polycube_and_skeleton {
            Some((p, s, q)) => (Some(p), Some(s), Some(q)),
            None => (None, None, None),
        };

        // TODO: path based simple mapping

        self.raw_curve_skeleton = Some(curve_skeleton); // Not updated now, but we calculate it anyways so might as well save it
        self.cleaned_skeleton = Some(cleaned_skeleton);
        self.labeled_skeleton = labeled;
        self.polycube_skeleton = polycube_skeleton;
        self.failed_surgery = failed_surgery;
        self.genus = genus;

        (polycube, quad)
    }

    /// Like [`retry_orthogonalization_backtracking`](Self::retry_orthogonalization_backtracking)
    /// but caps the DFS at `max_dfs_calls_per_round` recursive calls. If the round fails,
    /// subdivides one edge per cycle in the cleaned skeleton (mutating it) and retries, up
    /// to `max_rounds` times. Useful when straight backtracking would time out on a tangled
    /// skeleton — subdividing relaxes per-cycle constraints by adding edges.
    pub fn retry_orthogonalization_with_subdivisions(
        &mut self,
        mesh: Arc<Mesh<INPUT>>,
        omega: usize,
        max_dfs_calls_per_round: u64,
        max_rounds: usize,
    ) -> (Option<Polycube>, Option<Quad>) {
        let Some(cleaned) = self.cleaned_skeleton.as_mut() else {
            return (None, None);
        };

        let labeled = backtracking_orthogonalization_with_subdivisions(
            cleaned,
            &mesh,
            max_dfs_calls_per_round,
            max_rounds,
        );
        match &labeled {
            Some(_) => info!("Backtracking with subdivisions: orthogonalization successful."),
            None => warn!("Backtracking with subdivisions: orthogonalization failed."),
        }

        let polycube_and_skeleton = labeled
            .as_ref()
            .map(|labeled| generate_polycube(labeled, omega));

        let (polycube, polycube_skeleton, quad) = match polycube_and_skeleton {
            Some((p, s, q)) => (Some(p), Some(s), Some(q)),
            None => (None, None, None),
        };

        self.labeled_skeleton = labeled;
        self.polycube_skeleton = polycube_skeleton;

        (polycube, quad)
    }

    /// Re-runs orthogonalization on the existing cleaned skeleton using the slow backtracking
    /// search, then re-voxelizes if it succeeded. Used as a manual retry when greedy fails.
    /// Returns `(None, None)` if there is no cleaned skeleton yet or if backtracking also fails.
    pub fn retry_orthogonalization_backtracking(
        &mut self,
        mesh: Arc<Mesh<INPUT>>,
        omega: usize,
    ) -> (Option<Polycube>, Option<Quad>) {
        let Some(cleaned) = self.cleaned_skeleton.as_ref() else {
            return (None, None);
        };

        let labeled = backtracking_orthogonalization(cleaned, &mesh);
        match &labeled {
            Some(_) => info!("Backtracking orthogonalization successful."),
            None => warn!("Backtracking orthogonalization failed."),
        }

        let polycube_and_skeleton = labeled
            .as_ref()
            .map(|labeled| generate_polycube(labeled, omega));

        let (polycube, polycube_skeleton, quad) = match polycube_and_skeleton {
            Some((p, s, q)) => (Some(p), Some(s), Some(q)),
            None => (None, None, None),
        };

        self.labeled_skeleton = labeled;
        self.polycube_skeleton = polycube_skeleton;

        (polycube, quad)
    }

    /// Removes the specified skeleton nodes (by raw `NodeIndex` value) from the cleaned
    /// skeleton and reruns everything from convexification onwards.
    pub fn manually_remove_nodes(
        &mut self,
        nodes_to_remove: &[usize],
        mesh: Arc<Mesh<INPUT>>,
        convexity_threshold: f64,
        convexity_merge_threshold: f64,
        omega: usize,
        refine_embedding: bool,
    ) -> (Option<Polycube>, Option<Quad>) {
        let Some(cleaned) = &self.cleaned_skeleton else {
            return (None, None);
        };

        let mut skeleton = cleaned.clone();

        for &raw_idx in nodes_to_remove {
            let node_index = NodeIndex::new(raw_idx);
            if skeleton.node_weight(node_index).is_some() {
                remove_skeleton_node(&mut skeleton, node_index, &mesh);
            }
        }

        let (labeled, polycube_and_skeleton) = post_simplification_stage(
            mesh,
            convexity_threshold,
            convexity_merge_threshold,
            &mut skeleton,
            omega,
            refine_embedding,
        );

        let (polycube, polycube_skeleton, quad) = match polycube_and_skeleton {
            Some((p, s, q)) => (Some(p), Some(s), Some(q)),
            None => (None, None, None),
        };

        self.cleaned_skeleton = Some(skeleton);
        self.labeled_skeleton = labeled;
        self.polycube_skeleton = polycube_skeleton;

        (polycube, quad)
    }
}

/// Generates a polycube and a homeomorphism between the input mesh and the polycube,
/// using skeletonization.
pub fn get_skeleton_based_mapping(
    mesh: Arc<Mesh<INPUT>>,
    convexity_threshold: f64,
    convexity_merge_threshold: f64,
    omega: usize,
    refine_embedding: bool,
) -> (SkeletonData, Option<Polycube>, Option<Quad>) {
    // Start by doing contraction
    let contracted_mesh = contract_mesh(&mesh, 50);

    let (raw_curve_skeleton, mut cleaned_skeleton, failed_surgery, genus) =
        surgery_and_simplification(&mesh, &contracted_mesh, refine_embedding);

    let (labeled, polycube_and_skeleton) = post_simplification_stage(
        mesh,
        convexity_threshold,
        convexity_merge_threshold,
        &mut cleaned_skeleton,
        omega,
        refine_embedding,
    );

    let (polycube, polycube_skeleton, quad) = match polycube_and_skeleton {
        Some((p, s, q)) => (Some(p), Some(s), Some(q)),
        None => (None, None, None),
    };

    // TODO: path based simple mapping

    (
        SkeletonData {
            contraction_mesh: Arc::new(contracted_mesh),
            raw_curve_skeleton: Some(raw_curve_skeleton),
            cleaned_skeleton: Some(cleaned_skeleton),
            labeled_skeleton: labeled,
            polycube_skeleton,
            failed_surgery,
            genus,
        },
        polycube,
        quad,
    )
}

/// Returns `(raw, cleaned, failed_surgery_diagnostic)`. `raw` and `cleaned`
/// are always populated — they're empty `CurveSkeleton`s when surgery
/// failed, which lets every downstream stage no-op trivially without
/// needing a per-stage guard. The optional third value carries the mangled
/// partial state from a failed surgery, intended purely for visual
/// inspection.
fn surgery_and_simplification(
    mesh: &Arc<Mesh<INPUT>>,
    contracted_mesh: &Mesh<CONTRACTION>,
    refine_embedding: bool,
) -> (CurveSkeleton, CurveSkeleton, Option<FailedSurgeryDiagnostic>, Option<usize>) {
    let (curve_skeleton, failed_surgery, genus) =
        extract_skeleton(contracted_mesh, mesh, refine_embedding);

    let mut cleaned_skeleton = curve_skeleton.clone();
    simplify_skeleton(&mut cleaned_skeleton, mesh);
    // Smooth region boundaries
    // cleaned_skeleton.smooth_boundaries(mesh); // This is not really necessary...

    (curve_skeleton, cleaned_skeleton, failed_surgery, genus)
}

/// The decomposed part of the skeletonization process that happens after simplification.
fn post_simplification_stage(
    mesh: Arc<Mesh<INPUT>>,
    convexity_threshold: f64,
    convexity_merge_threshold: f64,
    cleaned_skeleton: &mut CurveSkeleton,
    omega: usize,
    refine_embedding: bool,
) -> (
    Option<LabeledCurveSkeleton>,
    Option<(Polycube, LabeledCurveSkeleton, Quad)>,
) {
    // Convexify skeleton to make patch volume close to convex shapes, which map nicely to cubes.
    convexify(
        cleaned_skeleton,
        &mesh,
        convexity_threshold,
        convexity_merge_threshold,
    );

    // Fix necessary conditions for orthogonal embeddability, most of the times this changes nothing.
    make_embedding_possible(cleaned_skeleton, &mesh);

    // Before labeling (which uses geometric node position), refine position again.
    // Positions likely are not accurate anymore after merges and splits and such.
    if refine_embedding {
        cleaned_skeleton.refine_embeddings(&mesh);
    }

    // Orthogonalize the curve skeleton
    let labeled = greedy_orthogonalization(&*cleaned_skeleton, &mesh);
    match &labeled {
        Some(_) => {
            info!("Orthogonalization successful.");
        }
        None => {
            warn!("Orthogonalization failed.");
        }
    }

    // Generate polycube based on labeled skeleton. Skip on an empty
    // skeleton (which is what surgery produces on failure) — generate_polycube
    // panics when handed a zero-face polycube mesh.
    let polycube: Option<(Polycube, LabeledCurveSkeleton, Quad)> = match &labeled {
        Some(labeled) if labeled.node_count() > 0 => Some(generate_polycube(labeled, omega)),
        _ => None,
    };

    // Create the mapping between input mesh and polycube
    // TODO: mapping
    (labeled, polycube)
}


