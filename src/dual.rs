use crate::Configuration;
use crate::EdgeWithDirection;
use crate::HasDirection;
use crate::LoopScoring;
use crate::Rules;
use bevy::prelude::*;
use douconel::douconel::{Douconel, EdgeID, FaceID, VertID};
use douconel::douconel_embedded::EmbeddedVertex;
use douconel::douconel_embedded::HasNormal;
use douconel::douconel_embedded::HasPosition;
use itertools::Itertools;
use petgraph::algo::astar;
use petgraph::graphmap::DiGraphMap;
use petgraph::graphmap::UnGraphMap;
use petgraph::visit::EdgeRef;
use potpoursi::math::convert_3d_to_2d;
use potpoursi::math::intersection_exact_in_2d;
use potpoursi::math::intersection_in_sequence;
use potpoursi::math::set_intersection;
use priq::PriorityQueue;
use rand::seq::SliceRandom;
use rand::Rng;
use rayon::prelude::*;
use serde::Deserialize;
use serde::Serialize;
use slotmap::SecondaryMap;
use std::cmp::Reverse;
use std::{
    collections::{HashMap, HashSet},
    mem::swap,
};

#[derive(Default, Clone, Debug, Serialize, Deserialize)]
pub struct Vertex {
    pub some_edge: Option<usize>,

    // Auxiliary data
    pub position: Vec3,
    pub normal: Vec3,
    pub original_face_id: FaceID,
    pub ordering: Vec<(usize, EdgeID)>,
}

#[derive(Default, Clone, Debug, Serialize, Deserialize)]
pub struct Face {
    pub some_edge: Option<usize>,

    // Auxiliary data
    pub color: Color,
    pub normal: Vec3,
    pub dual_position: Option<Vec3>,
    pub dual_normal: Option<Vec3>,
    pub original_face: Option<usize>,
}

#[derive(Default, Clone, Debug, Serialize, Deserialize)]
pub struct Edge {
    pub root: usize,
    pub face: Option<usize>,
    pub next: Option<usize>,
    pub twin: Option<usize>,

    // Auxiliary data
    pub label: Option<usize>,
    pub part_of_path: Option<usize>,
    pub edges_between: Option<Vec<EdgeID>>,
    pub edges_between_endpoints: Option<(usize, usize)>,
    pub direction: Option<PrincipalDirection>,
}

// Dual structure is an "ABSTRACT PATH REPRESENTATION"
// TODO: turn this into its own struct, then DUAL inherits this
#[derive(Clone, Default, Debug)]
pub struct Dual {
    pub paths: Vec<Path>,

    pub loop_graph: Douconel<EmbeddedVertex, EdgeWithDirection, ()>,

    // f32  =  1.0 -> incoming path
    //      = -1.0 -> outgoing path
    //pub paths_passing_per_edge: Vec<Vec<(usize, f32)>>,
    pub edge_to_paths: SecondaryMap<EdgeID, Vec<(usize, f32)>>,
}

impl Dual {}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Path {
    // A path is defined by a sequence of half-edges.
    pub edges: Vec<EdgeID>,

    // TODO: should be auxiliary data ;)
    // the direction or labeling associated with the path
    pub direction: PrincipalDirection,

    // token that is used to order all paths going through the same edge
    pub order_token: f32,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Hash, Default, Serialize, Deserialize)]
pub enum PrincipalDirection {
    #[default]
    X = 0,
    Y = 1,
    Z = 2,
}
impl PrincipalDirection {
    pub fn to_vector(&self) -> Vec3 {
        match self {
            PrincipalDirection::X => Vec3::new(1., 0., 0.),
            PrincipalDirection::Y => Vec3::new(0., 1., 0.),
            PrincipalDirection::Z => Vec3::new(0., 0., 1.),
        }
    }
}
