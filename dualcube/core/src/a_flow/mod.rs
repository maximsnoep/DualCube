//! Flow fields and flow graphs.
//!
//! - [`field`] computes smooth per-vertex direction fields (one per principal
//!   axis) over the input mesh.
//! - [`graph`] turns the fields into per-axis weighted edge graphs that loops
//!   are traced through (see [`crate::loops`]).

pub mod field;
pub mod graph;

pub use field::{Field, FieldParams, Fields, GlobalVectorKey};
pub use graph::GraphParams;
