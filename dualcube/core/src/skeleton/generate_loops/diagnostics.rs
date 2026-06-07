//! Diagnostics about loops that could not be routed, surfaced to the GUI overlay.

use crate::prelude::{EdgeID, PrincipalDirection};

/// Diagnostics about loops that could not be routed, for GUI visualization. Lets the user
/// see WHICH loops were dropped, how far they got, and WHERE a segment failed to connect.
/// Recomputed each `generate_loops` call; not persisted.
#[derive(Debug, Clone, Default)]
pub struct RoutingDiagnostics {
    /// Loops dropped because at least one segment failed to route. Each entry is the loop's
    /// axis and the raw single-half-edge path it had built before being abandoned (segments
    /// that did route are present; failed segments appear as gaps / straight chords).
    pub dropped_loops: Vec<(PrincipalDirection, Vec<EdgeID>)>,
    /// Control-point pairs `(src, tgt)` that a segment's Dijkstra could not connect — the
    /// gaps where routing got stuck.
    pub failed_segments: Vec<(EdgeID, EdgeID)>,
    /// Per failed segment, the `(src, tgt, axis)` of the gap, so the GUI can draw it in the failing
    /// loop's own direction colour.
    pub blocked_failures: Vec<BlockedFailure>,
}

/// One routing failure (a gap the router could not connect), tagged with the failing loop's axis so
/// the GUI overlay can colour it by direction.
#[derive(Debug, Clone)]
pub struct BlockedFailure {
    pub src: EdgeID,
    pub tgt: EdgeID,
    pub axis: PrincipalDirection,
}
