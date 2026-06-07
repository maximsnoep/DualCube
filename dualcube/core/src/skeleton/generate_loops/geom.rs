//! Small, pure geometry/topology helpers shared across the loop-generation submodules.

use std::f64::consts::PI;

use mehsh::prelude::{HasPosition, Mesh, Vector3D};

use crate::{
    prelude::{EdgeID, PrincipalDirection, INPUT},
    skeleton::{boundary_loop::BoundaryLoop, orthogonalize::AxisSign},
    solutions::Loop,
};

pub(super) const ALL_DIRS: [PrincipalDirection; 3] = [
    PrincipalDirection::X,
    PrincipalDirection::Y,
    PrincipalDirection::Z,
];
pub(super) const ALL_SIGNS: [AxisSign; 2] = [AxisSign::Positive, AxisSign::Negative];

/// Position of an edge's midpoint.
pub(super) fn edge_midpoint_pos(e: EdgeID, mesh: &Mesh<INPUT>) -> Vector3D {
    let a = mesh.position(mesh.root(e));
    let b = mesh.position(mesh.toor(e));
    (a + b) * 0.5
}

/// Shortest angular distance between two angles in radians.
pub(super) fn angle_distance(a: f64, b: f64) -> f64 {
    let mut d = (a - b) % (2.0 * PI);
    if d > PI {
        d -= 2.0 * PI;
    } else if d < -PI {
        d += 2.0 * PI;
    }
    d.abs()
}

pub(super) fn get_loop(boundary: BoundaryLoop, direction: PrincipalDirection) -> Loop {
    Loop {
        edges: boundary.edge_midpoints,
        direction,
    }
}

/// Returns the unique direction that is neither `a` nor `b`.
pub(super) fn third(a: PrincipalDirection, b: PrincipalDirection) -> PrincipalDirection {
    match (a, b) {
        (PrincipalDirection::X, PrincipalDirection::Y)
        | (PrincipalDirection::Y, PrincipalDirection::X) => PrincipalDirection::Z,
        (PrincipalDirection::X, PrincipalDirection::Z)
        | (PrincipalDirection::Z, PrincipalDirection::X) => PrincipalDirection::Y,
        (PrincipalDirection::Y, PrincipalDirection::Z)
        | (PrincipalDirection::Z, PrincipalDirection::Y) => PrincipalDirection::X,
        _ => panic!("directions must be different"),
    }
}
