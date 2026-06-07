use std::f64::consts::PI;

use mehsh::prelude::{HasPosition, Mesh, Vector3D};

use crate::prelude::{EdgeID, INPUT};

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

/// Möller–Trumbore line/triangle intersection.
///
/// Returns the parameter `t` such that `origin + t · direction` lies inside
/// triangle `(a, b, c)`, or `None` if the line misses the triangle or is
/// (near-)parallel to it. `parallel_eps` is the determinant magnitude below
/// which the line is treated as parallel; choose it relative to the scale of
/// `direction`.
///
/// The returned `t` is unbounded in sign — callers clamp it to whatever range
/// they care about: `t > 0` for a forward ray, `0 < t < 1` for the segment
/// `origin → origin + direction`.
pub(super) fn moller_trumbore(
    origin: Vector3D,
    direction: Vector3D,
    a: Vector3D,
    b: Vector3D,
    c: Vector3D,
    parallel_eps: f64,
) -> Option<f64> {
    let edge1 = b - a;
    let edge2 = c - a;

    let h = direction.cross(&edge2);
    let det = edge1.dot(&h);

    // Line is parallel to the triangle plane (or nearly so).
    if det.abs() < parallel_eps {
        return None;
    }

    let inv_det = 1.0 / det;
    let s = origin - a;
    let u = s.dot(&h) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&edge1);
    let v = direction.dot(&q) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    Some(edge2.dot(&q) * inv_det)
}
