use std::f64::consts::PI;

use mehsh::prelude::{HasNormal, HasPosition, Mesh, Vector3D, EPS};

use crate::prelude::{EdgeID, PrincipalDirection, INPUT};

/// The three principal axis directions as unit vectors, in X, Y, Z order.
pub(super) fn principal_axes() -> [Vector3D; 3] {
    [
        Vector3D::from(PrincipalDirection::X),
        Vector3D::from(PrincipalDirection::Y),
        Vector3D::from(PrincipalDirection::Z),
    ]
}

/// Position of an edge's midpoint.
pub(super) fn edge_midpoint_pos(e: EdgeID, mesh: &Mesh<INPUT>) -> Vector3D {
    let a = mesh.position(mesh.root(e));
    let b = mesh.position(mesh.toor(e));
    (a + b) * 0.5
}

/// Per-axis misalignment score for a closed boundary loop, used to LABEL the loop with a principal
/// axis (the axis with the LOWEST score wins). The loop is the cross-section that separates two
/// skeleton patches; at each segment its in-surface separating direction `d × n` (the edge-to-edge
/// tangent `d` rotated 90° about the surface normal `n`) points along the loop's limb axis — exactly
/// the axis we want to recover. For each axis A we accumulate, length-weighted, the misalignment
/// `θ = ∠(d × n, A-line)` raised to the same power the router uses (`θ^ALPHA`), so labeling and
/// routing share one notion of "aligned". θ is folded to `[0, π/2]` (the dot is taken in absolute
/// value) because the loop's traversal direction — hence the sign of `d × n` — is arbitrary here;
/// the SIGN of the final label is decided separately, from the skeleton-edge displacement.
///
/// Returns `Some([score_X, score_Y, score_Z])`, or `None` if the loop is too degenerate to score
/// (fewer than two edges, or every segment vanished). This replaces a least-squares plane-fit
/// normal: no eigen-decomposition, no planarity assumption (robust for wiggly loops), deterministic.
pub(super) fn boundary_loop_axis_scores(loop_edges: &[EdgeID], mesh: &Mesh<INPUT>) -> Option<[f64; 3]> {
    // Exponent on the misalignment angle — matches the router's `ALIGN_ALPHA` so labeling and the
    // routing cost agree. The chosen axis (the minimum) is robust to the exact value.
    const ALPHA: i32 = 10;
    let axes = principal_axes();
    let n = loop_edges.len();
    if n < 2 {
        return None;
    }
    let mut scores = [0.0f64; 3];
    let mut contributed = false;
    for i in 0..n {
        let m0 = edge_midpoint_pos(loop_edges[i], mesh);
        let m1 = edge_midpoint_pos(loop_edges[(i + 1) % n], mesh);
        let d = m1 - m0;
        let len = d.norm();
        if len <= EPS {
            continue;
        }
        let sep = d.cross(&mesh.normal(mesh.face(loop_edges[i])));
        let sn = sep.norm();
        if sn <= EPS {
            continue;
        }
        let sep_hat = sep / sn;
        for a in 0..3 {
            let cos = sep_hat.dot(&axes[a]).abs().clamp(0.0, 1.0);
            scores[a] += len * cos.acos().powi(ALPHA);
        }
        contributed = true;
    }
    contributed.then_some(scores)
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
