use mehsh::prelude::*;
use orx_parallel::*;
use std::collections::HashMap;

const AXIS_FIT_EPS: f64 = 1e-12;

#[derive(Debug, Clone, Copy)]
pub struct AxisFitParams {
    /// Number of random initial frames.
    /// Keep this low. For most polycube preprocessing, the deterministic
    /// identity and normal-covariance starts are enough.
    pub random_starts: usize,

    /// Coordinate-descent iterations per start.
    pub coordinate_descent_steps: usize,

    /// Initial local rotation angle in radians.
    pub initial_angle: f64,

    /// Stop once the search angle drops below this value.
    pub min_angle: f64,

    /// Number of bins used to compress equivalent/similar face normals.
    /// Higher values preserve more detail but cost more.
    pub normal_bins: i32,
}

impl Default for AxisFitParams {
    fn default() -> Self {
        Self {
            random_starts: 0,
            coordinate_descent_steps: 24,
            initial_angle: 0.25,
            min_angle: 1e-3,
            normal_bins: 96,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AxisFitResult {
    /// Original-space fitted X axis.
    pub axis_x: Vector3D,

    /// Original-space fitted Y axis.
    pub axis_y: Vector3D,

    /// Original-space fitted Z axis.
    pub axis_z: Vector3D,

    /// Normal-alignment energy. Lower is better.
    pub energy: f64,

    /// Number of input normals before compression.
    pub raw_normal_count: usize,

    /// Number of normals after compression.
    pub compressed_normal_count: usize,
}

/// Complete mesh-level helper.
///
/// This version assumes your mesh has:
///
///     mesh.face_ids()
///     mesh.normal(face)
///     mesh.area(face)
///     mesh.position(vertex)
///     mesh.set_position(vertex, position)
///
/// If these names differ, only edit `collect_weighted_face_normals` and
/// possibly `apply_axis_fit_to_mesh`.
pub fn reorient_mesh_by_face_normal_axis_fit<T: Tag>(
    mesh: &mut Mesh<T>,
    params: AxisFitParams,
) -> Option<AxisFitResult> {
    let weighted_normals = collect_weighted_face_normals(mesh)?;
    let fit = fit_axis_system_to_normals(&weighted_normals, params)?;

    log::info!(
        "axis-fit: applying reorientation, energy = {:.6e}, raw_normals = {}, compressed_normals = {}",
        fit.energy,
        fit.raw_normal_count,
        fit.compressed_normal_count,
    );

    apply_axis_fit_to_mesh(mesh, fit);

    Some(fit)
}

/// Fits an orthonormal axis system to weighted face normals.
///
/// Objective:
///
///     sum_f w_f (1 - max(|n_f·x|, |n_f·y|, |n_f·z|))^2
///
/// This finds the orthogonal frame whose axes explain most face normals.
/// It is better than position PCA for polycube preprocessing because it fits
/// face orientations rather than shape extent.
pub fn fit_axis_system_to_normals(
    weighted_normals: &[(Vector3D, f64)],
    params: AxisFitParams,
) -> Option<AxisFitResult> {
    let raw_normal_count = weighted_normals.len();
    let normals = compress_weighted_normals(weighted_normals, params.normal_bins);

    if normals.is_empty() {
        log::warn!("axis-fit: no valid weighted normals");
        return None;
    }

    log::info!(
        "axis-fit: start, raw_normals = {}, compressed_normals = {}, random_starts = {}, descent_steps = {}, initial_angle = {:.3e}, min_angle = {:.3e}, bins = {}",
        raw_normal_count,
        normals.len(),
        params.random_starts,
        params.coordinate_descent_steps,
        params.initial_angle,
        params.min_angle,
        params.normal_bins,
    );

    let mut candidates = Vec::with_capacity(params.random_starts + 2);
    candidates.push(("identity", Frame::identity()));
    candidates.push((
        "normal_covariance",
        initial_frame_from_normal_covariance(&normals),
    ));

    for _ in 0..params.random_starts {
        candidates.push(("random", random_frame()));
    }

    let candidate_count = candidates.len();
    let mut best: Option<AxisFitResult> = None;

    for (candidate_index, (candidate_name, candidate)) in candidates.into_iter().enumerate() {
        let candidate = candidate.orthonormalized();
        let initial_energy = frame_energy(candidate, &normals);
        let frame = optimize_frame_coordinate_descent(candidate, &normals, params);
        let energy = frame_energy(frame, &normals);

        log::info!(
            "axis-fit: candidate {}/{} ({}) initial_energy = {:.6e}, final_energy = {:.6e}",
            candidate_index + 1,
            candidate_count,
            candidate_name,
            initial_energy,
            energy,
        );

        if best.map_or(true, |b| energy < b.energy) {
            best = Some(AxisFitResult {
                axis_x: frame.x,
                axis_y: frame.y,
                axis_z: frame.z,
                energy,
                raw_normal_count,
                compressed_normal_count: normals.len(),
            });
        }

        // In practice, if the deterministic starts land at the same energy,
        // random starts usually add only time.
        if params.random_starts > 0 && candidate_index >= 1 {
            if let Some(best_fit) = best {
                if (energy - best_fit.energy).abs() < 1e-8 {
                    log::info!(
                        "axis-fit: early stop after deterministic agreement, best_energy = {:.6e}",
                        best_fit.energy,
                    );
                    break;
                }
            }
        }
    }

    let Some(best) = best else {
        log::warn!("axis-fit: failed");
        return None;
    };

    log::info!(
        "axis-fit: done, energy = {:.6e}, x = {:?}, y = {:?}, z = {:?}",
        best.energy,
        best.axis_x,
        best.axis_y,
        best.axis_z,
    );

    Some(best)
}

/// Reorients all mesh vertex positions using a precomputed axis fit.
///
/// Replace `mesh.set_position(id, q)` if your position setter has another name.
pub fn apply_axis_fit_to_mesh<T: Tag>(mesh: &mut Mesh<T>, fit: AxisFitResult) {
    let ids: Vec<_> = mesh.vert_ids();

    if ids.is_empty() {
        return;
    }

    let origin = centroid(mesh, &ids);

    for id in ids {
        let p = mesh.position(id);
        let q = apply_axis_fit_to_position(p, origin, fit);
        mesh.set_position(id, q);
    }
}

/// Reorients one position into the fitted axis system.
///
/// If p is an original-space point, q is its coordinate in the fitted frame:
///
///     q.x = (p - origin) · axis_x
///     q.y = (p - origin) · axis_y
///     q.z = (p - origin) · axis_z
pub fn apply_axis_fit_to_position(p: Vector3D, origin: Vector3D, fit: AxisFitResult) -> Vector3D {
    let v = p - origin;

    Vector3D::new(v.dot(&fit.axis_x), v.dot(&fit.axis_y), v.dot(&fit.axis_z))
}

/// EDIT THIS FUNCTION if your face API differs.
///
/// You want one entry per face:
///
///     (unit face normal, face area)
///
/// Area weighting matters: large faces should influence the fitted frame more
/// than tiny triangles.
fn collect_weighted_face_normals<T: Tag>(mesh: &Mesh<T>) -> Option<Vec<(Vector3D, f64)>> {
    let mut weighted_normals = Vec::new();

    for face in mesh.face_ids() {
        let n = mesh.normal(face);
        let area = mesh.triangle_area(face);

        if area > 0.0 && n.norm() > AXIS_FIT_EPS {
            weighted_normals.push((n.normalize(), area));
        }
    }

    if weighted_normals.is_empty() {
        None
    } else {
        Some(weighted_normals)
    }
}

fn compress_weighted_normals(
    weighted_normals: &[(Vector3D, f64)],
    bins: i32,
) -> Vec<(Vector3D, f64)> {
    let bins = bins.max(8);
    let mut map: HashMap<(i32, i32, i32), (Vector3D, f64)> = HashMap::new();

    for &(n_raw, w) in weighted_normals {
        if w <= 0.0 || n_raw.norm() <= AXIS_FIT_EPS {
            continue;
        }

        let n = canonical_normal_sign(n_raw.normalize());
        let key = normal_bin_key(n, bins);

        let entry = map
            .entry(key)
            .or_insert((Vector3D::new(0.0, 0.0, 0.0), 0.0));

        entry.0 += n * w;
        entry.1 += w;
    }

    let mut compressed = Vec::with_capacity(map.len());

    for (_, (sum, weight)) in map {
        if weight > 0.0 && sum.norm() > AXIS_FIT_EPS {
            compressed.push((sum.normalize(), weight));
        }
    }

    compressed
}

/// Canonicalizes the sign because the objective uses absolute normal-axis dot
/// products. A normal and its negation are equivalent for this fit.
fn canonical_normal_sign(n: Vector3D) -> Vector3D {
    let x = n.dot(&Vector3D::new(1.0, 0.0, 0.0));
    let y = n.dot(&Vector3D::new(0.0, 1.0, 0.0));
    let z = n.dot(&Vector3D::new(0.0, 0.0, 1.0));

    let ax = x.abs();
    let ay = y.abs();
    let az = z.abs();

    let sign = if ax >= ay && ax >= az {
        x.signum()
    } else if ay >= ax && ay >= az {
        y.signum()
    } else {
        z.signum()
    };

    if sign < 0.0 {
        -n
    } else {
        n
    }
}

fn normal_bin_key(n: Vector3D, bins: i32) -> (i32, i32, i32) {
    let x = n.dot(&Vector3D::new(1.0, 0.0, 0.0));
    let y = n.dot(&Vector3D::new(0.0, 1.0, 0.0));
    let z = n.dot(&Vector3D::new(0.0, 0.0, 1.0));

    (
        (x * bins as f64).round() as i32,
        (y * bins as f64).round() as i32,
        (z * bins as f64).round() as i32,
    )
}

#[derive(Debug, Clone, Copy)]
struct Frame {
    x: Vector3D,
    y: Vector3D,
    z: Vector3D,
}

impl Frame {
    fn identity() -> Self {
        Self {
            x: Vector3D::new(1.0, 0.0, 0.0),
            y: Vector3D::new(0.0, 1.0, 0.0),
            z: Vector3D::new(0.0, 0.0, 1.0),
        }
    }

    fn orthonormalized(self) -> Self {
        let x = safe_normalize(self.x, Vector3D::new(1.0, 0.0, 0.0));
        let z = safe_normalize(x.cross(&self.y), Vector3D::new(0.0, 0.0, 1.0));
        let y = safe_normalize(z.cross(&x), Vector3D::new(0.0, 1.0, 0.0));

        Self { x, y, z }
    }
}

fn optimize_frame_coordinate_descent(
    mut frame: Frame,
    normals: &[(Vector3D, f64)],
    params: AxisFitParams,
) -> Frame {
    frame = frame.orthonormalized();

    let mut angle = params.initial_angle;
    let mut best_energy = frame_energy(frame, normals);
    let mut accepted_moves = 0usize;

    for _ in 0..params.coordinate_descent_steps {
        let old_energy = best_energy;
        let mut improved = false;

        for axis_id in 0..3 {
            for sign in [-1.0, 1.0] {
                let candidate = rotate_frame_local(frame, axis_id, sign * angle);
                let energy = frame_energy(candidate, normals);

                if energy + 1e-12 < best_energy {
                    frame = candidate;
                    best_energy = energy;
                    improved = true;
                    accepted_moves += 1;
                }
            }
        }

        let improvement = old_energy - best_energy;

        if improvement.abs() < 1e-8 || !improved {
            angle *= 0.5;
        }

        if angle < params.min_angle {
            break;
        }
    }

    log::debug!(
        "axis-fit: descent finished, energy = {:.6e}, angle = {:.3e}, accepted_moves = {}",
        best_energy,
        angle,
        accepted_moves,
    );

    frame.orthonormalized()
}

/// Rotates the whole frame around one of its own current axes.
fn rotate_frame_local(frame: Frame, axis_id: usize, angle: f64) -> Frame {
    let axis = match axis_id {
        0 => frame.x,
        1 => frame.y,
        _ => frame.z,
    };

    Frame {
        x: rotate_around_axis(frame.x, axis, angle),
        y: rotate_around_axis(frame.y, axis, angle),
        z: rotate_around_axis(frame.z, axis, angle),
    }
    .orthonormalized()
}

fn frame_energy(frame: Frame, normals: &[(Vector3D, f64)]) -> f64 {
    if normals.is_empty() {
        return 0.0;
    }

    let total_weight: f64 = normals.iter().map(|(_, w)| *w).sum();

    if total_weight <= AXIS_FIT_EPS {
        return 0.0;
    }

    let energy: f64 = (0..normals.len())
        .par()
        .map(|i| {
            let (n, w) = normals[i];

            let dx = n.dot(&frame.x).abs();
            let dy = n.dot(&frame.y).abs();
            let dz = n.dot(&frame.z).abs();

            let best = dx.max(dy).max(dz).clamp(0.0, 1.0);
            let residual = 1.0 - best;

            w * residual * residual
        })
        .sum();

    energy / total_weight
}

/// Normal-covariance initializer.
///
/// This is not the final objective, but it gives a good deterministic start.
fn initial_frame_from_normal_covariance(normals: &[(Vector3D, f64)]) -> Frame {
    let mut c = [[0.0; 3]; 3];

    for &(n, w) in normals {
        let x = n.dot(&Vector3D::new(1.0, 0.0, 0.0));
        let y = n.dot(&Vector3D::new(0.0, 1.0, 0.0));
        let z = n.dot(&Vector3D::new(0.0, 0.0, 1.0));

        c[0][0] += w * x * x;
        c[0][1] += w * x * y;
        c[0][2] += w * x * z;

        c[1][0] += w * y * x;
        c[1][1] += w * y * y;
        c[1][2] += w * y * z;

        c[2][0] += w * z * x;
        c[2][1] += w * z * y;
        c[2][2] += w * z * z;
    }

    let eigen = symmetric_eigen_3x3(c);

    let mut pairs = [
        (eigen.values[0], eigen.vectors[0]),
        (eigen.values[1], eigen.vectors[1]),
        (eigen.values[2], eigen.vectors[2]),
    ];

    pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    Frame {
        x: pairs[0].1,
        y: pairs[1].1,
        z: pairs[2].1,
    }
    .orthonormalized()
}

fn rotate_around_axis(v: Vector3D, axis: Vector3D, angle: f64) -> Vector3D {
    let a = safe_normalize(axis, Vector3D::new(1.0, 0.0, 0.0));
    let c = angle.cos();
    let s = angle.sin();

    v * c + a.cross(&v) * s + a * a.dot(&v) * (1.0 - c)
}

fn centroid<T: Tag>(mesh: &Mesh<T>, ids: &[ids::Key<VERT, T>]) -> Vector3D {
    let mut c = Vector3D::new(0.0, 0.0, 0.0);

    for &id in ids {
        c += mesh.position(id);
    }

    c / ids.len() as f64
}

fn random_frame() -> Frame {
    let x = random_unit_vector();
    let y_raw = random_unit_vector();

    let z = safe_normalize(x.cross(&y_raw), Vector3D::new(0.0, 0.0, 1.0));
    let y = safe_normalize(z.cross(&x), Vector3D::new(0.0, 1.0, 0.0));

    Frame { x, y, z }
}

#[derive(Debug, Clone, Copy)]
struct Eigen3 {
    values: [f64; 3],
    vectors: [Vector3D; 3],
}

/// Jacobi eigen-decomposition for a symmetric 3x3 matrix.
fn symmetric_eigen_3x3(mut a: [[f64; 3]; 3]) -> Eigen3 {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    for _ in 0..32 {
        let (p, q) = largest_offdiag_entry(a);

        if a[p][q].abs() < 1e-14 {
            break;
        }

        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];

        let tau = (aqq - app) / (2.0 * apq);
        let t = if tau >= 0.0 {
            1.0 / (tau + (1.0 + tau * tau).sqrt())
        } else {
            -1.0 / (-tau + (1.0 + tau * tau).sqrt())
        };

        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;

        rotate_symmetric(&mut a, p, q, c, s);
        rotate_eigenvectors(&mut v, p, q, c, s);
    }

    Eigen3 {
        values: [a[0][0], a[1][1], a[2][2]],
        vectors: [
            safe_normalize(
                Vector3D::new(v[0][0], v[1][0], v[2][0]),
                Vector3D::new(1.0, 0.0, 0.0),
            ),
            safe_normalize(
                Vector3D::new(v[0][1], v[1][1], v[2][1]),
                Vector3D::new(0.0, 1.0, 0.0),
            ),
            safe_normalize(
                Vector3D::new(v[0][2], v[1][2], v[2][2]),
                Vector3D::new(0.0, 0.0, 1.0),
            ),
        ],
    }
}

fn largest_offdiag_entry(a: [[f64; 3]; 3]) -> (usize, usize) {
    let mut p = 0;
    let mut q = 1;
    let mut best = a[0][1].abs();

    for (i, j) in [(0, 2), (1, 2)] {
        let value = a[i][j].abs();

        if value > best {
            best = value;
            p = i;
            q = j;
        }
    }

    (p, q)
}

fn rotate_symmetric(a: &mut [[f64; 3]; 3], p: usize, q: usize, c: f64, s: f64) {
    let app = a[p][p];
    let aqq = a[q][q];
    let apq = a[p][q];

    a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
    a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;

    a[p][q] = 0.0;
    a[q][p] = 0.0;

    for r in 0..3 {
        if r == p || r == q {
            continue;
        }

        let arp = a[r][p];
        let arq = a[r][q];

        a[r][p] = c * arp - s * arq;
        a[p][r] = a[r][p];

        a[r][q] = s * arp + c * arq;
        a[q][r] = a[r][q];
    }
}

fn rotate_eigenvectors(v: &mut [[f64; 3]; 3], p: usize, q: usize, c: f64, s: f64) {
    for row in v.iter_mut() {
        let vip = row[p];
        let viq = row[q];

        row[p] = c * vip - s * viq;
        row[q] = s * vip + c * viq;
    }
}

fn safe_normalize(v: Vector3D, fallback: Vector3D) -> Vector3D {
    let len = v.norm();

    if len > AXIS_FIT_EPS {
        v / len
    } else {
        let fallback_len = fallback.norm();

        if fallback_len > AXIS_FIT_EPS {
            fallback / fallback_len
        } else {
            Vector3D::new(1.0, 0.0, 0.0)
        }
    }
}

fn random_unit_vector() -> Vector3D {
    let v = Vector3D::new(
        rand::random::<f64>() - 0.5,
        rand::random::<f64>() - 0.5,
        rand::random::<f64>() - 0.5,
    );

    safe_normalize(v, Vector3D::new(1.0, 0.0, 0.0))
}
