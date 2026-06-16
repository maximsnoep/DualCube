//! Flow fields.

use mehsh::prelude::*;
use num_complex::Complex64;
use orx_parallel::*;
use serde::{Deserialize, Serialize};
use slotmap::{new_key_type, SlotMap};
use std::collections::HashMap;

new_key_type! {
    pub struct GlobalVectorKey;
}

const EPS: f64 = 1e-12;

const CURVATURE_MIN_CONFIDENCE: f64 = 1e-3;
const CURVATURE_CONFIDENCE_SENSITIVITY: f64 = 2.0;
const CURVATURE_ALIGNMENT_MIN_DOT: f64 = 0.65;
const CURVATURE_ALIGNMENT_SHARPNESS: f64 = 2.0;

const REGULARIZATION: f64 = 1e-8;

const CG_STALL_MIN_ITERATIONS: usize = 8;
const CG_STALL_PATIENCE: usize = 4;
const CG_STALL_RELATIVE_IMPROVEMENT: f64 = 1e-3;

const X_AXIS: Vector3D = Vector3D::new(1.0, 0.0, 0.0);
const Y_AXIS: Vector3D = Vector3D::new(0.0, 1.0, 0.0);
const Z_AXIS: Vector3D = Vector3D::new(0.0, 0.0, 1.0);
const C_ZERO: Complex64 = Complex64::new(0.0, 0.0);
const C_ONE: Complex64 = Complex64::new(1.0, 0.0);

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct FieldParams {
    #[serde(alias = "iterations")]
    pub outer_iterations: usize,
    pub cg_iterations: usize,
    pub cg_tolerance: f64,
    pub damping_weight: f64,
    pub smooth_weight: f64,
    pub axis_weight: f64,
    pub axis_length_power: f64,
    pub curvature_weight: f64,
    pub coupling_weight: f64,
}

impl Default for FieldParams {
    fn default() -> Self {
        Self {
            outer_iterations: 100,
            cg_iterations: 500,
            cg_tolerance: 1e-4,
            damping_weight: 0.03,
            smooth_weight: 1.0,
            axis_weight: 0.5,
            axis_length_power: 3.0,
            curvature_weight: 0.5,
            coupling_weight: 0.01,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Field<T: Tag> {
    pub map: HashMap<ids::Key<VERT, T>, GlobalVectorKey>,
    pub vectors: SlotMap<GlobalVectorKey, Vector3D>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Fields<T: Tag> {
    pub field_x: Field<T>,
    pub field_y: Field<T>,
    pub field_z: Field<T>,
}

#[derive(Debug, Clone, Copy)]
struct TangentBasis {
    e1: Vector3D,
    e2: Vector3D,
}

#[derive(Debug, Clone, Copy)]
struct CurvatureData {
    dir_min: Vector3D,
    dir_max: Vector3D,
    c_min: f64,
    c_max: f64,
}

#[derive(Debug, Clone, Copy)]
struct NeighborConnection {
    j: usize,
    phase_j_to_i: Complex64,
}

#[derive(Debug, Clone)]
struct SolverData<T: Tag> {
    ids: Vec<ids::Key<VERT, T>>,
    normals: Vec<Vector3D>,
    bases: Vec<TangentBasis>,
    adjacency: Vec<Vec<NeighborConnection>>,
    curvature: Vec<Option<CurvatureData>>,
}

impl<T: Tag> Field<T> {
    pub fn new(mesh: &Mesh<T>) -> Self {
        let ids: Vec<_> = mesh.vert_ids();
        let mut field = Self {
            map: HashMap::with_capacity(ids.len()),
            vectors: SlotMap::with_key(),
        };

        for id in ids {
            let key = field.vectors.insert(random_unit_vector());
            field.map.insert(id.to_owned(), key);
        }

        field
    }

    pub fn vector_at(&self, id: ids::Key<VERT, T>) -> Option<Vector3D> {
        self.map.get(&id).map(|key| self.vectors[*key])
    }
}

fn random_unit_vector() -> Vector3D {
    Vector3D::new(
        rand::random::<f64>() - 0.5,
        rand::random::<f64>() - 0.5,
        rand::random::<f64>() - 0.5,
    )
    .normalize()
}

impl<T: Tag> Fields<T> {
    pub fn new(mesh: &Mesh<T>, params: FieldParams) -> Self {
        let mut fields = Self {
            field_x: Field::new(mesh),
            field_y: Field::new(mesh),
            field_z: Field::new(mesh),
        };

        if params.axis_weight > 0.0 {
            fields.initialize_around_axes(mesh);
        }

        let data = build_solver_data(mesh);
        fields.optimize_global_connection(&data, &params);
        fields
    }

    pub fn optimize(&mut self, mesh: &Mesh<T>, params: &FieldParams) {
        let data = build_solver_data(mesh);
        self.optimize_global_connection(&data, params);
    }

    fn initialize_around_axes(&mut self, mesh: &Mesh<T>) {
        initialize_field_around_axis(mesh, &mut self.field_x, X_AXIS);
        initialize_field_around_axis(mesh, &mut self.field_y, Y_AXIS);
        initialize_field_around_axis(mesh, &mut self.field_z, Z_AXIS);
    }

    fn optimize_global_connection(&mut self, data: &SolverData<T>, params: &FieldParams) {
        let mut z = [
            field_to_complex(&self.field_x, data),
            field_to_complex(&self.field_y, data),
            field_to_complex(&self.field_z, data),
        ];

        for _ in 0..params.outer_iterations {
            z = solve_fields_parallel(data, &z, params);
            let [x, y, z_] = &mut z;
            apply_complex_angle_coupling(x, y, z_, params.coupling_weight);
        }

        finish_field(&z[0], &mut self.field_x, X_AXIS, data, params);
        finish_field(&z[1], &mut self.field_y, Y_AXIS, data, params);
        finish_field(&z[2], &mut self.field_z, Z_AXIS, data, params);
    }
}

fn solve_fields_parallel<T: Tag>(
    data: &SolverData<T>,
    z: &[Vec<Complex64>; 3],
    params: &FieldParams,
) -> [Vec<Complex64>; 3] {
    let mut solved: Vec<_> = (0..3)
        .par()
        .map(|i| {
            let mut work = CgWorkspace::new(data.normals.len());
            solve_one_field(data, &z[i], [X_AXIS, Y_AXIS, Z_AXIS][i], params, &mut work)
        })
        .collect();

    [solved.remove(0), solved.remove(0), solved.remove(0)]
}

fn finish_field<T: Tag>(
    z: &[Complex64],
    field: &mut Field<T>,
    axis: Vector3D,
    data: &SolverData<T>,
    params: &FieldParams,
) {
    complex_to_field(z, field, data);
    apply_axis_alignment_lengths(field, axis, data, params);
}

fn build_solver_data<T: Tag>(mesh: &Mesh<T>) -> SolverData<T> {
    let ids: Vec<_> = mesh.vert_ids();

    let mut index_of = HashMap::with_capacity(ids.len());
    for (i, id) in ids.iter().enumerate() {
        index_of.insert(*id, i);
    }

    let normals: Vec<_> = ids.par().map(|id| mesh.normal(*id).normalize()).collect();
    let bases: Vec<_> = normals.par().map(|normal| tangent_basis(*normal)).collect();

    let mut adjacency = vec![Vec::new(); ids.len()];

    for (i, id_i) in ids.iter().enumerate() {
        for id_j in mesh.neighbors(*id_i) {
            let Some(&j) = index_of.get(&id_j) else {
                continue;
            };

            if j <= i {
                continue;
            }

            let phase_j_to_i = connection_phase_j_to_i(&bases, &normals, i, j);
            let phase_i_to_j = connection_phase_j_to_i(&bases, &normals, j, i);
            adjacency[i].push(NeighborConnection { j, phase_j_to_i });
            adjacency[j].push(NeighborConnection {
                j: i,
                phase_j_to_i: phase_i_to_j,
            });
        }
    }

    let curvature: Vec<_> = ids
        .par()
        .map(|id| estimate_curvature_data(mesh, *id))
        .collect();

    SolverData {
        ids,
        normals,
        bases,
        adjacency,
        curvature,
    }
}

fn solve_one_field<T: Tag>(
    data: &SolverData<T>,
    current: &[Complex64],
    axis: Vector3D,
    params: &FieldParams,
    work: &mut CgWorkspace,
) -> Vec<Complex64> {
    let n = data.normals.len();
    let mut diag = vec![REGULARIZATION; n];
    let mut rhs = vec![C_ZERO; n];
    let damping_weight = params.damping_weight.max(0.0);
    for i in 0..n {
        if damping_weight > 0.0 {
            diag[i] += damping_weight;
            rhs[i] += current[i] * damping_weight;
        }

        if params.axis_weight > 0.0 {
            if let Some(target) = axis_target_complex(data, i, axis) {
                let w = params.axis_weight * target.confidence * target.confidence;
                diag[i] += w;
                rhs[i] += target.z * w;
            }
        }

        if params.curvature_weight > 0.0 {
            if let Some(target) = curvature_target_complex(data, i, current[i]) {
                let w = params.curvature_weight * target.confidence;
                diag[i] += w;
                rhs[i] += target.z * w;
            }
        }
    }

    let mut solved = conjugate_gradient(data, &diag, &rhs, current, params, work);

    for z in &mut solved {
        *z = normalize_complex_unit(*z);
    }

    solved
}

#[derive(Debug)]
struct CgWorkspace {
    r: Vec<Complex64>,
    p: Vec<Complex64>,
    ap: Vec<Complex64>,
    inv_diag: Vec<f64>,
}

impl CgWorkspace {
    fn new(n: usize) -> Self {
        Self {
            r: vec![C_ZERO; n],
            p: vec![C_ZERO; n],
            ap: vec![C_ZERO; n],
            inv_diag: vec![0.0; n],
        }
    }
}

fn conjugate_gradient<T: Tag>(
    data: &SolverData<T>,
    diag: &[f64],
    rhs: &[Complex64],
    initial: &[Complex64],
    params: &FieldParams,
    work: &mut CgWorkspace,
) -> Vec<Complex64> {
    let mut x = initial.to_vec();
    for i in 0..work.inv_diag.len() {
        let smooth_diag = data.adjacency[i].len() as f64 * params.smooth_weight;
        work.inv_diag[i] = 1.0 / (diag[i] + smooth_diag).max(EPS);
    }
    apply_system_operator(data, diag, &x, params.smooth_weight, &mut work.ap);

    let (mut rz_old, rhs_norm) =
        initialize_pcg(rhs, &work.ap, &work.inv_diag, &mut work.r, &mut work.p);

    let mut residual = complex_inner_real(&work.r, &work.r).sqrt() / rhs_norm;
    let mut previous_residual = residual;
    let mut stalled_iterations = 0;

    for iteration in 0..params.cg_iterations {
        if residual < params.cg_tolerance {
            break;
        }

        apply_system_operator(data, diag, &work.p, params.smooth_weight, &mut work.ap);
        let denom = complex_inner_real(&work.p, &work.ap);
        if denom.abs() < EPS {
            break;
        }

        residual = update_solution_and_residual(
            &mut x,
            &mut work.r,
            &work.p,
            &work.ap,
            rz_old / denom,
            rhs_norm,
        );

        if iteration + 1 >= CG_STALL_MIN_ITERATIONS {
            let improvement = (previous_residual - residual) / previous_residual.abs().max(EPS);
            if improvement < CG_STALL_RELATIVE_IMPROVEMENT {
                stalled_iterations += 1;
                if stalled_iterations >= CG_STALL_PATIENCE {
                    break;
                }
            } else {
                stalled_iterations = 0;
            }
        }
        previous_residual = residual;

        rz_old = update_preconditioned_direction(&work.r, &mut work.p, &work.inv_diag, rz_old);
    }

    x
}

fn initialize_pcg(
    rhs: &[Complex64],
    ax: &[Complex64],
    inv_diag: &[f64],
    r: &mut [Complex64],
    p: &mut [Complex64],
) -> (f64, f64) {
    let mut rz = 0.0;
    let mut rhs_norm_sqr = 0.0;

    for i in 0..rhs.len() {
        r[i] = rhs[i] - ax[i];
        p[i] = r[i] * inv_diag[i];

        let r_norm_sqr = r[i].norm_sqr();
        rz += r_norm_sqr * inv_diag[i];
        rhs_norm_sqr += rhs[i].norm_sqr();
    }

    (rz, rhs_norm_sqr.sqrt().max(EPS))
}

fn update_solution_and_residual(
    x: &mut [Complex64],
    r: &mut [Complex64],
    p: &[Complex64],
    ap: &[Complex64],
    alpha: f64,
    rhs_norm: f64,
) -> f64 {
    let mut norm = 0.0;
    for i in 0..x.len() {
        x[i] += p[i] * alpha;
        r[i] -= ap[i] * alpha;
        norm += r[i].norm_sqr();
    }
    norm.sqrt() / rhs_norm
}

fn update_preconditioned_direction(
    r: &[Complex64],
    p: &mut [Complex64],
    inv_diag: &[f64],
    rz_old: f64,
) -> f64 {
    let mut rz_new = 0.0;

    for i in 0..r.len() {
        rz_new += r[i].norm_sqr() * inv_diag[i];
    }

    let beta = rz_new / rz_old.max(EPS);
    for i in 0..p.len() {
        p[i] = r[i] * inv_diag[i] + p[i] * beta;
    }

    rz_new
}

fn apply_system_operator<T: Tag>(
    data: &SolverData<T>,
    diag: &[f64],
    x: &[Complex64],
    smooth_weight: f64,
    out: &mut [Complex64],
) {
    for i in 0..out.len() {
        out[i] = x[i] * diag[i];

        if smooth_weight > 0.0 {
            for nb in &data.adjacency[i] {
                out[i] += (x[i] - nb.phase_j_to_i * x[nb.j]) * smooth_weight;
            }
        }
    }
}

fn complex_inner_real(a: &[Complex64], b: &[Complex64]) -> f64 {
    (0..a.len()).map(|i| (a[i].conj() * b[i]).re).sum()
}

fn apply_complex_angle_coupling(
    z_x: &mut [Complex64],
    z_y: &mut [Complex64],
    z_z: &mut [Complex64],
    weight: f64,
) {
    if weight <= 0.0 {
        return;
    }

    z_x.iter_mut()
        .zip(z_y.iter_mut())
        .zip(z_z.iter_mut())
        .iter_into_par()
        .for_each(|((x, y), z)| {
            let x_old = *x;
            let y_old = *y;
            let z_old = *z;

            let update_x = 2.0 * weight * (sin2_delta(x_old, y_old) + sin2_delta(x_old, z_old));
            let update_y = 2.0 * weight * (sin2_delta(y_old, x_old) + sin2_delta(y_old, z_old));
            let update_z = 2.0 * weight * (sin2_delta(z_old, x_old) + sin2_delta(z_old, y_old));

            *x = x_old * complex_from_angle(update_x);
            *y = y_old * complex_from_angle(update_y);
            *z = z_old * complex_from_angle(update_z);
        });
}

fn sin2_delta(a: Complex64, b: Complex64) -> f64 {
    let d = b.conj() * a;
    2.0 * d.re * d.im
}

#[derive(Debug, Clone, Copy)]
struct ComplexTarget {
    z: Complex64,
    confidence: f64,
}

fn axis_target_complex<T: Tag>(
    data: &SolverData<T>,
    i: usize,
    axis: Vector3D,
) -> Option<ComplexTarget> {
    around_axis_target(data.normals[i], axis).map(|t| ComplexTarget {
        z: vector_to_complex(t.direction, data.bases[i]),
        confidence: t.confidence,
    })
}

fn curvature_target_complex<T: Tag>(
    data: &SolverData<T>,
    i: usize,
    current: Complex64,
) -> Option<ComplexTarget> {
    curvature_target(data.curvature[i], complex_to_vector(current, data.bases[i])).map(|t| {
        ComplexTarget {
            z: vector_to_complex(t.direction, data.bases[i]),
            confidence: t.confidence,
        }
    })
}

fn field_to_complex<T: Tag>(field: &Field<T>, data: &SolverData<T>) -> Vec<Complex64> {
    (0..data.ids.len())
        .par()
        .map(|i| {
            let key = field.map[&data.ids[i]];
            let v = project_to_tangent(field.vectors[key], data.normals[i]).normalize();
            vector_to_complex(v, data.bases[i])
        })
        .collect()
}

fn complex_to_field<T: Tag>(z: &[Complex64], field: &mut Field<T>, data: &SolverData<T>) {
    let values: Vec<_> = (0..data.ids.len())
        .par()
        .map(|i| complex_to_vector(z[i], data.bases[i]))
        .collect();

    for (i, id) in data.ids.iter().enumerate() {
        field.vectors[field.map[id]] = values[i];
    }
}

fn apply_axis_alignment_lengths<T: Tag>(
    field: &mut Field<T>,
    axis: Vector3D,
    data: &SolverData<T>,
    params: &FieldParams,
) {
    if params.axis_weight <= 0.0 {
        return;
    }

    let power = params.axis_length_power.max(EPS);
    let lengths: Vec<_> = (0..data.ids.len())
        .par()
        .map(|i| {
            let Some(target) = axis_target_complex(data, i, axis) else {
                return 0.1;
            };

            let z = vector_to_complex(field.vectors[field.map[&data.ids[i]]], data.bases[i]);
            let dot = (z.conj() * target.z).re.clamp(-1.0, 1.0);
            let angle = dot.acos();
            let tight_angle = std::f64::consts::PI / 12.0;
            let bad_angle = std::f64::consts::FRAC_PI_4;

            if angle <= tight_angle {
                let quality = 1.0 - angle / tight_angle;
                0.5 + 0.5 * quality.powf(power)
            } else {
                let quality =
                    1.0 - ((angle - tight_angle) / (bad_angle - tight_angle)).clamp(0.0, 1.0);
                0.1 + 0.4 * quality.powf(power)
            }
        })
        .collect();

    for (i, id) in data.ids.iter().enumerate() {
        field.vectors[field.map[id]] *= lengths[i];
    }
}

fn vector_to_complex(v: Vector3D, basis: TangentBasis) -> Complex64 {
    normalize_complex_unit(Complex64::new(v.dot(&basis.e1), v.dot(&basis.e2)))
}

fn complex_to_vector(z: Complex64, basis: TangentBasis) -> Vector3D {
    basis.e1 * z.re + basis.e2 * z.im
}

fn normalize_complex_unit(z: Complex64) -> Complex64 {
    let norm_sqr = z.norm_sqr();
    if norm_sqr > EPS {
        z / norm_sqr.sqrt()
    } else {
        C_ONE
    }
}

fn complex_from_angle(theta: f64) -> Complex64 {
    Complex64::new(theta.cos(), theta.sin())
}

fn connection_phase_j_to_i(
    bases: &[TangentBasis],
    normals: &[Vector3D],
    i: usize,
    j: usize,
) -> Complex64 {
    let n_i = normals[i];
    let b_i = bases[i];
    let b_j = bases[j];

    let v = project_to_tangent(b_j.e1, n_i).normalize();
    let phi = v.dot(&b_i.e2).atan2(v.dot(&b_i.e1));

    complex_from_angle(phi)
}

fn initialize_field_around_axis<T: Tag>(mesh: &Mesh<T>, field: &mut Field<T>, axis: Vector3D) {
    for (id, key) in &field.map {
        let n = mesh.normal(*id).normalize();

        let v = around_axis_target(n, axis)
            .map(|t| t.direction)
            .unwrap_or_else(|| tangent_fallback(n));

        field.vectors[*key] = v;
    }
}

#[derive(Debug, Clone, Copy)]
struct Target {
    direction: Vector3D,
    confidence: f64,
}

fn around_axis_target(normal: Vector3D, axis: Vector3D) -> Option<Target> {
    let tangent_axis = project_to_tangent(axis, normal);
    let confidence = tangent_axis.norm();
    (confidence >= EPS).then(|| Target {
        direction: normal.cross(&tangent_axis).normalize(),
        confidence,
    })
}

fn estimate_curvature_data<T: Tag>(mesh: &Mesh<T>, id: ids::Key<VERT, T>) -> Option<CurvatureData> {
    let p = mesh.position(id);
    let (t_raw, _, n_raw) = mesh.tangent_frame(id);
    let n = n_raw.normalize();
    let t1 = project_to_tangent(t_raw, n).normalize();
    let t2 = n.cross(&t1).normalize();

    let (mut a, mut b, mut d) = (0.0, 0.0, 0.0);
    let (mut bx0, mut bx1, mut by0, mut by1) = (0.0, 0.0, 0.0, 0.0);
    let (mut edge_sum, mut edge_count) = (0.0, 0);

    for nb in mesh.neighbors(id) {
        let edge = mesh.position(nb) - p;
        edge_sum += edge.norm();
        edge_count += 1;

        let e = project_to_tangent(edge, n);
        let len2 = e.dot(&e);
        if len2 < EPS {
            continue;
        }

        let dn = project_to_tangent(mesh.normal(nb).normalize() - n, n);
        let (ux, uy) = (t1.dot(&e), t2.dot(&e));
        let (nx, ny) = (-t1.dot(&dn), -t2.dot(&dn));
        let w = (1.0 / len2).min(1e6);

        a += w * ux * ux;
        b += w * ux * uy;
        d += w * uy * uy;
        bx0 += w * nx * ux;
        bx1 += w * nx * uy;
        by0 += w * ny * ux;
        by1 += w * ny * uy;
    }

    let det = a * d - b * b;
    if det.abs() < EPS || edge_count == 0 {
        return None;
    }

    let inv_det = 1.0 / det;
    let s00 = (d * bx0 - b * bx1) * inv_det;
    let s11 = (-b * by0 + a * by1) * inv_det;
    let s01 = 0.5 * ((-b * bx0 + a * bx1) + (d * by0 - b * by1)) * inv_det;
    let trace = 0.5 * (s00 + s11);
    let radius = ((0.5 * (s00 - s11)).powi(2) + s01 * s01).sqrt();
    let h = (edge_sum / edge_count as f64).max(EPS);
    let (c_min, c_max) = ((trace - radius).abs() * h, (trace + radius).abs() * h);

    if c_min.max(c_max) < CURVATURE_MIN_CONFIDENCE
        || (c_max - c_min).abs() < CURVATURE_MIN_CONFIDENCE
    {
        return None;
    }

    let dir_min = principal_direction(s00, s01, s11, trace - radius, t1, t2);
    Some(CurvatureData {
        dir_min,
        dir_max: n.cross(&dir_min).normalize(),
        c_min,
        c_max,
    })
}

fn curvature_target(curvature: Option<CurvatureData>, current: Vector3D) -> Option<Target> {
    let curvature = curvature?;

    let (best_direction, best_strength, best_dot) = [
        (curvature.dir_min, curvature.c_min),
        (-curvature.dir_min, curvature.c_min),
        (curvature.dir_max, curvature.c_max),
        (-curvature.dir_max, curvature.c_max),
    ]
    .into_iter()
    .map(|(direction, strength)| (direction, strength, current.dot(&direction)))
    .fold(
        (
            curvature.dir_min,
            curvature.c_min,
            current.dot(&curvature.dir_min),
        ),
        |best, candidate| {
            if candidate.2 > best.2 {
                candidate
            } else {
                best
            }
        },
    );

    if best_dot < CURVATURE_ALIGNMENT_MIN_DOT || best_strength < CURVATURE_MIN_CONFIDENCE {
        return None;
    }

    let curvature_confidence = (CURVATURE_CONFIDENCE_SENSITIVITY * best_strength).tanh();

    let alignment_t = ((best_dot - CURVATURE_ALIGNMENT_MIN_DOT)
        / (1.0 - CURVATURE_ALIGNMENT_MIN_DOT))
        .clamp(0.0, 1.0);

    let alignment_confidence = alignment_t.powf(CURVATURE_ALIGNMENT_SHARPNESS);
    let confidence = curvature_confidence * alignment_confidence;

    (confidence >= CURVATURE_MIN_CONFIDENCE).then_some(Target {
        direction: best_direction,
        confidence,
    })
}

fn principal_direction(
    s00: f64,
    s01: f64,
    s11: f64,
    k: f64,
    t1: Vector3D,
    t2: Vector3D,
) -> Vector3D {
    if s01.abs() > EPS {
        (t1 * s01 + t2 * (k - s00)).normalize()
    } else if s00 <= s11 {
        t1
    } else {
        t2
    }
}

fn project_to_tangent(v: Vector3D, n: Vector3D) -> Vector3D {
    v - n * v.dot(&n)
}

fn tangent_basis(normal: Vector3D) -> TangentBasis {
    let n = normal.normalize();
    let e1 = tangent_fallback(n);
    TangentBasis {
        e1,
        e2: n.cross(&e1).normalize(),
    }
}

fn tangent_fallback(n: Vector3D) -> Vector3D {
    let axis = if n.x.abs() < 0.9 { X_AXIS } else { Y_AXIS };
    project_to_tangent(axis, n).normalize()
}
