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

const CG_STALL_MIN_ITERATIONS: usize = 8;
const CG_STALL_PATIENCE: usize = 4;
const CG_STALL_RELATIVE_IMPROVEMENT: f64 = 1e-3;

const X_AXIS: Vector3D = Vector3D::new(1.0, 0.0, 0.0);
const Y_AXIS: Vector3D = Vector3D::new(0.0, 1.0, 0.0);
const Z_AXIS: Vector3D = Vector3D::new(0.0, 0.0, 1.0);
const C_ZERO: Complex64 = Complex64::new(0.0, 0.0);
const AXES: [Vector3D; 3] = [X_AXIS, Y_AXIS, Z_AXIS];

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct FieldParams {
    #[serde(alias = "iterations")]
    pub outer_iterations: usize,
    pub cg_iterations: usize,
    pub cg_tolerance: f64,
    pub smooth_weight: f64,
    pub axis_weight: f64,
    pub coupling_weight: f64,
}

impl Default for FieldParams {
    fn default() -> Self {
        Self {
            outer_iterations: 100,
            cg_iterations: 500,
            cg_tolerance: 1e-4,
            smooth_weight: 1.0,
            axis_weight: 0.5,
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
struct NeighborConnection {
    j: usize,
    map_j_to_i: TangentMap,
}

#[derive(Debug, Clone, Copy)]
struct TangentMap {
    m11: f64,
    m12: f64,
    m21: f64,
    m22: f64,
}

impl TangentMap {
    fn apply(self, z: Complex64) -> Complex64 {
        Complex64::new(
            self.m11 * z.re + self.m12 * z.im,
            self.m21 * z.re + self.m22 * z.im,
        )
    }
}

#[derive(Debug, Clone)]
struct SolverData<T: Tag> {
    ids: Vec<ids::Key<VERT, T>>,
    normals: Vec<Vector3D>,
    bases: Vec<TangentBasis>,
    adjacency: Vec<Vec<NeighborConnection>>,
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
        for (field, axis) in [
            (&mut self.field_x, X_AXIS),
            (&mut self.field_y, Y_AXIS),
            (&mut self.field_z, Z_AXIS),
        ] {
            initialize_field_around_axis(mesh, field, axis);
        }
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

        for (field, z) in [&mut self.field_x, &mut self.field_y, &mut self.field_z]
            .into_iter()
            .zip(&z)
        {
            complex_to_field(z, field, data);
        }
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
            solve_one_field(data, &z[i], AXES[i], params, &mut work)
        })
        .collect();

    [solved.remove(0), solved.remove(0), solved.remove(0)]
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

            let map_j_to_i = extrinsic_tangent_map_j_to_i(&bases, i, j);
            let map_i_to_j = extrinsic_tangent_map_j_to_i(&bases, j, i);
            adjacency[i].push(NeighborConnection { j, map_j_to_i });
            adjacency[j].push(NeighborConnection {
                j: i,
                map_j_to_i: map_i_to_j,
            });
        }
    }

    SolverData {
        ids,
        normals,
        bases,
        adjacency,
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
    let mut diag = vec![0.0; n];
    let mut rhs = vec![C_ZERO; n];
    if params.axis_weight > 0.0 {
        for i in 0..n {
            if let Some((z, confidence)) = axis_target_complex(data, i, axis) {
                let w = params.axis_weight * confidence * confidence;
                diag[i] += w;
                rhs[i] += z * w;
            }
        }
    }

    conjugate_gradient(data, &diag, &rhs, current, params, work)
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
    if smooth_weight <= 0.0 {
        for i in 0..out.len() {
            out[i] = x[i] * diag[i];
        }
        return;
    }

    for i in 0..out.len() {
        out[i] = x[i] * diag[i];
        for nb in &data.adjacency[i] {
            out[i] += (x[i] - nb.map_j_to_i.apply(x[nb.j])) * smooth_weight;
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

fn axis_target_complex<T: Tag>(
    data: &SolverData<T>,
    i: usize,
    axis: Vector3D,
) -> Option<(Complex64, f64)> {
    around_axis_target(data.normals[i], axis)
        .map(|(direction, confidence)| (vector_to_complex(direction, data.bases[i]), confidence))
}

fn field_to_complex<T: Tag>(field: &Field<T>, data: &SolverData<T>) -> Vec<Complex64> {
    (0..data.ids.len())
        .par()
        .map(|i| {
            let key = field.map[&data.ids[i]];
            vector_to_complex(
                project_to_tangent(field.vectors[key], data.normals[i]),
                data.bases[i],
            )
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

fn vector_to_complex(v: Vector3D, basis: TangentBasis) -> Complex64 {
    Complex64::new(v.dot(&basis.e1), v.dot(&basis.e2))
}

fn complex_to_vector(z: Complex64, basis: TangentBasis) -> Vector3D {
    basis.e1 * z.re + basis.e2 * z.im
}

fn complex_from_angle(theta: f64) -> Complex64 {
    Complex64::new(theta.cos(), theta.sin())
}

fn extrinsic_tangent_map_j_to_i(bases: &[TangentBasis], i: usize, j: usize) -> TangentMap {
    let b_i = bases[i];
    let b_j = bases[j];

    TangentMap {
        m11: b_i.e1.dot(&b_j.e1),
        m12: b_i.e1.dot(&b_j.e2),
        m21: b_i.e2.dot(&b_j.e1),
        m22: b_i.e2.dot(&b_j.e2),
    }
}

fn initialize_field_around_axis<T: Tag>(mesh: &Mesh<T>, field: &mut Field<T>, axis: Vector3D) {
    for (id, key) in &field.map {
        let n = mesh.normal(*id).normalize();

        let v = around_axis_target(n, axis)
            .map(|(direction, _)| direction)
            .unwrap_or_else(|| tangent_fallback(n));

        field.vectors[*key] = v;
    }
}

fn around_axis_target(normal: Vector3D, axis: Vector3D) -> Option<(Vector3D, f64)> {
    let tangent_axis = project_to_tangent(axis, normal);
    let confidence = tangent_axis.norm();
    (confidence >= EPS).then(|| (normal.cross(&tangent_axis).normalize(), confidence))
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
