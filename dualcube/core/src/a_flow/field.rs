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

const STALL_QUALITY_IMPROVEMENT: f64 = 0.01;
const STALL_PATIENCE: usize = 5;

const X_AXIS: Vector3D = Vector3D::new(1.0, 0.0, 0.0);
const Y_AXIS: Vector3D = Vector3D::new(0.0, 1.0, 0.0);
const Z_AXIS: Vector3D = Vector3D::new(0.0, 0.0, 1.0);
const C_ZERO: Complex64 = Complex64::new(0.0, 0.0);
const C_ONE: Complex64 = Complex64::new(1.0, 0.0);

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct FieldParams {
    /// Nonlinear outer iterations.
    pub outer_iterations: usize,

    /// Maximum conjugate-gradient iterations for each global linear solve.
    pub cg_iterations: usize,

    /// Relative residual tolerance for CG.
    pub cg_tolerance: f64,

    /// Fixed numerical damping toward the previous outer iteration.
    ///
    /// Higher values make updates more conservative. Set to 0.0 to disable damping.
    pub damping_weight: f64,

    /// Global connection-Laplacian smoothness.
    ///
    /// Higher values produce fewer unnecessary singularities and smoother fields.
    pub smooth_weight: f64,

    /// Optional global around-axis target.
    ///
    /// For alignment with X/Y/Z axes. Set to 0.0 when you do not want to prescribe global axes.
    pub axis_weight: f64,

    /// Controls how quickly field-vector length fades with bad axis alignment.
    ///
    /// Controls how aggressively field-vector length fades with bad axis alignment.
    ///
    /// Only very well aligned vectors stay long: 0° maps to length 1.0, 15° maps
    /// to about 0.5, and angles of 45° or more map to length 0.1. Higher values
    /// push more vectors toward the 0.1 minimum.
    pub axis_length_power: f64,

    /// Optional principal-curvature-cross target.
    ///
    /// For feature awareness (through curvature)
    pub curvature_weight: f64,

    /// Local coupling between the three N=1 fields.
    ///
    /// This discourages the three tangent vectors from becoming parallel.
    pub coupling_weight: f64,
}

impl Default for FieldParams {
    fn default() -> Self {
        Self {
            // Should be > 0: with zero outer iterations the optimizer never runs and
            // `Fields::from_mesh` would return the random initial field.
            outer_iterations: 10,
            cg_iterations: 100,
            cg_tolerance: 1e-7,

            damping_weight: 0.03,

            smooth_weight: 1.0,
            axis_weight: 0.5,
            axis_length_power: 3.0,
            curvature_weight: 0.5,
            coupling_weight: 0.2,
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

    /// Unit phase mapping a complex direction at j into the tangent basis at i.
    phase_j_to_i: Complex64,

    weight: f64,
}

#[derive(Debug, Clone, Copy)]
struct UndirectedConnectionEdge {
    i: usize,
    j: usize,
    phase_j_to_i: Complex64,
    weight: f64,
}

#[derive(Debug, Clone)]
struct GlobalSolverCache {
    normals: Vec<Vector3D>,
    bases: Vec<TangentBasis>,

    /// Directed adjacency. Used for matrix-vector products without atomics.
    adjacency: Vec<Vec<NeighborConnection>>,

    /// Undirected edges. Used only for energy logging.
    edges: Vec<UndirectedConnectionEdge>,

    key_x: Vec<GlobalVectorKey>,
    key_y: Vec<GlobalVectorKey>,
    key_z: Vec<GlobalVectorKey>,

    axis_x: Vec<Option<ComplexTarget>>,
    axis_y: Vec<Option<ComplexTarget>>,
    axis_z: Vec<Option<ComplexTarget>>,

    curvature: Vec<Option<CurvatureData>>,
}

impl GlobalSolverCache {
    fn len(&self) -> usize {
        self.normals.len()
    }
}

impl<T: Tag> Default for Field<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Tag> Field<T> {
    pub fn new() -> Self {
        Self {
            map: HashMap::new(),
            vectors: SlotMap::with_key(),
        }
    }

    pub fn from_mesh(mesh: &Mesh<T>) -> Self {
        let mut field = Self::new();

        for id in mesh.vert_ids() {
            let key = field.vectors.insert(random_unit_vector());
            field.map.insert(id.to_owned(), key);
        }

        field
    }

    pub fn vector_at(&self, id: ids::Key<VERT, T>) -> Option<Vector3D> {
        self.map.get(&id).map(|key| self.vectors[*key])
    }
}

impl<T: Tag> Default for Fields<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Tag> Fields<T> {
    pub fn new() -> Self {
        Self {
            field_x: Field::new(),
            field_y: Field::new(),
            field_z: Field::new(),
        }
    }

    pub fn from_mesh(mesh: &Mesh<T>) -> Self {
        Self::from_mesh_with_params(mesh, FieldParams::default())
    }

    pub fn from_mesh_with_params(mesh: &Mesh<T>, params: FieldParams) -> Self {
        let mut fields = Self {
            field_x: Field::from_mesh(mesh),
            field_y: Field::from_mesh(mesh),
            field_z: Field::from_mesh(mesh),
        };

        if params.axis_weight > 0.0 {
            fields.initialize_around_axes(mesh);
        }

        let cache = build_global_solver_cache(mesh, &fields);
        fields.optimize_global_connection(&cache, &params);
        fields
    }

    pub fn optimize(&mut self, mesh: &Mesh<T>, params: &FieldParams) {
        let cache = build_global_solver_cache(mesh, self);
        self.optimize_global_connection(&cache, params);
    }

    fn initialize_around_axes(&mut self, mesh: &Mesh<T>) {
        initialize_field_around_axis(mesh, &mut self.field_x, X_AXIS);
        initialize_field_around_axis(mesh, &mut self.field_y, Y_AXIS);
        initialize_field_around_axis(mesh, &mut self.field_z, Z_AXIS);
    }

    fn optimize_global_connection(&mut self, cache: &GlobalSolverCache, params: &FieldParams) {
        let mut z_x = field_to_complex(&self.field_x.vectors, &cache.key_x, cache);
        let mut z_y = field_to_complex(&self.field_y.vectors, &cache.key_y, cache);
        let mut z_z = field_to_complex(&self.field_z.vectors, &cache.key_z, cache);

        let mut previous_energy = total_energy(cache, &z_x, &z_y, &z_z, params);

        log::info!(
            "global N=1 fields: iteration = 0/{}, energy = {:.6e}",
            params.outer_iterations,
            previous_energy
        );

        let mut stalled_iterations = 0;

        for iteration in 0..params.outer_iterations {
            let (mut next_x, mut next_y, mut next_z) =
                solve_fields_parallel(cache, &z_x, &z_y, &z_z, params);

            apply_complex_angle_coupling(
                &mut next_x,
                &mut next_y,
                &mut next_z,
                params.coupling_weight,
            );

            let energy = total_energy(cache, &next_x, &next_y, &next_z, params);
            let improvement = relative_energy_delta(previous_energy - energy, previous_energy);

            log::info!(
                "global N=1 fields: iteration = {}/{}, energy = {:.6e}, improvement = {:.2}%",
                iteration + 1,
                params.outer_iterations,
                energy,
                improvement * 100.0
            );

            if improvement >= 0.0 {
                z_x = next_x;
                z_y = next_y;
                z_z = next_z;
                previous_energy = energy;
            } else {
                log::info!(
                    "global N=1 fields: rejected iteration because quality decreased by {:.2}%",
                    -improvement * 100.0
                );
            }

            if improvement < STALL_QUALITY_IMPROVEMENT {
                stalled_iterations += 1;
            } else {
                stalled_iterations = 0;
            }

            if stalled_iterations >= STALL_PATIENCE {
                log::info!(
                    "global N=1 fields: stopping after {} iterations below 1% quality improvement",
                    STALL_PATIENCE
                );
                break;
            }
        }

        complex_to_field(&z_x, &mut self.field_x.vectors, &cache.key_x, cache);
        complex_to_field(&z_y, &mut self.field_y.vectors, &cache.key_y, cache);
        complex_to_field(&z_z, &mut self.field_z.vectors, &cache.key_z, cache);

        apply_axis_alignment_lengths(
            &mut self.field_x.vectors,
            &cache.key_x,
            &cache.axis_x,
            cache,
            params,
        );
        apply_axis_alignment_lengths(
            &mut self.field_y.vectors,
            &cache.key_y,
            &cache.axis_y,
            cache,
            params,
        );
        apply_axis_alignment_lengths(
            &mut self.field_z.vectors,
            &cache.key_z,
            &cache.axis_z,
            cache,
            params,
        );
    }
}

fn solve_fields_parallel(
    cache: &GlobalSolverCache,
    z_x: &[Complex64],
    z_y: &[Complex64],
    z_z: &[Complex64],
    params: &FieldParams,
) -> (Vec<Complex64>, Vec<Complex64>, Vec<Complex64>) {
    let mut solved: Vec<_> = (0..3)
        .par()
        .map(|field| {
            let (current, axis_targets) = match field {
                0 => (z_x, &cache.axis_x[..]),
                1 => (z_y, &cache.axis_y[..]),
                _ => (z_z, &cache.axis_z[..]),
            };

            let mut work = CgWorkspace::new(cache.len());
            solve_one_field(cache, current, axis_targets, params, &mut work)
        })
        .collect();

    (solved.remove(0), solved.remove(0), solved.remove(0))
}

fn build_global_solver_cache<T: Tag>(mesh: &Mesh<T>, fields: &Fields<T>) -> GlobalSolverCache {
    let ids: Vec<_> = mesh.vert_ids();

    let mut index_of = HashMap::new();
    for (i, id) in ids.iter().enumerate() {
        index_of.insert(*id, i);
    }

    let normals: Vec<_> = ids
        .par()
        .map(|id| unit_or(mesh.normal(*id), Z_AXIS))
        .collect();

    let bases: Vec<_> = normals
        .par()
        .map(|normal| {
            let (e1, e2) = tangent_basis(*normal);
            TangentBasis { e1, e2 }
        })
        .collect();

    let mut adjacency = vec![Vec::new(); ids.len()];
    let mut edges = Vec::new();

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
            let weight = 1.0;

            adjacency[i].push(NeighborConnection {
                j,
                phase_j_to_i,
                weight,
            });

            adjacency[j].push(NeighborConnection {
                j: i,
                phase_j_to_i: phase_i_to_j,
                weight,
            });

            edges.push(UndirectedConnectionEdge {
                i,
                j,
                phase_j_to_i,
                weight,
            });
        }
    }

    let key_x: Vec<_> = ids
        .par()
        .map(|id| *fields.field_x.map.get(id).expect("missing x-field key"))
        .collect();

    let key_y: Vec<_> = ids
        .par()
        .map(|id| *fields.field_y.map.get(id).expect("missing y-field key"))
        .collect();

    let key_z: Vec<_> = ids
        .par()
        .map(|id| *fields.field_z.map.get(id).expect("missing z-field key"))
        .collect();

    let axis_x = precompute_axis_targets(&normals, &bases, X_AXIS);
    let axis_y = precompute_axis_targets(&normals, &bases, Y_AXIS);
    let axis_z = precompute_axis_targets(&normals, &bases, Z_AXIS);

    let curvature: Vec<_> = ids
        .par()
        .map(|id| estimate_curvature_data(mesh, *id))
        .collect();

    GlobalSolverCache {
        normals,
        bases,
        adjacency,
        edges,
        key_x,
        key_y,
        key_z,
        axis_x,
        axis_y,
        axis_z,
        curvature,
    }
}

fn solve_one_field(
    cache: &GlobalSolverCache,
    current: &[Complex64],
    axis_targets: &[Option<ComplexTarget>],
    params: &FieldParams,
    work: &mut CgWorkspace,
) -> Vec<Complex64> {
    let n = cache.len();

    let terms: Vec<(f64, Complex64)> = (0..n)
        .par()
        .map(|i| {
            let mut diag = REGULARIZATION;
            let mut rhs = C_ZERO;

            let damping_weight = params.damping_weight.max(0.0);
            if damping_weight > 0.0 {
                diag += damping_weight;
                rhs += current[i] * damping_weight;
            }

            if params.axis_weight > 0.0 {
                if let Some(target) = axis_targets[i] {
                    let w = params.axis_weight * target.confidence * target.confidence;
                    diag += w;
                    rhs += target.z * w;
                }
            }

            if params.curvature_weight > 0.0 {
                if let Some(target) = curvature_target_complex(cache, i, current[i]) {
                    let w = params.curvature_weight * target.confidence;
                    diag += w;
                    rhs += target.z * w;
                }
            }

            (diag, rhs)
        })
        .collect();

    let (diag, rhs): (Vec<_>, Vec<_>) = terms.into_iter().unzip();

    let solved = conjugate_gradient(
        cache,
        &diag,
        &rhs,
        current,
        params.smooth_weight,
        params.cg_iterations,
        params.cg_tolerance,
        work,
    );

    solved.into_iter().map(normalize_complex_unit).collect()
}

#[derive(Debug)]
struct CgWorkspace {
    ax: Vec<Complex64>,
    r: Vec<Complex64>,
    p: Vec<Complex64>,
    ap: Vec<Complex64>,
    operator_values: Vec<Complex64>,
}

impl CgWorkspace {
    fn new(n: usize) -> Self {
        Self {
            ax: vec![C_ZERO; n],
            r: vec![C_ZERO; n],
            p: vec![C_ZERO; n],
            ap: vec![C_ZERO; n],
            operator_values: vec![C_ZERO; n],
        }
    }
}

fn conjugate_gradient(
    cache: &GlobalSolverCache,
    diag: &[f64],
    rhs: &[Complex64],
    initial: &[Complex64],
    smooth_weight: f64,
    max_iterations: usize,
    tolerance: f64,
    work: &mut CgWorkspace,
) -> Vec<Complex64> {
    let n = rhs.len();
    let mut x = initial.to_vec();

    apply_system_operator(
        cache,
        diag,
        &x,
        smooth_weight,
        &mut work.ax,
        &mut work.operator_values,
    );

    for i in 0..n {
        work.r[i] = rhs[i] - work.ax[i];
        work.p[i] = work.r[i];
    }

    let mut rs_old = complex_inner_real(&work.r, &work.r);
    let rhs_norm = complex_inner_real(rhs, rhs).sqrt().max(EPS);

    if rs_old.sqrt() / rhs_norm < tolerance {
        return x;
    }

    for _ in 0..max_iterations {
        apply_system_operator(
            cache,
            diag,
            &work.p,
            smooth_weight,
            &mut work.ap,
            &mut work.operator_values,
        );
        let denom = complex_inner_real(&work.p, &work.ap);

        if denom.abs() < EPS {
            break;
        }

        let alpha = rs_old / denom;

        for i in 0..n {
            x[i] += work.p[i] * alpha;
            work.r[i] -= work.ap[i] * alpha;
        }

        let rs_new = complex_inner_real(&work.r, &work.r);

        if rs_new.sqrt() / rhs_norm < tolerance {
            break;
        }

        let beta = rs_new / rs_old;
        for i in 0..n {
            work.p[i] = work.r[i] + work.p[i] * beta;
        }

        rs_old = rs_new;
    }

    x
}

fn apply_system_operator(
    cache: &GlobalSolverCache,
    diag: &[f64],
    x: &[Complex64],
    smooth_weight: f64,
    out: &mut [Complex64],
    values: &mut [Complex64],
) {
    let computed: Vec<_> = (0..cache.len())
        .par()
        .map(|i| {
            let mut y = x[i] * diag[i];

            if smooth_weight > 0.0 {
                for nb in &cache.adjacency[i] {
                    y += (x[i] - nb.phase_j_to_i * x[nb.j]) * (smooth_weight * nb.weight);
                }
            }

            y
        })
        .collect();

    values.copy_from_slice(&computed);
    out.copy_from_slice(values);
}

fn complex_inner_real(a: &[Complex64], b: &[Complex64]) -> f64 {
    (0..a.len()).par().map(|i| (a[i].conj() * b[i]).re).sum()
}

fn relative_energy_delta(delta: f64, previous_energy: f64) -> f64 {
    if previous_energy.abs() > EPS {
        delta / previous_energy.abs()
    } else {
        0.0
    }
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

    let updated: Vec<_> = (0..z_x.len())
        .par()
        .map(|i| {
            let mut tx = z_x[i].arg();
            let mut ty = z_y[i].arg();
            let mut tz = z_z[i].arg();

            let gx = -sin2(tx - ty) - sin2(tx - tz);
            let gy = sin2(tx - ty) - sin2(ty - tz);
            let gz = sin2(tx - tz) + sin2(ty - tz);

            tx -= weight * gx;
            ty -= weight * gy;
            tz -= weight * gz;

            (
                complex_from_angle(tx),
                complex_from_angle(ty),
                complex_from_angle(tz),
            )
        })
        .collect();

    for (i, (x, y, z)) in updated.into_iter().enumerate() {
        z_x[i] = x;
        z_y[i] = y;
        z_z[i] = z;
    }
}

fn total_energy(
    cache: &GlobalSolverCache,
    z_x: &[Complex64],
    z_y: &[Complex64],
    z_z: &[Complex64],
    params: &FieldParams,
) -> f64 {
    let smooth = connection_laplacian_energy(cache, z_x)
        + connection_laplacian_energy(cache, z_y)
        + connection_laplacian_energy(cache, z_z);

    let axis = axis_energy(z_x, &cache.axis_x)
        + axis_energy(z_y, &cache.axis_y)
        + axis_energy(z_z, &cache.axis_z);

    let curvature =
        curvature_energy(cache, z_x) + curvature_energy(cache, z_y) + curvature_energy(cache, z_z);

    let coupling = complex_coupling_energy(z_x, z_y, z_z);

    params.smooth_weight * smooth
        + params.axis_weight * axis
        + params.curvature_weight * curvature
        + params.coupling_weight * coupling
}

fn connection_laplacian_energy(cache: &GlobalSolverCache, z: &[Complex64]) -> f64 {
    (0..cache.edges.len())
        .par()
        .map(|i| {
            let edge = cache.edges[i];
            let diff = z[edge.i] - edge.phase_j_to_i * z[edge.j];
            edge.weight * diff.norm_sqr()
        })
        .sum()
}

fn axis_energy(z: &[Complex64], targets: &[Option<ComplexTarget>]) -> f64 {
    (0..z.len())
        .par()
        .map(|i| {
            let Some(target) = targets[i] else {
                return 0.0;
            };

            let diff = z[i] - target.z;
            target.confidence * target.confidence * diff.norm_sqr()
        })
        .sum()
}

fn curvature_energy(cache: &GlobalSolverCache, z: &[Complex64]) -> f64 {
    (0..cache.len())
        .par()
        .map(|i| {
            let Some(target) = curvature_target_complex(cache, i, z[i]) else {
                return 0.0;
            };

            let diff = z[i] - target.z;
            target.confidence * diff.norm_sqr()
        })
        .sum()
}

fn complex_coupling_energy(z_x: &[Complex64], z_y: &[Complex64], z_z: &[Complex64]) -> f64 {
    (0..z_x.len())
        .par()
        .map(|i| {
            let tx = z_x[i].arg();
            let ty = z_y[i].arg();
            let tz = z_z[i].arg();

            (tx - ty).cos().powi(2) + (tx - tz).cos().powi(2) + (ty - tz).cos().powi(2)
        })
        .sum()
}

#[derive(Debug, Clone, Copy)]
struct ComplexTarget {
    z: Complex64,
    confidence: f64,
}

fn precompute_axis_targets(
    normals: &[Vector3D],
    bases: &[TangentBasis],
    axis: Vector3D,
) -> Vec<Option<ComplexTarget>> {
    (0..normals.len())
        .par()
        .map(|i| {
            let target = around_axis_target(normals[i], axis)?;
            Some(ComplexTarget {
                z: vector_to_complex(target.direction, bases[i]),
                confidence: target.confidence,
            })
        })
        .collect()
}

fn curvature_target_complex(
    cache: &GlobalSolverCache,
    i: usize,
    current: Complex64,
) -> Option<ComplexTarget> {
    let current_v = complex_to_vector(current, cache.bases[i]);
    let target = curvature_target_from_cache(cache.curvature[i], current_v)?;
    Some(ComplexTarget {
        z: vector_to_complex(target.direction, cache.bases[i]),
        confidence: target.confidence,
    })
}

fn field_to_complex(
    vectors: &SlotMap<GlobalVectorKey, Vector3D>,
    keys: &[GlobalVectorKey],
    cache: &GlobalSolverCache,
) -> Vec<Complex64> {
    (0..keys.len())
        .par()
        .map(|i| {
            let n = cache.normals[i];
            let v = unit_or(project_to_tangent(vectors[keys[i]], n), cache.bases[i].e1);
            vector_to_complex(v, cache.bases[i])
        })
        .collect()
}

fn complex_to_field(
    z: &[Complex64],
    vectors: &mut SlotMap<GlobalVectorKey, Vector3D>,
    keys: &[GlobalVectorKey],
    cache: &GlobalSolverCache,
) {
    let values: Vec<_> = (0..keys.len())
        .par()
        .map(|i| complex_to_vector(z[i], cache.bases[i]))
        .collect();

    for (i, key) in keys.iter().enumerate() {
        vectors[*key] = values[i];
    }
}

fn apply_axis_alignment_lengths(
    vectors: &mut SlotMap<GlobalVectorKey, Vector3D>,
    keys: &[GlobalVectorKey],
    axis_targets: &[Option<ComplexTarget>],
    cache: &GlobalSolverCache,
    params: &FieldParams,
) {
    if params.axis_weight <= 0.0 {
        return;
    }

    let power = params.axis_length_power.max(EPS);
    let lengths: Vec<_> = (0..keys.len())
        .par()
        .map(|i| {
            let Some(target) = axis_targets[i] else {
                return 0.1;
            };

            let z = vector_to_complex(vectors[keys[i]], cache.bases[i]);
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

    for (i, key) in keys.iter().enumerate() {
        vectors[*key] *= lengths[i];
    }
}

fn vector_to_complex(v: Vector3D, basis: TangentBasis) -> Complex64 {
    normalize_complex_unit(Complex64::new(v.dot(&basis.e1), v.dot(&basis.e2)))
}

fn complex_to_vector(z: Complex64, basis: TangentBasis) -> Vector3D {
    basis.e1 * z.re + basis.e2 * z.im
}

fn normalize_complex_unit(z: Complex64) -> Complex64 {
    if z.norm_sqr() > EPS {
        z / z.norm()
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
        let n = unit_or(mesh.normal(*id), Z_AXIS);

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
        direction: tangent_axis.cross(&normal).normalize(),
        confidence,
    })
}

fn estimate_curvature_data<T: Tag>(mesh: &Mesh<T>, id: ids::Key<VERT, T>) -> Option<CurvatureData> {
    let p = mesh.position(id);
    let (t_raw, _, n_raw) = mesh.tangent_frame(id);
    let n = unit_or(n_raw, Z_AXIS);
    let t1 = unit_or(project_to_tangent(t_raw, n), tangent_fallback(n));
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

        let dn = project_to_tangent(unit_or(mesh.normal(nb), n) - n, n);
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

fn curvature_target_from_cache(data: Option<CurvatureData>, current: Vector3D) -> Option<Target> {
    let data = data?;

    let candidates = [
        (data.dir_min, data.c_min),
        (-data.dir_min, data.c_min),
        (data.dir_max, data.c_max),
        (-data.dir_max, data.c_max),
    ];

    let mut best_direction = candidates[0].0;
    let mut best_strength = candidates[0].1;
    let mut best_dot = current.dot(&best_direction);

    for (direction, strength) in candidates.into_iter().skip(1) {
        let dot = current.dot(&direction);

        if dot > best_dot {
            best_direction = direction;
            best_strength = strength;
            best_dot = dot;
        }
    }

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

fn tangent_basis(normal: Vector3D) -> (Vector3D, Vector3D) {
    let n = unit_or(normal, Z_AXIS);
    let e1 = tangent_fallback(n);
    let e2 = n.cross(&e1).normalize();
    (e1, e2)
}

fn tangent_fallback(n: Vector3D) -> Vector3D {
    let axis = if n.x.abs() < 0.9 { X_AXIS } else { Y_AXIS };

    project_to_tangent(axis, n).normalize()
}

fn unit_or(v: Vector3D, fallback: Vector3D) -> Vector3D {
    if v.norm() > EPS {
        v.normalize()
    } else {
        fallback
    }
}

fn random_unit_vector() -> Vector3D {
    let v = Vector3D::new(
        rand::random::<f64>() - 0.5,
        rand::random::<f64>() - 0.5,
        rand::random::<f64>() - 0.5,
    );

    unit_or(v, X_AXIS)
}

fn sin2(theta: f64) -> f64 {
    (2.0 * theta).sin()
}
