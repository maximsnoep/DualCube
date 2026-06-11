use mehsh::prelude::*;
use orx_parallel::*;
use serde::{Deserialize, Serialize};
use slotmap::{new_key_type, SecondaryMap, SlotMap};
use std::collections::{HashMap, HashSet};

new_key_type! {
    pub struct VectorKey;
}

const EPS: f64 = 1e-12;

const CURVATURE_MIN_CONFIDENCE: f64 = 1e-3;
const CURVATURE_CONFIDENCE_SENSITIVITY: f64 = 2.0;

const CURVATURE_ALIGNMENT_MIN_DOT: f64 = 0.75;
const CURVATURE_ALIGNMENT_SHARPNESS: f64 = 2.0;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldsParams {
    pub iterations: usize,
    pub axis_weight: f64,
    pub curvature_weight: f64,
    pub smooth_weight: f64,

    /// Coherence of the induced "around" directions.
    ///
    /// For a tangent field u at normal n, define r = n × u.
    /// This term smooths r across the 1-ring and then converts the averaged
    /// around-direction back to a tangent vector.
    pub coherence_weight: f64,

    pub coupling_weight: f64,
    pub inertia_weight: f64,
}

impl Default for FieldsParams {
    fn default() -> Self {
        Self {
            iterations: 30,
            axis_weight: 4.0,
            curvature_weight: 0.5,
            smooth_weight: 2.0,
            coherence_weight: 0.0,
            coupling_weight: 0.02,
            inertia_weight: 1.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Field<T: Tag> {
    pub map: HashMap<ids::Key<VERT, T>, VectorKey>,
    pub vectors: SlotMap<VectorKey, Vector3D>,
    pub connectivity: SecondaryMap<VectorKey, HashSet<VectorKey>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Fields<T: Tag> {
    pub field_x: Field<T>,
    pub field_y: Field<T>,
    pub field_z: Field<T>,
}

#[derive(Debug, Clone, Copy)]
struct Target {
    direction: Vector3D,
    confidence: f64,
}

#[derive(Debug, Clone, Copy)]
struct CurvatureData {
    dir_min: Vector3D,
    dir_max: Vector3D,
    c_min: f64,
    c_max: f64,
}

#[derive(Debug, Clone)]
struct SolverCache {
    normals: Vec<Vector3D>,
    neighbors: Vec<Vec<usize>>,

    key_x: Vec<VectorKey>,
    key_y: Vec<VectorKey>,
    key_z: Vec<VectorKey>,

    curvature: Vec<Option<CurvatureData>>,
}

impl SolverCache {
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
            connectivity: SecondaryMap::new(),
        }
    }

    pub fn from_mesh(mesh: &Mesh<T>) -> Self {
        let mut field = Field::new();

        for id in mesh.vert_ids() {
            let key = field.vectors.insert(random_unit_vector());
            field.map.insert(id.to_owned(), key);
        }

        for id in mesh.vert_ids() {
            let key = *field.map.get(&id).expect("missing field vector for vertex");

            let neighbors = mesh
                .neighbors(id)
                .filter_map(|nb| field.map.get(&nb).copied())
                .collect();

            field.connectivity.insert(key, neighbors);
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
        Self::from_mesh_with_params(mesh, FieldsParams::default())
    }

    pub fn from_mesh_with_params(mesh: &Mesh<T>, params: FieldsParams) -> Self {
        let mut fields = Self {
            field_x: Field::from_mesh(mesh),
            field_y: Field::from_mesh(mesh),
            field_z: Field::from_mesh(mesh),
        };

        if params.axis_weight > 0.0 {
            fields.initialize_around_axes(mesh);
        }

        let cache = build_solver_cache(mesh, &fields);
        fields.optimize_cached(&cache, &params);

        fields
    }

    fn initialize_around_axes(&mut self, mesh: &Mesh<T>) {
        initialize_field_around_axis(mesh, &mut self.field_x, Vector3D::new(1.0, 0.0, 0.0));
        initialize_field_around_axis(mesh, &mut self.field_y, Vector3D::new(0.0, 1.0, 0.0));
        initialize_field_around_axis(mesh, &mut self.field_z, Vector3D::new(0.0, 0.0, 1.0));
    }

    pub fn optimize(&mut self, mesh: &Mesh<T>, params: &FieldsParams) {
        let cache = build_solver_cache(mesh, self);
        self.optimize_cached(&cache, params);
    }

    fn optimize_cached(&mut self, cache: &SolverCache, params: &FieldsParams) {
        let mut previous_energy = self.total_energy(cache, params);

        log::info!(
            "field optimization start: iteration = 0, energy = {:.6e}",
            previous_energy
        );

        for iteration in 0..params.iterations {
            update_field_parallel(
                cache,
                &mut self.field_x.vectors,
                &cache.key_x,
                Vector3D::new(1.0, 0.0, 0.0),
                params,
            );

            update_field_parallel(
                cache,
                &mut self.field_y.vectors,
                &cache.key_y,
                Vector3D::new(0.0, 1.0, 0.0),
                params,
            );

            update_field_parallel(
                cache,
                &mut self.field_z.vectors,
                &cache.key_z,
                Vector3D::new(0.0, 0.0, 1.0),
                params,
            );

            apply_angle_coupling_parallel(cache, self, params.coupling_weight);

            let energy = self.total_energy(cache, params);
            let delta = previous_energy - energy;
            let relative_delta = if previous_energy.abs() > EPS {
                delta / previous_energy.abs()
            } else {
                0.0
            };

            log::info!(
                "field optimization: iteration = {}/{}, energy = {:.6e}, delta = {:.6e}, relative_delta = {:.6e}",
                iteration + 1,
                params.iterations,
                energy,
                delta,
                relative_delta
            );

            previous_energy = energy;
        }
    }

    fn total_energy(&self, cache: &SolverCache, params: &FieldsParams) -> f64 {
        let axis_energy = field_axis_energy(
            cache,
            &self.field_x.vectors,
            &cache.key_x,
            Vector3D::new(1.0, 0.0, 0.0),
        ) + field_axis_energy(
            cache,
            &self.field_y.vectors,
            &cache.key_y,
            Vector3D::new(0.0, 1.0, 0.0),
        ) + field_axis_energy(
            cache,
            &self.field_z.vectors,
            &cache.key_z,
            Vector3D::new(0.0, 0.0, 1.0),
        );

        let curvature_energy = field_curvature_energy(cache, &self.field_x.vectors, &cache.key_x)
            + field_curvature_energy(cache, &self.field_y.vectors, &cache.key_y)
            + field_curvature_energy(cache, &self.field_z.vectors, &cache.key_z);

        let smooth_energy = field_smooth_energy(cache, &self.field_x.vectors, &cache.key_x)
            + field_smooth_energy(cache, &self.field_y.vectors, &cache.key_y)
            + field_smooth_energy(cache, &self.field_z.vectors, &cache.key_z);

        let coherence_energy = field_coherence_energy(cache, &self.field_x.vectors, &cache.key_x)
            + field_coherence_energy(cache, &self.field_y.vectors, &cache.key_y)
            + field_coherence_energy(cache, &self.field_z.vectors, &cache.key_z);

        let coupling_energy = coupling_energy(cache, self);

        params.axis_weight * axis_energy
            + params.curvature_weight * curvature_energy
            + params.smooth_weight * smooth_energy
            + params.coherence_weight * coherence_energy
            + params.coupling_weight * coupling_energy
    }
}

fn build_solver_cache<T: Tag>(mesh: &Mesh<T>, fields: &Fields<T>) -> SolverCache {
    let ids: Vec<_> = mesh.vert_ids();

    let mut index_of = HashMap::new();
    for (i, id) in ids.iter().enumerate() {
        index_of.insert(*id, i);
    }

    let normals: Vec<_> = ids
        .iter()
        .map(|id| safe_normalize(mesh.normal(*id), Vector3D::new(0.0, 0.0, 1.0)))
        .collect();

    let neighbors: Vec<Vec<usize>> = ids
        .iter()
        .map(|id| {
            mesh.neighbors(*id)
                .filter_map(|nb| index_of.get(&nb).copied())
                .collect()
        })
        .collect();

    let key_x: Vec<_> = ids
        .iter()
        .map(|id| *fields.field_x.map.get(id).expect("missing x-field key"))
        .collect();

    let key_y: Vec<_> = ids
        .iter()
        .map(|id| *fields.field_y.map.get(id).expect("missing y-field key"))
        .collect();

    let key_z: Vec<_> = ids
        .iter()
        .map(|id| *fields.field_z.map.get(id).expect("missing z-field key"))
        .collect();

    let curvature: Vec<_> = ids
        .iter()
        .zip(normals.iter())
        .map(|(id, normal)| precompute_curvature_data(mesh, *id, *normal))
        .collect();

    SolverCache {
        normals,
        neighbors,
        key_x,
        key_y,
        key_z,
        curvature,
    }
}

fn initialize_field_around_axis<T: Tag>(mesh: &Mesh<T>, field: &mut Field<T>, axis: Vector3D) {
    for (id, key) in &field.map {
        let n = safe_normalize(mesh.normal(*id), Vector3D::new(0.0, 0.0, 1.0));

        let v = around_axis_target(n, axis)
            .map(|t| t.direction)
            .unwrap_or_else(|| tangent_fallback(n));

        field.vectors[*key] = v;
    }
}

fn update_field_parallel(
    cache: &SolverCache,
    vectors: &mut SlotMap<VectorKey, Vector3D>,
    keys: &[VectorKey],
    axis: Vector3D,
    params: &FieldsParams,
) {
    let updates: Vec<(VectorKey, Vector3D)> = {
        let old = &*vectors;

        (0..cache.len())
            .par()
            .map(|i| {
                let key = keys[i];
                let n_i = cache.normals[i];

                let current =
                    safe_normalize(project_to_tangent(old[key], n_i), tangent_fallback(n_i));

                let mut acc = current * params.inertia_weight;

                if params.axis_weight > 0.0 {
                    if let Some(target) = around_axis_target(n_i, axis) {
                        acc += target.direction
                            * params.axis_weight
                            * target.confidence
                            * target.confidence;
                    }
                }

                if params.curvature_weight > 0.0 {
                    if let Some(target) = curvature_target_from_cache(cache.curvature[i], current) {
                        acc += target.direction * params.curvature_weight * target.confidence;
                    }
                }

                if params.smooth_weight > 0.0 {
                    if let Some(target) = neighbor_average(cache, old, keys, i) {
                        acc += target.direction * params.smooth_weight;
                    }
                }

                if params.coherence_weight > 0.0 {
                    if let Some(target) = around_vector_coherence_target(cache, old, keys, i) {
                        acc += target.direction * params.coherence_weight;
                    }
                }

                let new_v = safe_normalize(project_to_tangent(acc, n_i), current);

                (key, new_v)
            })
            .collect()
    };

    for (key, v) in updates {
        vectors[key] = v;
    }
}

fn around_axis_target(normal: Vector3D, axis: Vector3D) -> Option<Target> {
    let n = safe_normalize(normal, Vector3D::new(0.0, 0.0, 1.0));
    let a = safe_normalize(axis, Vector3D::new(1.0, 0.0, 0.0));

    let tangent_axis = project_to_tangent(a, n);
    let confidence = tangent_axis.norm();

    if confidence < EPS {
        return None;
    }

    let direction = tangent_axis.cross(&n);

    if direction.norm() < EPS {
        return None;
    }

    Some(Target {
        direction: direction.normalize(),
        confidence,
    })
}

fn precompute_curvature_data<T: Tag>(
    mesh: &Mesh<T>,
    id: ids::Key<VERT, T>,
    normal: Vector3D,
) -> Option<CurvatureData> {
    let n = safe_normalize(normal, Vector3D::new(0.0, 0.0, 1.0));

    let (k_min, k_max, dir_min, dir_max) =
        crate::elastica::estimate_vertex_principal_frame(mesh, id);

    let h = local_edge_scale(mesh, id)?;

    let c_min = k_min.abs() * h;
    let c_max = k_max.abs() * h;

    let strength = c_min.max(c_max);
    if strength < CURVATURE_MIN_CONFIDENCE {
        return None;
    }

    let anisotropy = (c_max - c_min).abs();
    if anisotropy < CURVATURE_MIN_CONFIDENCE {
        return None;
    }

    let d_min = project_to_tangent(dir_min, n);
    let d_max = project_to_tangent(dir_max, n);

    if d_min.norm() < EPS || d_max.norm() < EPS {
        return None;
    }

    Some(CurvatureData {
        dir_min: d_min.normalize(),
        dir_max: d_max.normalize(),
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

    if best_dot < CURVATURE_ALIGNMENT_MIN_DOT {
        return None;
    }

    if best_strength < CURVATURE_MIN_CONFIDENCE {
        return None;
    }

    let curvature_confidence = (CURVATURE_CONFIDENCE_SENSITIVITY * best_strength).tanh();

    if curvature_confidence < CURVATURE_MIN_CONFIDENCE {
        return None;
    }

    let alignment_t = ((best_dot - CURVATURE_ALIGNMENT_MIN_DOT)
        / (1.0 - CURVATURE_ALIGNMENT_MIN_DOT))
        .clamp(0.0, 1.0);

    let alignment_confidence = alignment_t.powf(CURVATURE_ALIGNMENT_SHARPNESS);
    let confidence = curvature_confidence * alignment_confidence;

    if confidence < CURVATURE_MIN_CONFIDENCE {
        return None;
    }

    Some(Target {
        direction: best_direction,
        confidence,
    })
}

fn neighbor_average(
    cache: &SolverCache,
    old: &SlotMap<VectorKey, Vector3D>,
    keys: &[VectorKey],
    i: usize,
) -> Option<Target> {
    let n_i = cache.normals[i];

    let mut acc = Vector3D::new(0.0, 0.0, 0.0);
    let mut count = 0.0;

    for &j in &cache.neighbors[i] {
        let key_j = keys[j];
        let v_j = project_to_tangent(old[key_j], n_i);

        if v_j.norm() < EPS {
            continue;
        }

        acc += v_j.normalize();
        count += 1.0;
    }

    if acc.norm() < EPS || count <= 0.0 {
        return None;
    }

    Some(Target {
        direction: acc.normalize(),
        confidence: 1.0,
    })
}

/// Coherence of the induced around-directions.
///
/// For a tangent vector u_i, define:
///
///     r_i = n_i × u_i.
///
/// This term averages neighboring r_j in ambient space and then converts the
/// average around-direction back to a tangent vector at i. It encourages the
/// "around-axis" interpretation to become locally coherent without prescribing
/// a fixed global axis.
fn around_vector_coherence_target(
    cache: &SolverCache,
    old: &SlotMap<VectorKey, Vector3D>,
    keys: &[VectorKey],
    i: usize,
) -> Option<Target> {
    let n_i = cache.normals[i];

    let mut r_avg = Vector3D::new(0.0, 0.0, 0.0);
    let mut count = 0.0;

    for &j in &cache.neighbors[i] {
        let n_j = cache.normals[j];
        let key_j = keys[j];

        let u_j = safe_normalize(project_to_tangent(old[key_j], n_j), tangent_fallback(n_j));

        let r_j = n_j.cross(&u_j);

        if r_j.norm() < EPS {
            continue;
        }

        r_avg += r_j.normalize();
        count += 1.0;
    }

    if count <= 0.0 || r_avg.norm() < EPS {
        return None;
    }

    let r_avg = r_avg.normalize();

    // We want n_i × u_i ≈ r_avg.
    // A tangent solution is u_i = r_avg × n_i.
    let u_target = project_to_tangent(r_avg.cross(&n_i), n_i);

    if u_target.norm() < EPS {
        return None;
    }

    Some(Target {
        direction: u_target.normalize(),
        confidence: 1.0,
    })
}

fn apply_angle_coupling_parallel<T: Tag>(cache: &SolverCache, fields: &mut Fields<T>, weight: f64) {
    if weight <= 0.0 {
        return;
    }

    let updates: Vec<(
        VectorKey,
        Vector3D,
        VectorKey,
        Vector3D,
        VectorKey,
        Vector3D,
    )> = {
        let old_x = &fields.field_x.vectors;
        let old_y = &fields.field_y.vectors;
        let old_z = &fields.field_z.vectors;

        (0..cache.len())
            .par()
            .map(|i| {
                let kx = cache.key_x[i];
                let ky = cache.key_y[i];
                let kz = cache.key_z[i];

                let n = cache.normals[i];
                let (e1, e2) = tangent_basis(n);

                let x = safe_normalize(project_to_tangent(old_x[kx], n), e1);
                let y = safe_normalize(project_to_tangent(old_y[ky], n), e1);
                let z = safe_normalize(project_to_tangent(old_z[kz], n), e1);

                let mut tx = tangent_angle(x, e1, e2);
                let mut ty = tangent_angle(y, e1, e2);
                let mut tz = tangent_angle(z, e1, e2);

                let gx = -sin2(tx - ty) - sin2(tx - tz);
                let gy = sin2(tx - ty) - sin2(ty - tz);
                let gz = sin2(tx - tz) + sin2(ty - tz);

                tx -= weight * gx;
                ty -= weight * gy;
                tz -= weight * gz;

                let new_x = vector_from_tangent_angle(tx, e1, e2);
                let new_y = vector_from_tangent_angle(ty, e1, e2);
                let new_z = vector_from_tangent_angle(tz, e1, e2);

                (kx, new_x, ky, new_y, kz, new_z)
            })
            .collect()
    };

    for (kx, x, ky, y, kz, z) in updates {
        fields.field_x.vectors[kx] = x;
        fields.field_y.vectors[ky] = y;
        fields.field_z.vectors[kz] = z;
    }
}

fn local_edge_scale<T: Tag>(mesh: &Mesh<T>, id: ids::Key<VERT, T>) -> Option<f64> {
    let p = mesh.position(id);

    let mut sum = 0.0;
    let mut count = 0.0;

    for nb in mesh.neighbors(id) {
        let q = mesh.position(nb);
        sum += (q - p).norm();
        count += 1.0;
    }

    if count <= 0.0 {
        None
    } else {
        Some((sum / count).max(EPS))
    }
}

fn project_to_tangent(v: Vector3D, n: Vector3D) -> Vector3D {
    v - n * v.dot(&n)
}

fn tangent_basis(normal: Vector3D) -> (Vector3D, Vector3D) {
    let n = safe_normalize(normal, Vector3D::new(0.0, 0.0, 1.0));
    let e1 = tangent_fallback(n);
    let e2 = safe_normalize(n.cross(&e1), Vector3D::new(0.0, 1.0, 0.0));
    (e1, e2)
}

fn tangent_angle(v: Vector3D, e1: Vector3D, e2: Vector3D) -> f64 {
    let x = v.dot(&e1);
    let y = v.dot(&e2);
    y.atan2(x)
}

fn vector_from_tangent_angle(theta: f64, e1: Vector3D, e2: Vector3D) -> Vector3D {
    e1 * theta.cos() + e2 * theta.sin()
}

fn sin2(theta: f64) -> f64 {
    (2.0 * theta).sin()
}

fn tangent_fallback(normal: Vector3D) -> Vector3D {
    let n = safe_normalize(normal, Vector3D::new(0.0, 0.0, 1.0));

    let x_axis = Vector3D::new(1.0, 0.0, 0.0);
    let y_axis = Vector3D::new(0.0, 1.0, 0.0);

    let axis = if n.dot(&x_axis).abs() < 0.9 {
        x_axis
    } else {
        y_axis
    };

    safe_normalize(project_to_tangent(axis, n), x_axis)
}

fn safe_normalize(v: Vector3D, fallback: Vector3D) -> Vector3D {
    let len = v.norm();

    if len > EPS {
        v / len
    } else {
        let fallback_len = fallback.norm();

        if fallback_len > EPS {
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

fn field_axis_energy(
    cache: &SolverCache,
    vectors: &SlotMap<VectorKey, Vector3D>,
    keys: &[VectorKey],
    axis: Vector3D,
) -> f64 {
    (0..cache.len())
        .map(|i| {
            let n = cache.normals[i];
            let u = safe_normalize(project_to_tangent(vectors[keys[i]], n), tangent_fallback(n));

            let Some(target) = around_axis_target(n, axis) else {
                return 0.0;
            };

            let diff = u - target.direction;
            target.confidence * target.confidence * diff.dot(&diff)
        })
        .sum()
}

fn field_curvature_energy(
    cache: &SolverCache,
    vectors: &SlotMap<VectorKey, Vector3D>,
    keys: &[VectorKey],
) -> f64 {
    (0..cache.len())
        .map(|i| {
            let n = cache.normals[i];
            let u = safe_normalize(project_to_tangent(vectors[keys[i]], n), tangent_fallback(n));

            let Some(target) = curvature_target_from_cache(cache.curvature[i], u) else {
                return 0.0;
            };

            let diff = u - target.direction;
            target.confidence * diff.dot(&diff)
        })
        .sum()
}

fn field_smooth_energy(
    cache: &SolverCache,
    vectors: &SlotMap<VectorKey, Vector3D>,
    keys: &[VectorKey],
) -> f64 {
    let mut energy = 0.0;

    for i in 0..cache.len() {
        let n_i = cache.normals[i];
        let u_i = safe_normalize(
            project_to_tangent(vectors[keys[i]], n_i),
            tangent_fallback(n_i),
        );

        for &j in &cache.neighbors[i] {
            if j <= i {
                continue;
            }

            let u_j = safe_normalize(
                project_to_tangent(vectors[keys[j]], n_i),
                tangent_fallback(n_i),
            );

            let diff = u_i - u_j;
            energy += diff.dot(&diff);
        }
    }

    energy
}

fn field_coherence_energy(
    cache: &SolverCache,
    vectors: &SlotMap<VectorKey, Vector3D>,
    keys: &[VectorKey],
) -> f64 {
    let mut energy = 0.0;

    for i in 0..cache.len() {
        let n_i = cache.normals[i];

        let u_i = safe_normalize(
            project_to_tangent(vectors[keys[i]], n_i),
            tangent_fallback(n_i),
        );

        let r_i = safe_normalize(n_i.cross(&u_i), tangent_fallback(n_i));

        for &j in &cache.neighbors[i] {
            if j <= i {
                continue;
            }

            let n_j = cache.normals[j];

            let u_j = safe_normalize(
                project_to_tangent(vectors[keys[j]], n_j),
                tangent_fallback(n_j),
            );

            let r_j = safe_normalize(n_j.cross(&u_j), tangent_fallback(n_j));

            let diff = r_i - r_j;
            energy += diff.dot(&diff);
        }
    }

    energy
}

fn coupling_energy<T: Tag>(cache: &SolverCache, fields: &Fields<T>) -> f64 {
    let mut energy = 0.0;

    for i in 0..cache.len() {
        let n = cache.normals[i];

        let x = safe_normalize(
            project_to_tangent(fields.field_x.vectors[cache.key_x[i]], n),
            tangent_fallback(n),
        );
        let y = safe_normalize(
            project_to_tangent(fields.field_y.vectors[cache.key_y[i]], n),
            tangent_fallback(n),
        );
        let z = safe_normalize(
            project_to_tangent(fields.field_z.vectors[cache.key_z[i]], n),
            tangent_fallback(n),
        );

        let xy = x.dot(&y);
        let xz = x.dot(&z);
        let yz = y.dot(&z);

        energy += xy * xy + xz * xz + yz * yz;
    }

    energy
}
