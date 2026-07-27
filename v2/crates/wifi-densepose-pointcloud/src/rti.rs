//! Geometry-based backprojection and nonnegative regularized RTI solver.
//!
//! The production output uses a projected-gradient inverse solver. The legacy-style
//! backprojection is retained as an explicit diagnostic comparator and fallback only.

use serde::{Deserialize, Serialize};

const HEIGHT_LAYER_CENTERS_M: [f32; 3] = [0.30, 0.95, 1.70];
const HEIGHT_LAYER_SIGMA_M: f32 = 0.42;
const EPSILON: f32 = 1.0e-6;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SolverKind {
    Backprojection,
    NonnegativeRti,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RtiConfig {
    pub lambda_smooth: f32,
    pub lambda_sparse: f32,
    pub iterations: usize,
    pub convergence_tolerance: f32,
    pub fresnel_width_m: f32,
    pub minimum_link_quality: f32,
    pub minimum_links: usize,
    pub temporal_alpha: f32,
}

impl Default for RtiConfig {
    fn default() -> Self {
        Self {
            lambda_smooth: 0.12,
            lambda_sparse: 0.018,
            iterations: 60,
            convergence_tolerance: 1.0e-4,
            fresnel_width_m: 0.65,
            minimum_link_quality: 0.12,
            minimum_links: 3,
            temporal_alpha: 0.32,
        }
    }
}

impl RtiConfig {
    pub fn normalized(mut self) -> Self {
        self.lambda_smooth = self.lambda_smooth.clamp(0.0, 5.0);
        self.lambda_sparse = self.lambda_sparse.clamp(0.0, 2.0);
        self.iterations = self.iterations.clamp(5, 500);
        self.convergence_tolerance = self.convergence_tolerance.clamp(1.0e-7, 0.1);
        self.fresnel_width_m = self.fresnel_width_m.clamp(0.05, 5.0);
        self.minimum_link_quality = self.minimum_link_quality.clamp(0.0, 1.0);
        self.minimum_links = self.minimum_links.clamp(1, 64);
        self.temporal_alpha = self.temporal_alpha.clamp(0.01, 1.0);
        self
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct RtiLinkMeasurement {
    pub link_id: u32,
    pub transmitter_m: [f32; 3],
    pub receiver_m: [f32; 3],
    pub observation: f32,
    pub quality: f32,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct SolverComparison {
    pub active_links: usize,
    pub cell_count: usize,
    pub solver_kind: String,
    pub fallback_used: bool,
    pub iterations: usize,
    pub converged: bool,
    pub nonnegative_residual_rmse: f32,
    pub backprojection_residual_rmse: f32,
    pub centroid_distance_m: Option<f32>,
    pub nonnegative_peak: f32,
    pub backprojection_peak: f32,
}

#[derive(Clone, Debug, Serialize)]
pub struct RtiVolume {
    pub grid_size: [usize; 3],
    pub dense: Vec<f32>,
    pub low_mid_high_energy: [f32; 3],
    pub centroid_m: Option<[f32; 3]>,
    pub comparison: SolverComparison,
}

#[derive(Clone, Debug)]
struct WeightCache {
    signature: u64,
    horizontal_grid: [usize; 2],
    weights: Vec<Vec<f32>>,
}

pub struct RtiEngine {
    config: RtiConfig,
    previous_layers: Vec<f32>,
    cache: Option<WeightCache>,
}

impl Default for RtiEngine {
    fn default() -> Self {
        Self::new(RtiConfig::default())
    }
}

impl RtiEngine {
    pub fn new(config: RtiConfig) -> Self {
        Self {
            config: config.normalized(),
            previous_layers: Vec::new(),
            cache: None,
        }
    }

    pub fn invalidate_geometry(&mut self) {
        self.cache = None;
        self.previous_layers.clear();
    }

    pub fn config(&self) -> &RtiConfig {
        &self.config
    }

    pub fn solve(
        &mut self,
        room_size_m: [f32; 3],
        output_grid: [usize; 3],
        measurements: &[RtiLinkMeasurement],
    ) -> RtiVolume {
        let [nx, ny, nz] = output_grid;
        let horizontal_grid = [nx.max(2), nz.max(2)];
        let active = measurements
            .iter()
            .filter(|measurement| {
                measurement.observation.is_finite()
                    && measurement.quality >= self.config.minimum_link_quality
                    && measurement.transmitter_m.iter().all(|value| value.is_finite())
                    && measurement.receiver_m.iter().all(|value| value.is_finite())
            })
            .cloned()
            .collect::<Vec<_>>();
        let layer_cells = horizontal_grid[0] * horizontal_grid[1] * 3;
        if active.is_empty() {
            self.previous_layers.iter_mut().for_each(|value| *value *= 0.85);
            let layers = if self.previous_layers.len() == layer_cells {
                self.previous_layers.clone()
            } else {
                vec![0.0; layer_cells]
            };
            return self.to_volume(
                room_size_m,
                output_grid,
                &layers,
                SolverComparison {
                    active_links: 0,
                    cell_count: layer_cells,
                    solver_kind: "unavailable".to_string(),
                    ..SolverComparison::default()
                },
            );
        }

        let signature = geometry_signature(room_size_m, horizontal_grid, &active, self.config.fresnel_width_m);
        let rebuild = self
            .cache
            .as_ref()
            .is_none_or(|cache| cache.signature != signature);
        if rebuild {
            self.cache = Some(WeightCache {
                signature,
                horizontal_grid,
                weights: build_weight_matrix(
                    room_size_m,
                    horizontal_grid,
                    &active,
                    self.config.fresnel_width_m,
                ),
            });
            self.previous_layers = vec![0.0; layer_cells];
        }
        let cache = self.cache.as_ref().expect("RTI weight cache");
        debug_assert_eq!(cache.horizontal_grid, horizontal_grid);
        let observations = active
            .iter()
            .map(|measurement| measurement.observation.clamp(0.0, 2.0))
            .collect::<Vec<_>>();
        let qualities = active
            .iter()
            .map(|measurement| measurement.quality.clamp(0.0, 1.0))
            .collect::<Vec<_>>();

        let backprojection = backproject(&cache.weights, &observations, &qualities, layer_cells);
        let (inverse, iterations, converged) = if active.len() >= self.config.minimum_links {
            projected_gradient(
                &cache.weights,
                &observations,
                &qualities,
                horizontal_grid,
                &self.previous_layers,
                &self.config,
            )
        } else {
            (backprojection.clone(), 0, false)
        };
        let fallback_used = active.len() < self.config.minimum_links;
        let selected = if fallback_used { &backprojection } else { &inverse };

        if self.previous_layers.len() != selected.len() {
            self.previous_layers = selected.clone();
        } else {
            for (previous, current) in self.previous_layers.iter_mut().zip(selected) {
                *previous = *previous * (1.0 - self.config.temporal_alpha)
                    + *current * self.config.temporal_alpha;
            }
        }

        let inverse_residual = residual_rmse(&cache.weights, &inverse, &observations, &qualities);
        let backprojection_residual =
            residual_rmse(&cache.weights, &backprojection, &observations, &qualities);
        let inverse_centroid = layer_centroid(room_size_m, horizontal_grid, &inverse);
        let backprojection_centroid = layer_centroid(room_size_m, horizontal_grid, &backprojection);
        let centroid_distance_m = match (inverse_centroid, backprojection_centroid) {
            (Some(left), Some(right)) => Some(distance(left, right)),
            _ => None,
        };
        let comparison = SolverComparison {
            active_links: active.len(),
            cell_count: layer_cells,
            solver_kind: if fallback_used {
                "backprojection_fallback".to_string()
            } else {
                "nonnegative_rti".to_string()
            },
            fallback_used,
            iterations,
            converged,
            nonnegative_residual_rmse: inverse_residual,
            backprojection_residual_rmse: backprojection_residual,
            centroid_distance_m,
            nonnegative_peak: inverse.iter().copied().fold(0.0, f32::max),
            backprojection_peak: backprojection.iter().copied().fold(0.0, f32::max),
        };
        let layers = self.previous_layers.clone();
        self.to_volume(room_size_m, output_grid, &layers, comparison)
    }

    fn to_volume(
        &self,
        room_size_m: [f32; 3],
        output_grid: [usize; 3],
        layers: &[f32],
        comparison: SolverComparison,
    ) -> RtiVolume {
        let [nx, ny, nz] = output_grid;
        let mut dense = vec![0.0f32; nx * ny * nz];
        let mut layer_energy = [0.0f32; 3];
        let mut layer_counts = [0usize; 3];
        for layer in 0..3 {
            for z in 0..nz {
                for x in 0..nx {
                    let index = layer_index(x, layer, z, nx, nz);
                    if let Some(value) = layers.get(index).copied() {
                        layer_energy[layer] += value;
                        layer_counts[layer] += 1;
                    }
                }
            }
        }
        for layer in 0..3 {
            if layer_counts[layer] > 0 {
                layer_energy[layer] /= layer_counts[layer] as f32;
            }
        }

        for y in 0..ny {
            let height = (y as f32 + 0.5) * room_size_m[1] / ny.max(1) as f32;
            let mut layer_weights = [0.0f32; 3];
            let mut sum = 0.0f32;
            for layer in 0..3 {
                let delta = height - HEIGHT_LAYER_CENTERS_M[layer].min(room_size_m[1]);
                let weight = (-(delta * delta) / (2.0 * HEIGHT_LAYER_SIGMA_M.powi(2))).exp();
                layer_weights[layer] = weight;
                sum += weight;
            }
            for z in 0..nz {
                for x in 0..nx {
                    let mut probability = 0.0f32;
                    for layer in 0..3 {
                        let layer_value = layers
                            .get(layer_index(x, layer, z, nx, nz))
                            .copied()
                            .unwrap_or(0.0);
                        probability += layer_value * layer_weights[layer] / sum.max(EPSILON);
                    }
                    dense[z * ny * nx + y * nx + x] = probability.clamp(0.0, 1.0);
                }
            }
        }
        let centroid_m = dense_centroid(room_size_m, output_grid, &dense);
        RtiVolume {
            grid_size: output_grid,
            dense,
            low_mid_high_energy: layer_energy,
            centroid_m,
            comparison,
        }
    }
}

fn build_weight_matrix(
    room_size_m: [f32; 3],
    grid: [usize; 2],
    measurements: &[RtiLinkMeasurement],
    fresnel_width_m: f32,
) -> Vec<Vec<f32>> {
    let [nx, nz] = grid;
    measurements
        .iter()
        .map(|measurement| {
            let mut row = vec![0.0f32; nx * nz * 3];
            let direct = distance(measurement.transmitter_m, measurement.receiver_m).max(0.01);
            let horizontal_dx = measurement.receiver_m[0] - measurement.transmitter_m[0];
            let horizontal_dz = measurement.receiver_m[2] - measurement.transmitter_m[2];
            let horizontal_length_sq = horizontal_dx * horizontal_dx + horizontal_dz * horizontal_dz;
            let mut row_sum = 0.0f32;
            for layer in 0..3 {
                let y = HEIGHT_LAYER_CENTERS_M[layer].min(room_size_m[1] * 0.95);
                for z in 0..nz {
                    for x in 0..nx {
                        let point = [
                            (x as f32 + 0.5) * room_size_m[0] / nx as f32,
                            y,
                            (z as f32 + 0.5) * room_size_m[2] / nz as f32,
                        ];
                        let excess = distance(measurement.transmitter_m, point)
                            + distance(measurement.receiver_m, point)
                            - direct;
                        let fresnel = (-(excess * excess)
                            / (2.0 * fresnel_width_m.max(0.05).powi(2)))
                        .exp();
                        let projection = if horizontal_length_sq > EPSILON {
                            (((point[0] - measurement.transmitter_m[0]) * horizontal_dx
                                + (point[2] - measurement.transmitter_m[2]) * horizontal_dz)
                                / horizontal_length_sq)
                                .clamp(0.0, 1.0)
                        } else {
                            0.5
                        };
                        let expected_height = measurement.transmitter_m[1]
                            + (measurement.receiver_m[1] - measurement.transmitter_m[1]) * projection;
                        let height_delta = point[1] - expected_height;
                        let height_weight = (-(height_delta * height_delta)
                            / (2.0 * HEIGHT_LAYER_SIGMA_M.powi(2)))
                        .exp();
                        let value = fresnel * height_weight;
                        let index = layer_index(x, layer, z, nx, nz);
                        row[index] = value;
                        row_sum += value;
                    }
                }
            }
            if row_sum > EPSILON {
                row.iter_mut().for_each(|value| *value /= row_sum);
            }
            row
        })
        .collect()
}

fn backproject(
    weights: &[Vec<f32>],
    observations: &[f32],
    qualities: &[f32],
    cell_count: usize,
) -> Vec<f32> {
    let mut numerator = vec![0.0f32; cell_count];
    let mut denominator = vec![0.0f32; cell_count];
    for row in 0..weights.len() {
        let quality = qualities.get(row).copied().unwrap_or(0.0);
        let observation = observations.get(row).copied().unwrap_or(0.0);
        for cell in 0..cell_count {
            let weight = weights[row].get(cell).copied().unwrap_or(0.0) * quality;
            numerator[cell] += weight * observation;
            denominator[cell] += weight;
        }
    }
    numerator
        .into_iter()
        .zip(denominator)
        .map(|(value, weight)| {
            if weight > EPSILON {
                (value / weight).clamp(0.0, 1.0)
            } else {
                0.0
            }
        })
        .collect()
}

fn projected_gradient(
    weights: &[Vec<f32>],
    observations: &[f32],
    qualities: &[f32],
    grid: [usize; 2],
    warm_start: &[f32],
    config: &RtiConfig,
) -> (Vec<f32>, usize, bool) {
    let cell_count = grid[0] * grid[1] * 3;
    let mut x = if warm_start.len() == cell_count {
        warm_start.to_vec()
    } else {
        vec![0.0; cell_count]
    };
    let mut column_energy = vec![0.0f32; cell_count];
    for (row_index, row) in weights.iter().enumerate() {
        let quality = qualities.get(row_index).copied().unwrap_or(0.0);
        for cell in 0..cell_count {
            let value = row.get(cell).copied().unwrap_or(0.0) * quality;
            column_energy[cell] += value * value;
        }
    }
    let lipschitz = column_energy
        .iter()
        .copied()
        .fold(0.0, f32::max)
        + config.lambda_smooth * 12.0
        + 1.0e-3;
    let step = 0.9 / lipschitz.max(1.0e-3);
    let mut converged = false;
    let mut completed_iterations = 0usize;

    for iteration in 0..config.iterations {
        let predictions = matrix_vector(weights, &x);
        let mut gradient = vec![0.0f32; cell_count];
        for row_index in 0..weights.len() {
            let quality = qualities.get(row_index).copied().unwrap_or(0.0);
            let residual = (predictions[row_index]
                - observations.get(row_index).copied().unwrap_or(0.0))
                * quality
                * quality;
            for cell in 0..cell_count {
                gradient[cell] += weights[row_index][cell] * residual;
            }
        }
        add_smoothness_gradient(&x, &mut gradient, grid, config.lambda_smooth);
        let mut change_sq = 0.0f32;
        let mut norm_sq = 0.0f32;
        for cell in 0..cell_count {
            let next = (x[cell] - step * (gradient[cell] + config.lambda_sparse))
                .max(0.0)
                .min(1.0);
            let change = next - x[cell];
            change_sq += change * change;
            norm_sq += x[cell] * x[cell];
            x[cell] = next;
        }
        completed_iterations = iteration + 1;
        let relative_change = change_sq.sqrt() / norm_sq.sqrt().max(1.0e-4);
        if relative_change <= config.convergence_tolerance {
            converged = true;
            break;
        }
    }
    (x, completed_iterations, converged)
}

fn add_smoothness_gradient(
    values: &[f32],
    gradient: &mut [f32],
    grid: [usize; 2],
    lambda: f32,
) {
    if lambda <= 0.0 {
        return;
    }
    let [nx, nz] = grid;
    for layer in 0..3 {
        for z in 0..nz {
            for x in 0..nx {
                let index = layer_index(x, layer, z, nx, nz);
                let center = values[index];
                let mut neighbor_sum = 0.0f32;
                let mut count = 0.0f32;
                for (dx, dz, dl) in [
                    (-1i32, 0i32, 0i32),
                    (1, 0, 0),
                    (0, -1, 0),
                    (0, 1, 0),
                    (0, 0, -1),
                    (0, 0, 1),
                ] {
                    let xx = x as i32 + dx;
                    let zz = z as i32 + dz;
                    let ll = layer as i32 + dl;
                    if xx >= 0
                        && xx < nx as i32
                        && zz >= 0
                        && zz < nz as i32
                        && ll >= 0
                        && ll < 3
                    {
                        neighbor_sum += values[layer_index(
                            xx as usize,
                            ll as usize,
                            zz as usize,
                            nx,
                            nz,
                        )];
                        count += 1.0;
                    }
                }
                gradient[index] += lambda * (center * count - neighbor_sum);
            }
        }
    }
}

fn matrix_vector(matrix: &[Vec<f32>], vector: &[f32]) -> Vec<f32> {
    matrix
        .iter()
        .map(|row| row.iter().zip(vector).map(|(left, right)| left * right).sum())
        .collect()
}

fn residual_rmse(
    weights: &[Vec<f32>],
    values: &[f32],
    observations: &[f32],
    qualities: &[f32],
) -> f32 {
    if weights.is_empty() {
        return 0.0;
    }
    let predictions = matrix_vector(weights, values);
    let error = predictions
        .iter()
        .zip(observations)
        .zip(qualities)
        .map(|((prediction, observation), quality)| {
            let delta = (prediction - observation) * quality;
            delta * delta
        })
        .sum::<f32>();
    (error / weights.len() as f32).sqrt()
}

fn layer_centroid(
    room_size_m: [f32; 3],
    grid: [usize; 2],
    values: &[f32],
) -> Option<[f32; 3]> {
    let [nx, nz] = grid;
    let mut weighted = [0.0f32; 3];
    let mut total = 0.0f32;
    for layer in 0..3 {
        for z in 0..nz {
            for x in 0..nx {
                let value = values
                    .get(layer_index(x, layer, z, nx, nz))
                    .copied()
                    .unwrap_or(0.0);
                if value <= 0.0 {
                    continue;
                }
                weighted[0] += (x as f32 + 0.5) * room_size_m[0] / nx as f32 * value;
                weighted[1] += HEIGHT_LAYER_CENTERS_M[layer].min(room_size_m[1]) * value;
                weighted[2] += (z as f32 + 0.5) * room_size_m[2] / nz as f32 * value;
                total += value;
            }
        }
    }
    (total > EPSILON).then(|| {
        [
            weighted[0] / total,
            weighted[1] / total,
            weighted[2] / total,
        ]
    })
}

fn dense_centroid(
    room_size_m: [f32; 3],
    grid: [usize; 3],
    values: &[f32],
) -> Option<[f32; 3]> {
    let [nx, ny, nz] = grid;
    let mut weighted = [0.0f32; 3];
    let mut total = 0.0f32;
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let value = values.get(z * ny * nx + y * nx + x).copied().unwrap_or(0.0);
                if value <= 0.0 {
                    continue;
                }
                weighted[0] += (x as f32 + 0.5) * room_size_m[0] / nx as f32 * value;
                weighted[1] += (y as f32 + 0.5) * room_size_m[1] / ny as f32 * value;
                weighted[2] += (z as f32 + 0.5) * room_size_m[2] / nz as f32 * value;
                total += value;
            }
        }
    }
    (total > EPSILON).then(|| {
        [
            weighted[0] / total,
            weighted[1] / total,
            weighted[2] / total,
        ]
    })
}

fn layer_index(x: usize, layer: usize, z: usize, nx: usize, nz: usize) -> usize {
    debug_assert!(z < nz);
    layer * nx * nz + z * nx + x
}

fn distance(left: [f32; 3], right: [f32; 3]) -> f32 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    let dz = left[2] - right[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn geometry_signature(
    room: [f32; 3],
    grid: [usize; 2],
    measurements: &[RtiLinkMeasurement],
    fresnel: f32,
) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    for value in room
        .into_iter()
        .chain([grid[0] as f32, grid[1] as f32, fresnel])
        .chain(measurements.iter().flat_map(|measurement| {
            measurement
                .transmitter_m
                .into_iter()
                .chain(measurement.receiver_m)
                .chain([measurement.link_id as f32])
        }))
    {
        for byte in value.to_bits().to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nonnegative_solver_returns_finite_volume() {
        let mut engine = RtiEngine::new(RtiConfig {
            iterations: 30,
            ..RtiConfig::default()
        });
        let links = vec![
            RtiLinkMeasurement {
                link_id: 1,
                transmitter_m: [0.1, 0.5, 0.1],
                receiver_m: [3.9, 1.7, 3.9],
                observation: 0.8,
                quality: 0.9,
            },
            RtiLinkMeasurement {
                link_id: 2,
                transmitter_m: [3.9, 0.5, 0.1],
                receiver_m: [0.1, 1.7, 3.9],
                observation: 0.75,
                quality: 0.9,
            },
            RtiLinkMeasurement {
                link_id: 3,
                transmitter_m: [0.1, 1.0, 3.9],
                receiver_m: [3.9, 1.0, 0.1],
                observation: 0.65,
                quality: 0.85,
            },
        ];
        let volume = engine.solve([4.0, 2.4, 4.0], [20, 10, 20], &links);
        assert_eq!(volume.dense.len(), 20 * 10 * 20);
        assert!(volume.dense.iter().all(|value| value.is_finite() && *value >= 0.0));
        assert!(!volume.comparison.fallback_used);
    }

    #[test]
    fn insufficient_links_use_explicit_fallback() {
        let mut engine = RtiEngine::default();
        let links = vec![RtiLinkMeasurement {
            link_id: 1,
            transmitter_m: [0.0, 0.5, 0.0],
            receiver_m: [4.0, 0.5, 4.0],
            observation: 0.7,
            quality: 1.0,
        }];
        let volume = engine.solve([4.0, 2.4, 4.0], [20, 10, 20], &links);
        assert!(volume.comparison.fallback_used);
        assert_eq!(volume.comparison.solver_kind, "backprojection_fallback");
    }
}
