//! Unified WLS solver dispatch and Gauss-Newton iteration loop.
//!
//! Dispatches to the appropriate solver formulation based on `EstimationConfig`.
//! The Normal Equations (NE) path supports dense/sparse Cholesky and LU backends.
//! Alternative formulations (QR, Peters-Wilkinson, Equality-Constrained,
//! Fast-Decoupled, DC) are implemented in dedicated modules.

use std::time::Instant;

use sprs::CsMat;

use crate::model::{AcModel, MeasurementSet, PowerSystem};
use super::gain::{self, GainCscCache};
use super::factorize;
use super::jacobian;
use super::types::*;

/// The built-in Rust WLS solver implementing the `SeSolver` trait.
/// Dispatches to the formulation specified in `EstimationConfig`.
pub struct WlsSolver;

impl SeSolver for WlsSolver {
    fn estimate(
        &self,
        system: &PowerSystem,
        model: &AcModel,
        measurements: &MeasurementSet,
        config: &EstimationConfig,
    ) -> Result<EstimationResult, SolverError> {
        match &config.formulation {
            SolverFormulation::NormalEquations { .. } => {
                gauss_newton(system, model, measurements, config)
            }
            SolverFormulation::OrthogonalQR => {
                super::qr_givens::solve_qr(system, model, measurements, config)
            }
            SolverFormulation::PetersWilkinson => {
                super::peters_wilkinson::solve_pw(system, model, measurements, config)
            }
            SolverFormulation::EqualityConstrained { .. } => {
                super::equality_constrained::solve_ec(system, model, measurements, config)
            }
            SolverFormulation::FastDecoupled => {
                super::fast_decoupled::solve_fd(system, model, measurements, config)
            }
            SolverFormulation::DcEstimation => {
                super::dc_estimation::solve_dc(system, model, measurements, config)
            }
        }
    }
}

/// Run the AC WLS state estimation using the Normal Equations (Gauss-Newton).
///
/// Returns solver artifacts from the final iteration for use by
/// post-estimation analysis (BDD, observability, etc.).
pub fn gauss_newton(
    system: &PowerSystem,
    model: &AcModel,
    measurements: &MeasurementSet,
    config: &EstimationConfig,
) -> Result<EstimationResult, SolverError> {
    let n = system.n_buses();
    let s = 2 * n;
    let slack = system.slack_index;

    if n == 0 {
        return Err(SolverError::InvalidInput("system has no buses".into()));
    }
    if measurements.n_equations() == 0 {
        return Err(SolverError::InvalidInput("no active measurements".into()));
    }

    let mut v_mag: Vec<f64> = system.buses.iter().map(|b| b.voltage_magnitude).collect();
    let mut theta: Vec<f64> = system.buses.iter().map(|b| b.voltage_angle).collect();

    let mut converged = false;
    let mut final_inc = f64::MAX;
    let mut iter = 0;
    let mut diagnostics = Vec::with_capacity(config.max_iterations);

    let backend = config.ne_factorization();

    let mut gain_buf = if backend == Factorization::DenseCholesky {
        vec![0.0f64; s * s]
    } else {
        Vec::new()
    };
    let mut rhs_buf = vec![0.0f64; s];

    let mut gain_cache: Option<GainCscCache> = None;
    let mut symbolic_llt: Option<faer::sparse::linalg::solvers::SymbolicLlt<usize>> = None;
    let mut symbolic_lu: Option<faer::sparse::linalg::solvers::SymbolicLu<usize>> = None;

    // Artifacts from the last iteration (captured for BDD).
    let mut last_jac_csc: Option<CsMat<f64>> = None;
    let mut last_h: Option<Vec<f64>> = None;
    let mut last_z: Option<Vec<f64>> = None;
    let mut last_r: Option<Vec<f64>> = None;
    let mut last_w_diag: Option<Vec<f64>> = None;
    let mut last_w_off: Option<Vec<(usize, usize, f64)>> = None;

    for nu in 0..=config.max_iterations {
        iter = nu;
        let iter_start = Instant::now();

        let t0 = Instant::now();
        let (h_vec, jac_tri, z_vec, w_diag, w_off) =
            jacobian::evaluate(system, model, measurements, &theta, &v_mag);
        let jac_time = t0.elapsed().as_secs_f64();

        let k = h_vec.len();
        let r: Vec<f64> = z_vec.iter().zip(h_vec.iter()).map(|(z, h)| z - h).collect();

        let t0 = Instant::now();
        let jac_csc: CsMat<f64> = jac_tri.to_csc();

        let delta_x = match backend {
            Factorization::DenseCholesky => {
                rhs_buf.iter_mut().for_each(|x| *x = 0.0);
                gain::build_gain_dense(&jac_csc, &w_diag, &w_off, &r, k, s, &mut gain_buf, &mut rhs_buf);
                gain::apply_slack_dense(&mut gain_buf, &mut rhs_buf, slack, s);
                let gain_time = t0.elapsed().as_secs_f64();

                let t1 = Instant::now();
                let dx = factorize::solve_dense_faer(&gain_buf, &rhs_buf, s);
                let solve_time = t1.elapsed().as_secs_f64();

                diagnostics.push(IterationDiagnostic {
                    iteration: nu,
                    max_delta_x: dx.iter().fold(0.0f64, |a, &x| a.max(x.abs())),
                    jacobian_time_s: jac_time,
                    gain_time_s: gain_time,
                    solve_time_s: solve_time,
                    total_time_s: iter_start.elapsed().as_secs_f64(),
                });

                gain_buf.iter_mut().for_each(|x| *x = 0.0);
                dx
            }
            Factorization::SparseCholesky | Factorization::SparseLU => {
                let j_csr = jac_csc.to_csr();

                let mut wr = vec![0.0f64; k];
                for i in 0..k { wr[i] = w_diag[i] * r[i]; }
                for &(r1, r2, val) in &w_off {
                    wr[r1] += val * r[r2];
                    wr[r2] += val * r[r1];
                }

                if gain_cache.is_none() {
                    gain_cache = Some(GainCscCache::from_jacobian(&j_csr, s, slack));
                }
                let cache = gain_cache.as_mut().unwrap();

                rhs_buf.iter_mut().for_each(|x| *x = 0.0);
                cache.fill_values(&j_csr, &w_diag, &w_off, slack, &mut rhs_buf, &wr, &jac_csc);
                let gain_time = t0.elapsed().as_secs_f64();

                let t1 = Instant::now();
                let gain_view = cache.as_csc_view();
                let dx = factorize::solve_sparse_faer(
                    &gain_view, &rhs_buf, s, backend,
                    &mut symbolic_llt, &mut symbolic_lu,
                );
                let solve_time = t1.elapsed().as_secs_f64();

                diagnostics.push(IterationDiagnostic {
                    iteration: nu,
                    max_delta_x: dx.iter().fold(0.0f64, |a, &x| a.max(x.abs())),
                    jacobian_time_s: jac_time,
                    gain_time_s: gain_time,
                    solve_time_s: solve_time,
                    total_time_s: iter_start.elapsed().as_secs_f64(),
                });
                dx
            }
        };

        // Capture artifacts from this iteration (overwritten each time;
        // the final capture is the one that matters).
        last_jac_csc = Some(jac_csc);
        last_h = Some(h_vec);
        last_z = Some(z_vec);
        last_r = Some(r);
        last_w_diag = Some(w_diag);
        last_w_off = Some(w_off);

        final_inc = delta_x.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));

        if final_inc < config.tolerance {
            converged = true;
            break;
        }

        for i in 0..n {
            theta[i] += delta_x[i];
            v_mag[i] += delta_x[n + i];
        }
    }

    // Build solver artifacts from the final iteration.
    let artifacts = if let (Some(jac), Some(h), Some(z), Some(r), Some(wd), Some(wo)) =
        (last_jac_csc, last_h, last_z, last_r, last_w_diag, last_w_off)
    {
        let gain_mat = match backend {
            Factorization::DenseCholesky => build_gain_csc_from_dense(&gain_buf, s),
            _ => {
                let c = gain_cache.as_ref().unwrap();
                CsMat::new(
                    (s, s),
                    c.col_ptr.clone(),
                    c.row_idx.clone(),
                    c.values.clone(),
                )
            }
        };
        let mmap = build_measurement_map(measurements);

        Some(SolverArtifacts {
            jacobian: jac,
            residuals: r,
            measurement_z: z,
            measurement_h: h,
            precision_diag: wd,
            precision_off: wo,
            gain_matrix: gain_mat,
            n_states: s,
            slack_index: slack,
            measurement_map: mmap,
        })
    } else {
        None
    };

    Ok(EstimationResult {
        voltage_magnitude: v_mag,
        voltage_angle: theta,
        converged,
        iterations: iter,
        final_increment: final_inc,
        diagnostics,
        artifacts,
    })
}

/// Build a CSC gain matrix from the dense buffer (used only for DenseCholesky path).
fn build_gain_csc_from_dense(gain_flat: &[f64], s: usize) -> CsMat<f64> {
    use sprs::TriMat;
    let mut tri = TriMat::new((s, s));
    for r in 0..s {
        for c in 0..s {
            let v = gain_flat[r * s + c];
            if v.abs() > 1e-20 {
                tri.add_triplet(r, c, v);
            }
        }
    }
    tri.to_csc()
}

/// Build the measurement ordering map so BDD can identify which measurement
/// each equation row corresponds to.
pub(crate) fn build_measurement_map(measurements: &MeasurementSet) -> Vec<MeasurementRef> {
    let mut map = Vec::with_capacity(measurements.n_equations());
    for m in measurements.voltmeters.iter().filter(|m| m.status) {
        map.push(MeasurementRef::Voltmeter { label: m.label.clone() });
    }
    for m in measurements.ammeters.iter().filter(|m| m.status) {
        map.push(MeasurementRef::Ammeter { label: m.label.clone() });
    }
    for m in measurements.wattmeters.iter().filter(|m| m.status) {
        map.push(MeasurementRef::Wattmeter { label: m.label.clone() });
    }
    for m in measurements.varmeters.iter().filter(|m| m.status) {
        map.push(MeasurementRef::Varmeter { label: m.label.clone() });
    }
    for m in measurements.pmus.iter().filter(|m| m.status) {
        map.push(MeasurementRef::PmuMagnitude { label: m.label.clone() });
        map.push(MeasurementRef::PmuAngle { label: m.label.clone() });
    }
    for m in measurements.current_angle_meters.iter().filter(|m| m.status) {
        map.push(MeasurementRef::CurrentAngle { label: m.label.clone() });
    }
    map
}
