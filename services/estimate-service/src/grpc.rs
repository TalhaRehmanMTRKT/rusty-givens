//! gRPC service implementations.
//!
//! Free:  EstimateService (GetNetwork, GetTrueState, RunEstimate, GetLastResult)
//! Pro:   BddService, ObsService, RedService

use std::sync::Arc;

use tonic::{Request, Response, Status};

use rusty_givens_core::kernel::{Factorization, SolverFormulation};

#[cfg(feature = "pro")]
use rusty_givens_bdd::{chi_squared_test, rn_max_test, BddConfig};
#[cfg(feature = "pro")]
use rusty_givens_obs::{analyze_observability, ObsConfig, ObsMethod};

use crate::pb::rusty_givens::v1 as pb;
use crate::pb::rusty_givens::v1::estimate_service_server::EstimateService;
#[cfg(feature = "pro")]
use crate::pb::rusty_givens::v1::bdd_service_server::BddService;
#[cfg(feature = "pro")]
use crate::pb::rusty_givens::v1::obs_service_server::ObsService;
#[cfg(feature = "pro")]
use crate::pb::rusty_givens::v1::red_service_server::RedService;

use crate::{AppState, BusResult, SeResultPayload, execute_estimation};
#[cfg(feature = "pro")]
use crate::{BddPayload, meas_ref_type, meas_ref_label};

// ── EstimateService (Free) ──────────────────────────────────────────

pub struct EstimateServiceImpl {
    pub state: Arc<AppState>,
}

#[tonic::async_trait]
impl EstimateService for EstimateServiceImpl {
    async fn get_network(
        &self,
        _req: Request<pb::Empty>,
    ) -> Result<Response<pb::NetworkResponse>, Status> {
        let state = &self.state;
        let buses: Vec<pb::BusNode> = state
            .bus_metadata
            .iter()
            .enumerate()
            .map(|(i, b)| pb::BusNode {
                index: i as u32,
                label: b.label as u32,
                vn_kv: b.vn_kv,
                bus_type: b.bus_type as u32,
                geo_x: b.geo_x,
                geo_y: b.geo_y,
            })
            .collect();

        let branches: Vec<pb::BranchEdge> = state
            .system
            .branches
            .iter()
            .enumerate()
            .filter(|(_, b)| b.status)
            .map(|(i, b)| pb::BranchEdge {
                index: i as u32,
                from_bus: b.from as u32,
                to_bus: b.to as u32,
            })
            .collect();

        Ok(Response::new(pb::NetworkResponse {
            n_buses: state.info.n_buses as u32,
            n_branches: state.info.n_branches as u32,
            slack_bus_index: state.info.slack_bus_index as u32,
            base_mva: state.info.base_mva,
            buses,
            branches,
        }))
    }

    async fn get_true_state(
        &self,
        _req: Request<pb::Empty>,
    ) -> Result<Response<pb::TrueStateResponse>, Status> {
        let ts = &self.state.true_state;
        Ok(Response::new(pb::TrueStateResponse {
            voltage_magnitude: ts.voltage_magnitude.clone(),
            voltage_angle_deg: ts.voltage_angle.iter().map(|a| a.to_degrees()).collect(),
        }))
    }

    async fn run_estimate(
        &self,
        req: Request<pb::EstimateRequest>,
    ) -> Result<Response<pb::EstimateResponse>, Status> {
        let msg = req.into_inner();
        let (factorization, fact_name) = parse_proto_factorization(msg.factorization)?;
        let tolerance = msg.tolerance.unwrap_or(1e-4);
        let max_iterations = msg.max_iterations.unwrap_or(50);

        let formulation = SolverFormulation::NormalEquations { factorization };
        let form_name = format!("NormalEquations/{fact_name}");

        let zi_config = rusty_givens_core::kernel::ZeroInjectionConfig::default();
        let exec = execute_estimation(
            &self.state,
            formulation,
            &form_name,
            tolerance,
            max_iterations as usize,
            zi_config,
        )
        .map_err(|e| Status::internal(e))?;

        *self.state.last_result.lock().await = Some(exec.payload.clone());
        *self.state.last_se_result.lock().await = Some(exec.se_result);

        Ok(Response::new(payload_to_proto(&exec.payload)))
    }

    async fn get_last_result(
        &self,
        _req: Request<pb::Empty>,
    ) -> Result<Response<pb::EstimateResponse>, Status> {
        let guard = self.state.last_result.lock().await;
        match guard.as_ref() {
            Some(p) => Ok(Response::new(payload_to_proto(p))),
            None => Err(Status::not_found("no estimation result available")),
        }
    }
}

// ── BddService (Pro) ────────────────────────────────────────────────

#[cfg(feature = "pro")]
pub struct BddServiceImpl {
    pub state: Arc<AppState>,
}

#[cfg(feature = "pro")]
#[tonic::async_trait]
impl BddService for BddServiceImpl {
    async fn run_bdd(
        &self,
        req: Request<pb::RunBddRequest>,
    ) -> Result<Response<pb::RunBddResponse>, Status> {
        let msg = req.into_inner();
        let se_result = self.state.last_se_result.lock().await;
        let se_result = se_result
            .as_ref()
            .ok_or_else(|| Status::failed_precondition("run SE first"))?;
        let artifacts = se_result
            .artifacts
            .as_ref()
            .ok_or_else(|| Status::internal("no solver artifacts"))?;

        let proto_cfg = msg.config.unwrap_or_default();
        let mut bdd_config = BddConfig::default();
        if let Some(c) = proto_cfg.chi2_confidence {
            bdd_config.chi2_confidence = c;
        }
        if let Some(t) = proto_cfg.rn_max_threshold {
            bdd_config.residual_threshold = t;
        }
        let top_k = proto_cfg.top_k.unwrap_or(20) as usize;

        let chi2 = chi_squared_test(artifacts, &bdd_config)
            .ok()
            .map(|c| pb::ChiSquaredResult {
                bad_data_suspected: c.bad_data_suspected,
                objective: c.objective,
                threshold: c.threshold,
                degrees_of_freedom: c.degrees_of_freedom as u32,
            });

        let rn = rn_max_test(artifacts, &bdd_config, top_k).ok();
        let compute_time = rn.as_ref().map_or(0.0, |r| r.compute_time_s);
        let rn_proto = rn.map(|r| pb::RnMaxResult {
            bad_data_detected: r.bad_data_detected,
            compute_time_s: r.compute_time_s,
            top_residuals: r
                .top_residuals
                .iter()
                .map(|f| pb::FlaggedMeasurement {
                    index: f.index as u32,
                    normalized_residual: f.residual_value,
                    raw_residual: f.raw_residual,
                    omega_ii: f.omega_ii,
                    measurement_type: meas_ref_type(&f.measurement).to_string(),
                    measurement_label: meas_ref_label(&f.measurement).to_string(),
                })
                .collect(),
        });

        Ok(Response::new(pb::RunBddResponse {
            chi_squared: chi2,
            rn_max: rn_proto,
            compute_time_s: compute_time,
        }))
    }
}

// ── ObsService (Pro) ────────────────────────────────────────────────

#[cfg(feature = "pro")]
pub struct ObsServiceImpl {
    pub state: Arc<AppState>,
}

#[cfg(feature = "pro")]
#[tonic::async_trait]
impl ObsService for ObsServiceImpl {
    async fn run_observability(
        &self,
        req: Request<pb::RunObsRequest>,
    ) -> Result<Response<pb::RunObsResponse>, Status> {
        let msg = req.into_inner();
        let proto_cfg = msg.config.unwrap_or_default();

        let method = match pb::ObsMethod::try_from(proto_cfg.method) {
            Ok(pb::ObsMethod::ObsTopological) => ObsMethod::Topological,
            _ => ObsMethod::Numerical,
        };
        let config = ObsConfig {
            method,
            zero_pivot_threshold: proto_cfg.zero_pivot_threshold.unwrap_or(1e-5),
        };

        let result =
            analyze_observability(&self.state.system, &self.state.measurements, &config)
                .map_err(|e| Status::internal(e.to_string()))?;

        let method_proto = match result.method {
            ObsMethod::Numerical => pb::ObsMethod::ObsNumerical as i32,
            ObsMethod::Topological => pb::ObsMethod::ObsTopological as i32,
        };

        Ok(Response::new(pb::RunObsResponse {
            observable: result.observable,
            p_theta: Some(sub_model_to_proto(&result.p_theta)),
            q_v: Some(sub_model_to_proto(&result.q_v)),
            pq_pairs: Some(pb::PqPairCheck {
                all_paired: result.pq_pairs.all_paired,
                n_paired: result.pq_pairs.n_paired as u32,
                unpaired_wattmeters: result.pq_pairs.unpaired_wattmeters.clone(),
                unpaired_varmeters: result.pq_pairs.unpaired_varmeters.clone(),
                paired_bus_indices: result
                    .pq_pairs
                    .paired_bus_indices
                    .iter()
                    .map(|&i| i as u32)
                    .collect(),
            }),
            bus_observability: result
                .bus_observability
                .iter()
                .map(|b| pb::BusObsStatus {
                    bus_index: b.bus_index as u32,
                    p_theta_observable: b.p_theta_observable,
                    q_v_observable: b.q_v_observable,
                    p_theta_island_id: b.p_theta_island_id.map(|id| id as u32),
                    q_v_island_id: b.q_v_island_id.map(|id| id as u32),
                    needs_pseudo: b.needs_pseudo,
                    pseudo_models: b.pseudo_models.iter().map(|m| m.to_string()).collect(),
                    detected_in: b.detected_in.to_string(),
                    pseudo_basis: b
                        .pseudo_basis
                        .as_ref()
                        .map(|pb| pb.to_string())
                        .unwrap_or_default(),
                    paired_observable: b.paired_observable,
                })
                .collect(),
            branch_observability: result
                .branch_observability
                .iter()
                .map(|b| pb::BranchObsStatus {
                    branch_index: b.branch_index as u32,
                    p_theta_observable: b.p_theta_observable,
                    q_v_observable: b.q_v_observable,
                    detected_in: b.detected_in.to_string(),
                })
                .collect(),
            method: method_proto,
            analysis_time_s: result.analysis_time_s,
        }))
    }
}

#[cfg(feature = "pro")]
fn sub_model_to_proto(sm: &rusty_givens_obs::SubModelResult) -> pb::SubModelResult {
    pb::SubModelResult {
        kind: sm.kind.to_string(),
        observable: sm.observable,
        islands: sm
            .islands
            .iter()
            .map(|isl| pb::ObservableIsland {
                island_id: isl.island_id as u32,
                bus_indices: isl.buses.iter().map(|&b| b as u32).collect(),
                branch_indices: isl.branches.iter().map(|&b| b as u32).collect(),
            })
            .collect(),
        unobservable_buses: sm.unobservable_buses.iter().map(|&b| b as u32).collect(),
        unobservable_branches: sm.unobservable_branches.iter().map(|&b| b as u32).collect(),
        zero_pivot_states: sm.zero_pivot_states.iter().map(|&s| s as u32).collect(),
        n_measurements_used: sm.n_measurements_used as u32,
    }
}

// ── RedService (Pro) ────────────────────────────────────────────────

#[cfg(feature = "pro")]
pub struct RedServiceImpl {
    pub state: Arc<AppState>,
}

#[cfg(feature = "pro")]
#[tonic::async_trait]
impl RedService for RedServiceImpl {
    async fn run_redundancy(
        &self,
        req: Request<pb::RunRedRequest>,
    ) -> Result<Response<pb::RunRedResponse>, Status> {
        let msg = req.into_inner();
        let proto_cfg = msg.config.unwrap_or_default();

        let artifacts = {
            let guard = self.state.last_se_result.lock().await;
            let se_result = guard
                .as_ref()
                .ok_or_else(|| Status::failed_precondition("run SE first"))?;
            se_result
                .artifacts
                .clone()
                .ok_or_else(|| Status::internal("no solver artifacts"))?
        };

        let config = rusty_givens_red::RedundancyConfig {
            w_ii_critical_threshold: proto_cfg.w_ii_critical_threshold.unwrap_or(1e-6),
            detection_sensitivity_min: proto_cfg.detection_sensitivity_min.unwrap_or(0.3),
            lambda_k: proto_cfg
                .lambda_k
                .unwrap_or(std::f64::consts::FRAC_1_SQRT_2),
        };

        let result = rusty_givens_red::analyze_redundancy(
            &artifacts,
            &self.state.system,
            &self.state.measurements,
            &config,
        )
        .map_err(|e| Status::internal(e.to_string()))?;

        let payload = crate::pro::red_result_to_payload(&result, "estimate");

        Ok(Response::new(pb::RunRedResponse {
            global: Some(pb::GlobalRedundancy {
                n_buses: payload.global.n_buses as u32,
                n_state_variables: payload.global.n_state_variables as u32,
                n_measurements: payload.global.n_measurements as u32,
                degrees_of_freedom: payload.global.degrees_of_freedom,
                redundancy_ratio: payload.global.redundancy_ratio,
                n_voltmeters: payload.global.n_voltmeters as u32,
                n_ammeters: payload.global.n_ammeters as u32,
                n_wattmeters: payload.global.n_wattmeters as u32,
                n_varmeters: payload.global.n_varmeters as u32,
                n_pmu_pairs: payload.global.n_pmu_pairs as u32,
                sufficient: payload.global.sufficient,
            }),
            measurements: payload
                .measurements
                .iter()
                .map(|m| pb::MeasRedundancy {
                    index: m.index as u32,
                    measurement_type: m.measurement_type.clone(),
                    measurement_label: m.measurement_label.clone(),
                    sub_system: m.sub_system.clone(),
                    w_ii: m.w_ii,
                    max_abs_k_ik: m.max_abs_k_ik,
                    max_k_ik_partner: m.max_k_ik_partner.map(|p| p as u32),
                    redundancy_class: m.redundancy_class.clone(),
                    detection_reliable: m.detection_reliable,
                    coupling_indicator: m.coupling_indicator,
                    associated_buses: m.associated_buses.iter().map(|&b| b as u32).collect(),
                })
                .collect(),
            local: payload
                .local
                .iter()
                .map(|l| pb::LocalRedundancy {
                    bus_index: l.bus_index as u32,
                    n_connected_measurements: l.n_connected_measurements as u32,
                    min_w_ii: l.min_w_ii,
                    n_critical: l.n_critical as u32,
                    n_simply_redundant: l.n_simply_redundant as u32,
                    n_multiply_redundant: l.n_multiply_redundant as u32,
                })
                .collect(),
            n_critical: payload.n_critical as u32,
            n_simply_redundant: payload.n_simply_redundant as u32,
            n_multiply_redundant: payload.n_multiply_redundant as u32,
            analysis_time_s: payload.analysis_time_s,
        }))
    }
}

// ── Shared conversions ──────────────────────────────────────────────

fn parse_proto_factorization(val: i32) -> Result<(Factorization, &'static str), Status> {
    match pb::Factorization::try_from(val) {
        Ok(pb::Factorization::DenseCholesky) => {
            Ok((Factorization::DenseCholesky, "DenseCholesky"))
        }
        Ok(pb::Factorization::SparseCholesky) => {
            Ok((Factorization::SparseCholesky, "SparseCholesky"))
        }
        Ok(pb::Factorization::SparseLu) => Ok((Factorization::SparseLU, "SparseLU")),
        _ => Err(Status::invalid_argument(
            "factorization must be DENSE_CHOLESKY, SPARSE_CHOLESKY, or SPARSE_LU",
        )),
    }
}

fn factorization_to_proto(name: &str) -> i32 {
    match name {
        "DenseCholesky" => pb::Factorization::DenseCholesky as i32,
        "SparseCholesky" => pb::Factorization::SparseCholesky as i32,
        "SparseLU" => pb::Factorization::SparseLu as i32,
        _ => pb::Factorization::Unspecified as i32,
    }
}

fn payload_to_proto(p: &SeResultPayload) -> pb::EstimateResponse {
    pb::EstimateResponse {
        converged: p.converged,
        iterations: p.iterations as u32,
        se_time_seconds: p.se_time_seconds,
        final_increment: p.final_increment,
        factorization: factorization_to_proto(&p.factorization),
        tolerance: p.tolerance,
        max_iterations: p.max_iterations as u32,
        vm_mae: p.vm_mae,
        vm_max_error: p.vm_max_error,
        va_mae_deg: p.va_mae_deg,
        va_max_error_deg: p.va_max_error_deg,
        buses: p.buses.iter().map(bus_to_proto).collect(),
        #[cfg(feature = "pro")]
        bdd: p.bdd.as_ref().map(bdd_to_proto),
        #[cfg(not(feature = "pro"))]
        bdd: None,
        obs_check: None,
    }
}

fn bus_to_proto(b: &BusResult) -> pb::BusResult {
    pb::BusResult {
        index: b.index as u32,
        label: b.label as u32,
        est_vm: b.est_vm,
        est_va_deg: b.est_va_deg,
        true_vm: b.true_vm,
        true_va_deg: b.true_va_deg,
        vm_error: b.vm_error,
        va_error_deg: b.va_error_deg,
    }
}

#[cfg(feature = "pro")]
fn bdd_to_proto(b: &BddPayload) -> pb::BddResult {
    pb::BddResult {
        chi_squared: b.chi_squared.as_ref().map(|c| pb::ChiSquaredResult {
            bad_data_suspected: c.bad_data_suspected,
            objective: c.objective,
            threshold: c.threshold,
            degrees_of_freedom: c.degrees_of_freedom as u32,
        }),
        rn_max: b.residual_test.as_ref().map(|r| pb::RnMaxResult {
            bad_data_detected: r.bad_data_detected,
            compute_time_s: r.compute_time_s,
            top_residuals: r
                .top_residuals
                .iter()
                .map(|f| pb::FlaggedMeasurement {
                    index: f.index as u32,
                    normalized_residual: f.residual_value,
                    raw_residual: f.raw_residual,
                    omega_ii: f.omega_ii,
                    measurement_type: f.measurement_type.clone(),
                    measurement_label: f.measurement_label.clone(),
                })
                .collect(),
        }),
    }
}
