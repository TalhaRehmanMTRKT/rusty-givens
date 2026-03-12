//! Zero-injection bus handling for State Estimation.
//!
//! Buses that have neither a load nor a generator connected must satisfy
//! the constraint P_inj = 0, Q_inj = 0.  This module provides:
//!
//!   1. **Identification** of zero-injection buses from the network model
//!      and measurement set.
//!   2. **Virtual measurement injection** — adds P = 0 MW and Q = 0 Mvar
//!      wattmeter/varmeter pairs with a very small standard deviation so
//!      that all solver formulations enforce the zero-injection constraint.
//!   3. **Post-estimation violation checking** — compares estimated bus
//!      injections against configurable thresholds and produces a report.

use crate::model::measurement::{VarmeterLocation, WattmeterLocation};
use crate::model::network::{BusType, PowerSystem};
use crate::model::MeasurementSet;

use super::post_estimation::BusInjectionResult;

// ── Configuration ───────────────────────────────────────────────────

/// How the solver should handle zero-injection buses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZeroInjectionMethod {
    /// Inject virtual P = 0 / Q = 0 measurements with a very small variance.
    /// Works with every solver formulation.
    VirtualMeasurements,
    /// Use explicit equality constraints c(x) = 0 via the Lagrangian KKT
    /// system.  Only meaningful for the `EqualityConstrained` formulation;
    /// other formulations fall back to `VirtualMeasurements`.
    EqualityConstraints,
}

/// Configuration for zero-injection bus treatment.
#[derive(Debug, Clone)]
pub struct ZeroInjectionConfig {
    /// Whether zero-injection handling is enabled.
    pub enabled: bool,
    /// Preferred enforcement method.
    pub method: ZeroInjectionMethod,
    /// Standard deviation (sigma) for virtual zero-injection measurements.
    /// A very small value effectively forces P = Q = 0.
    /// Typical value: 1e-6 p.u. (variance = 1e-12).
    pub sigma: f64,
    /// Absolute threshold (p.u.) above which an estimated injection at a
    /// zero-injection bus is flagged as a violation.
    pub violation_threshold_pu: f64,
    /// Explicit list of bus **labels** to treat as zero-injection, bypassing
    /// automatic detection.  When `Some`, only these buses are considered ZI
    /// (the automatic heuristic is skipped entirely).  When `None`, the
    /// automatic detection based on network topology and measurements is used.
    pub explicit_buses: Option<Vec<usize>>,
}

impl Default for ZeroInjectionConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            method: ZeroInjectionMethod::VirtualMeasurements,
            sigma: 1e-6,
            violation_threshold_pu: 1e-3,
            explicit_buses: None,
        }
    }
}

// ── Identification ──────────────────────────────────────────────────

/// A bus identified as having zero net injection.
#[derive(Debug, Clone)]
pub struct ZeroInjectionBus {
    /// Sequential bus index (0-based).
    pub bus_index: usize,
    /// Original bus label.
    pub bus_label: usize,
}

/// Resolve the set of zero-injection buses according to the configuration:
///
/// - If `config.explicit_buses` is `Some`, uses only those bus labels
///   (skipping automatic detection entirely).  Labels that don't exist
///   in the system are silently ignored.
/// - Otherwise, falls back to [`find_zero_injection_buses`] for automatic
///   detection based on network topology and measurements.
pub fn resolve_zi_buses(
    system: &PowerSystem,
    measurements: &MeasurementSet,
    config: &ZeroInjectionConfig,
) -> Vec<ZeroInjectionBus> {
    if let Some(labels) = &config.explicit_buses {
        labels
            .iter()
            .filter_map(|&label| {
                system.bus_index.get(&label).map(|&idx| ZeroInjectionBus {
                    bus_index: idx,
                    bus_label: label,
                })
            })
            .collect()
    } else {
        find_zero_injection_buses(system, measurements)
    }
}

/// Identify buses with zero injection: no load, no generation, and no
/// existing bus-level P/Q injection measurement with a nonzero value.
///
/// Slack and PV buses are excluded (they have generation by definition).
///
/// For explicit control, prefer [`resolve_zi_buses`] which respects the
/// `explicit_buses` override in [`ZeroInjectionConfig`].
pub fn find_zero_injection_buses(
    system: &PowerSystem,
    measurements: &MeasurementSet,
) -> Vec<ZeroInjectionBus> {
    let n = system.n_buses();
    let mut has_injection = vec![false; n];

    for bus in &system.buses {
        let idx = system.bus_idx(bus.label);
        if bus.bus_type == BusType::Slack || bus.bus_type == BusType::PV {
            has_injection[idx] = true;
        }
        if bus.active_demand.abs() > 1e-12 || bus.reactive_demand.abs() > 1e-12 {
            has_injection[idx] = true;
        }
    }

    for wm in measurements.wattmeters.iter().filter(|m| m.status) {
        if let WattmeterLocation::Bus(bus) = &wm.location {
            if wm.active.abs() > 1e-12 {
                has_injection[system.bus_idx(*bus)] = true;
            }
        }
    }

    for vm in measurements.varmeters.iter().filter(|m| m.status) {
        if let VarmeterLocation::Bus(bus) = &vm.location {
            if vm.reactive.abs() > 1e-12 {
                has_injection[system.bus_idx(*bus)] = true;
            }
        }
    }

    (0..n)
        .filter(|&i| !has_injection[i])
        .map(|i| ZeroInjectionBus {
            bus_index: i,
            bus_label: system.buses[i].label,
        })
        .collect()
}

// ── Virtual measurement injection ───────────────────────────────────

/// The label prefix used for virtual zero-injection measurements so they
/// can be identified in measurement maps and reports.
pub const ZI_LABEL_PREFIX: &str = "ZI";

/// Inject virtual P = 0 and Q = 0 measurements for every zero-injection
/// bus into the measurement set.  Returns the number of measurement pairs
/// added (one wattmeter + one varmeter per ZI bus).
///
/// The `measurements` set is modified in place.  The added measurements
/// carry labels of the form `ZI_P_bus{label}` / `ZI_Q_bus{label}` so
/// that downstream analysis (BDD, redundancy) can recognise them.
pub fn inject_virtual_zi_measurements(
    zi_buses: &[ZeroInjectionBus],
    measurements: &mut MeasurementSet,
    sigma: f64,
) -> usize {
    let variance = sigma * sigma;

    for zi in zi_buses {
        let p_label = format!("{}_P_bus{}", ZI_LABEL_PREFIX, zi.bus_label);
        let q_label = format!("{}_Q_bus{}", ZI_LABEL_PREFIX, zi.bus_label);

        measurements.add_bus_wattmeter(p_label, zi.bus_label, 0.0, variance);
        measurements.add_bus_varmeter(q_label, zi.bus_label, 0.0, variance);
    }

    zi_buses.len()
}

// ── Post-estimation violation report ────────────────────────────────

/// A zero-injection bus whose estimated injection exceeds the threshold.
#[derive(Debug, Clone)]
pub struct ZiViolation {
    pub bus_index: usize,
    pub bus_label: usize,
    pub p_estimated_pu: f64,
    pub q_estimated_pu: f64,
    pub p_exceeds: bool,
    pub q_exceeds: bool,
}

/// Complete zero-injection report produced after estimation.
#[derive(Debug, Clone)]
pub struct ZeroInjectionReport {
    /// Total number of zero-injection buses identified.
    pub n_zi_buses: usize,
    /// The violation threshold that was applied (p.u.).
    pub threshold_pu: f64,
    /// Buses where the estimated injection exceeds the threshold.
    pub violations: Vec<ZiViolation>,
    /// True if there are no violations.
    pub all_clean: bool,
    /// Per-bus estimated injections for ALL zero-injection buses
    /// (including those within tolerance), for diagnostic purposes.
    pub zi_bus_injections: Vec<ZiBusInjection>,
}

/// Estimated injection at a single zero-injection bus (diagnostic data).
#[derive(Debug, Clone)]
pub struct ZiBusInjection {
    pub bus_index: usize,
    pub bus_label: usize,
    pub p_estimated_pu: f64,
    pub q_estimated_pu: f64,
}

/// Check estimated bus injections at zero-injection buses against the
/// configured threshold and produce a report.
pub fn check_zi_violations(
    zi_buses: &[ZeroInjectionBus],
    bus_injections: &[BusInjectionResult],
    threshold_pu: f64,
) -> ZeroInjectionReport {
    let mut violations = Vec::new();
    let mut zi_bus_injections = Vec::with_capacity(zi_buses.len());

    for zi in zi_buses {
        let inj = &bus_injections[zi.bus_index];
        let p_abs = inj.p_inj.abs();
        let q_abs = inj.q_inj.abs();

        zi_bus_injections.push(ZiBusInjection {
            bus_index: zi.bus_index,
            bus_label: zi.bus_label,
            p_estimated_pu: inj.p_inj,
            q_estimated_pu: inj.q_inj,
        });

        let p_exceeds = p_abs > threshold_pu;
        let q_exceeds = q_abs > threshold_pu;

        if p_exceeds || q_exceeds {
            violations.push(ZiViolation {
                bus_index: zi.bus_index,
                bus_label: zi.bus_label,
                p_estimated_pu: inj.p_inj,
                q_estimated_pu: inj.q_inj,
                p_exceeds,
                q_exceeds,
            });
        }
    }

    let all_clean = violations.is_empty();
    ZeroInjectionReport {
        n_zi_buses: zi_buses.len(),
        threshold_pu,
        violations,
        all_clean,
        zi_bus_injections,
    }
}
