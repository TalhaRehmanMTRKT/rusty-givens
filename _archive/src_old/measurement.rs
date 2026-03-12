use serde::{Deserialize, Serialize};

/// Which end of the branch a measurement is placed at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BranchEnd {
    From,
    To,
}

/// Coordinate system for PMU measurements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PmuCoordinate {
    Polar,
    Rectangular,
}

// ────────────────────────────────────────────────────────────────────
//  Individual measurement device types
// ────────────────────────────────────────────────────────────────────

/// Voltmeter: measures bus voltage magnitude V_i.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Voltmeter {
    pub label: String,
    pub bus: usize,
    pub magnitude: f64,
    pub variance: f64,
    pub status: bool,
}

/// Ammeter: measures branch current magnitude I_{ij} or I_{ji}.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ammeter {
    pub label: String,
    pub branch: usize,
    pub end: BranchEnd,
    pub magnitude: f64,
    pub variance: f64,
    /// If true, the measurement function uses the squared form
    /// (improves numerical robustness for small currents).
    pub square: bool,
    pub status: bool,
}

/// Wattmeter: measures active power injection P_i or flow P_{ij}/P_{ji}.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wattmeter {
    pub label: String,
    pub location: WattmeterLocation,
    pub active: f64,
    pub variance: f64,
    pub status: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum WattmeterLocation {
    Bus(usize),
    Branch { branch: usize, end: BranchEnd },
}

/// Varmeter: measures reactive power injection Q_i or flow Q_{ij}/Q_{ji}.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Varmeter {
    pub label: String,
    pub location: VarmeterLocation,
    pub reactive: f64,
    pub variance: f64,
    pub status: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum VarmeterLocation {
    Bus(usize),
    Branch { branch: usize, end: BranchEnd },
}

/// PMU: measures a phasor (magnitude + angle) at a bus or branch end.
///
/// When in rectangular coordinates, the magnitude/angle are converted
/// to real/imaginary components with propagated variances.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pmu {
    pub label: String,
    pub location: PmuLocation,
    pub magnitude: f64,
    pub angle: f64,
    pub variance_magnitude: f64,
    pub variance_angle: f64,
    pub coordinate: PmuCoordinate,
    /// Whether to include covariance between real/imag parts
    /// (only meaningful in rectangular coordinates).
    pub correlated: bool,
    /// If true and this is a branch current PMU, use squared magnitude form.
    pub square: bool,
    pub status: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PmuLocation {
    Bus(usize),
    Branch { branch: usize, end: BranchEnd },
}

// ────────────────────────────────────────────────────────────────────
//  Unified measurement set M
// ────────────────────────────────────────────────────────────────────

/// Container for all measurement devices, analogous to JuliaGrid's `Measurement` type.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MeasurementSet {
    pub voltmeters: Vec<Voltmeter>,
    pub ammeters: Vec<Ammeter>,
    pub wattmeters: Vec<Wattmeter>,
    pub varmeters: Vec<Varmeter>,
    pub pmus: Vec<Pmu>,
}

impl MeasurementSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_voltmeter(&mut self, v: Voltmeter) -> &mut Self {
        self.voltmeters.push(v);
        self
    }

    pub fn add_ammeter(&mut self, a: Ammeter) -> &mut Self {
        self.ammeters.push(a);
        self
    }

    pub fn add_wattmeter(&mut self, w: Wattmeter) -> &mut Self {
        self.wattmeters.push(w);
        self
    }

    pub fn add_varmeter(&mut self, v: Varmeter) -> &mut Self {
        self.varmeters.push(v);
        self
    }

    pub fn add_pmu(&mut self, p: Pmu) -> &mut Self {
        self.pmus.push(p);
        self
    }

    /// Total number of scalar measurement equations k.
    ///
    /// Legacy measurements contribute 1 equation each.
    /// PMUs contribute 2 equations each (magnitude+angle or real+imag).
    pub fn n_equations(&self) -> usize {
        let n_legacy = self.voltmeters.iter().filter(|m| m.status).count()
            + self.ammeters.iter().filter(|m| m.status).count()
            + self.wattmeters.iter().filter(|m| m.status).count()
            + self.varmeters.iter().filter(|m| m.status).count();
        let n_pmu = self.pmus.iter().filter(|m| m.status).count();
        n_legacy + 2 * n_pmu
    }
}

// ────────────────────────────────────────────────────────────────────
//  Builder helpers (mirrors JuliaGrid's addXxx! functions)
// ────────────────────────────────────────────────────────────────────

impl MeasurementSet {
    /// Convenience: add a bus voltmeter.
    pub fn add_bus_voltmeter(
        &mut self,
        label: impl Into<String>,
        bus: usize,
        magnitude: f64,
        variance: f64,
    ) -> &mut Self {
        self.voltmeters.push(Voltmeter {
            label: label.into(),
            bus,
            magnitude,
            variance,
            status: true,
        });
        self
    }

    /// Convenience: add a branch ammeter.
    pub fn add_branch_ammeter(
        &mut self,
        label: impl Into<String>,
        branch: usize,
        end: BranchEnd,
        magnitude: f64,
        variance: f64,
        square: bool,
    ) -> &mut Self {
        self.ammeters.push(Ammeter {
            label: label.into(),
            branch,
            end,
            magnitude,
            variance,
            square,
            status: true,
        });
        self
    }

    /// Convenience: add a bus wattmeter (power injection).
    pub fn add_bus_wattmeter(
        &mut self,
        label: impl Into<String>,
        bus: usize,
        active: f64,
        variance: f64,
    ) -> &mut Self {
        self.wattmeters.push(Wattmeter {
            label: label.into(),
            location: WattmeterLocation::Bus(bus),
            active,
            variance,
            status: true,
        });
        self
    }

    /// Convenience: add a branch wattmeter (power flow).
    pub fn add_branch_wattmeter(
        &mut self,
        label: impl Into<String>,
        branch: usize,
        end: BranchEnd,
        active: f64,
        variance: f64,
    ) -> &mut Self {
        self.wattmeters.push(Wattmeter {
            label: label.into(),
            location: WattmeterLocation::Branch { branch, end },
            active,
            variance,
            status: true,
        });
        self
    }

    /// Convenience: add a bus varmeter (reactive power injection).
    pub fn add_bus_varmeter(
        &mut self,
        label: impl Into<String>,
        bus: usize,
        reactive: f64,
        variance: f64,
    ) -> &mut Self {
        self.varmeters.push(Varmeter {
            label: label.into(),
            location: VarmeterLocation::Bus(bus),
            reactive,
            variance,
            status: true,
        });
        self
    }

    /// Convenience: add a branch varmeter (reactive power flow).
    pub fn add_branch_varmeter(
        &mut self,
        label: impl Into<String>,
        branch: usize,
        end: BranchEnd,
        reactive: f64,
        variance: f64,
    ) -> &mut Self {
        self.varmeters.push(Varmeter {
            label: label.into(),
            location: VarmeterLocation::Branch { branch, end },
            reactive,
            variance,
            status: true,
        });
        self
    }

    /// Convenience: add a bus PMU.
    pub fn add_bus_pmu(
        &mut self,
        label: impl Into<String>,
        bus: usize,
        magnitude: f64,
        angle: f64,
        variance_magnitude: f64,
        variance_angle: f64,
        polar: bool,
        correlated: bool,
    ) -> &mut Self {
        self.pmus.push(Pmu {
            label: label.into(),
            location: PmuLocation::Bus(bus),
            magnitude,
            angle,
            variance_magnitude,
            variance_angle,
            coordinate: if polar {
                PmuCoordinate::Polar
            } else {
                PmuCoordinate::Rectangular
            },
            correlated,
            square: false,
            status: true,
        });
        self
    }

    /// Convenience: add a branch PMU (current phasor).
    pub fn add_branch_pmu(
        &mut self,
        label: impl Into<String>,
        branch: usize,
        end: BranchEnd,
        magnitude: f64,
        angle: f64,
        variance_magnitude: f64,
        variance_angle: f64,
        polar: bool,
        correlated: bool,
    ) -> &mut Self {
        self.pmus.push(Pmu {
            label: label.into(),
            location: PmuLocation::Branch { branch, end },
            magnitude,
            angle,
            variance_magnitude,
            variance_angle,
            coordinate: if polar {
                PmuCoordinate::Polar
            } else {
                PmuCoordinate::Rectangular
            },
            correlated,
            square: false,
            status: true,
        });
        self
    }
}
