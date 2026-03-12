#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BranchEnd {
    From,
    To,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PmuCoordinate {
    Polar,
    Rectangular,
}

#[derive(Debug, Clone)]
pub struct Voltmeter {
    pub label: String,
    pub bus: usize,
    pub magnitude: f64,
    pub variance: f64,
    pub status: bool,
}

#[derive(Debug, Clone)]
pub struct Ammeter {
    pub label: String,
    pub branch: usize,
    pub end: BranchEnd,
    pub magnitude: f64,
    pub variance: f64,
    pub square: bool,
    pub status: bool,
}

#[derive(Debug, Clone)]
pub struct Wattmeter {
    pub label: String,
    pub location: WattmeterLocation,
    pub active: f64,
    pub variance: f64,
    pub status: bool,
}

#[derive(Debug, Clone)]
pub enum WattmeterLocation {
    Bus(usize),
    Branch { branch: usize, end: BranchEnd },
}

#[derive(Debug, Clone)]
pub struct Varmeter {
    pub label: String,
    pub location: VarmeterLocation,
    pub reactive: f64,
    pub variance: f64,
    pub status: bool,
}

#[derive(Debug, Clone)]
pub enum VarmeterLocation {
    Bus(usize),
    Branch { branch: usize, end: BranchEnd },
}

#[derive(Debug, Clone)]
pub struct Pmu {
    pub label: String,
    pub location: PmuLocation,
    pub magnitude: f64,
    pub angle: f64,
    pub variance_magnitude: f64,
    pub variance_angle: f64,
    pub coordinate: PmuCoordinate,
    pub correlated: bool,
    pub square: bool,
    pub status: bool,
}

#[derive(Debug, Clone)]
pub enum PmuLocation {
    Bus(usize),
    Branch { branch: usize, end: BranchEnd },
}

/// Standalone current angle measurement on a branch (from a dedicated sensor
/// or a current-only PMU channel).  Provides the angle of the branch current
/// phasor, which constrains the voltage-angle difference across the branch
/// in the P-θ sub-model.
#[derive(Debug, Clone)]
pub struct CurrentAngleMeter {
    pub label: String,
    pub branch: usize,
    pub end: BranchEnd,
    pub angle: f64,
    pub variance: f64,
    pub status: bool,
}

#[derive(Debug, Clone, Default)]
pub struct MeasurementSet {
    pub voltmeters: Vec<Voltmeter>,
    pub ammeters: Vec<Ammeter>,
    pub wattmeters: Vec<Wattmeter>,
    pub varmeters: Vec<Varmeter>,
    pub pmus: Vec<Pmu>,
    pub current_angle_meters: Vec<CurrentAngleMeter>,
}

impl MeasurementSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn n_equations(&self) -> usize {
        let n_legacy = self.voltmeters.iter().filter(|m| m.status).count()
            + self.ammeters.iter().filter(|m| m.status).count()
            + self.wattmeters.iter().filter(|m| m.status).count()
            + self.varmeters.iter().filter(|m| m.status).count()
            + self.current_angle_meters.iter().filter(|m| m.status).count();
        let n_pmu = self.pmus.iter().filter(|m| m.status).count();
        n_legacy + 2 * n_pmu
    }

    pub fn add_bus_voltmeter(
        &mut self, label: impl Into<String>, bus: usize, magnitude: f64, variance: f64,
    ) -> &mut Self {
        self.voltmeters.push(Voltmeter { label: label.into(), bus, magnitude, variance, status: true });
        self
    }

    pub fn add_branch_ammeter(
        &mut self, label: impl Into<String>, branch: usize, end: BranchEnd,
        magnitude: f64, variance: f64, square: bool,
    ) -> &mut Self {
        self.ammeters.push(Ammeter { label: label.into(), branch, end, magnitude, variance, square, status: true });
        self
    }

    pub fn add_bus_wattmeter(
        &mut self, label: impl Into<String>, bus: usize, active: f64, variance: f64,
    ) -> &mut Self {
        self.wattmeters.push(Wattmeter {
            label: label.into(), location: WattmeterLocation::Bus(bus), active, variance, status: true,
        });
        self
    }

    pub fn add_branch_wattmeter(
        &mut self, label: impl Into<String>, branch: usize, end: BranchEnd,
        active: f64, variance: f64,
    ) -> &mut Self {
        self.wattmeters.push(Wattmeter {
            label: label.into(), location: WattmeterLocation::Branch { branch, end }, active, variance, status: true,
        });
        self
    }

    pub fn add_bus_varmeter(
        &mut self, label: impl Into<String>, bus: usize, reactive: f64, variance: f64,
    ) -> &mut Self {
        self.varmeters.push(Varmeter {
            label: label.into(), location: VarmeterLocation::Bus(bus), reactive, variance, status: true,
        });
        self
    }

    pub fn add_branch_varmeter(
        &mut self, label: impl Into<String>, branch: usize, end: BranchEnd,
        reactive: f64, variance: f64,
    ) -> &mut Self {
        self.varmeters.push(Varmeter {
            label: label.into(), location: VarmeterLocation::Branch { branch, end }, reactive, variance, status: true,
        });
        self
    }

    pub fn add_bus_pmu(
        &mut self, label: impl Into<String>, bus: usize, magnitude: f64, angle: f64,
        variance_magnitude: f64, variance_angle: f64, polar: bool, correlated: bool,
    ) -> &mut Self {
        self.pmus.push(Pmu {
            label: label.into(), location: PmuLocation::Bus(bus), magnitude, angle,
            variance_magnitude, variance_angle,
            coordinate: if polar { PmuCoordinate::Polar } else { PmuCoordinate::Rectangular },
            correlated, square: false, status: true,
        });
        self
    }

    pub fn add_branch_pmu(
        &mut self, label: impl Into<String>, branch: usize, end: BranchEnd,
        magnitude: f64, angle: f64, variance_magnitude: f64, variance_angle: f64,
        polar: bool, correlated: bool,
    ) -> &mut Self {
        self.pmus.push(Pmu {
            label: label.into(), location: PmuLocation::Branch { branch, end }, magnitude, angle,
            variance_magnitude, variance_angle,
            coordinate: if polar { PmuCoordinate::Polar } else { PmuCoordinate::Rectangular },
            correlated, square: false, status: true,
        });
        self
    }

    pub fn add_branch_current_angle(
        &mut self, label: impl Into<String>, branch: usize, end: BranchEnd,
        angle: f64, variance: f64,
    ) -> &mut Self {
        self.current_angle_meters.push(CurrentAngleMeter {
            label: label.into(), branch, end, angle, variance, status: true,
        });
        self
    }
}
