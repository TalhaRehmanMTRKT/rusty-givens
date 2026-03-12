export interface BusNode {
  index: number;
  label: number;
  vn_kv: number;
  bus_type: number; // 1=PQ, 2=PV, 3=Slack
  geo_x?: number;
  geo_y?: number;
}

export interface BranchEdge {
  index: number;
  from_bus: number;
  to_bus: number;
}

export interface NetworkData {
  n_buses: number;
  n_branches: number;
  slack_bus_index: number;
  base_mva: number;
  buses: BusNode[];
  branches: BranchEdge[];
}

export interface BusResult {
  index: number;
  label: number;
  est_vm: number;
  est_va_deg: number;
  true_vm: number;
  true_va_deg: number;
  vm_error: number;
  va_error_deg: number;
}

// ── Post-estimation dependent results ──────────────────────────────

export interface TerminalFlow {
  p: number;
  q: number;
  i_mag: number;
}

export interface BranchFlow {
  branch_index: number;
  from_bus: number;
  to_bus: number;
  from: TerminalFlow;
  to: TerminalFlow;
  p_loss: number;
  q_loss: number;
}

export interface BusInjection {
  bus_index: number;
  p_inj: number;
  q_inj: number;
}

export interface MeasCounts {
  voltmeters: number;
  ammeters: number;
  wattmeters: number;
  varmeters: number;
  pmu_pairs: number;
  current_angle_meters: number;
  total: number;
}

export interface ObjectiveSummary {
  objective_value: number;
  expected_value: number;
  degrees_of_freedom: number;
}

export interface VoltageLevelStats {
  voltage_kv: number;
  n_buses: number;
  n_branches: number;
  measurement_counts: MeasCounts;
}

export interface PowerBalance {
  total_p_loss: number;
  total_q_loss: number;
  total_p_generation: number;
  total_q_generation: number;
  total_p_load: number;
  total_q_load: number;
}

export interface GlobalStatus {
  timestamp: string;
  n_buses: number;
  n_branches: number;
  n_state_variables: number;
  objective?: ObjectiveSummary;
  measurement_counts: MeasCounts;
  per_voltage_level: VoltageLevelStats[];
  branch_flows: BranchFlow[];
  bus_injections: BusInjection[];
  power_balance: PowerBalance;
}

export interface MeasurementExport {
  index: number;
  measurement_type: string;
  label: string;
  measured_value: number;
  standard_deviation: number;
  status: boolean;
  estimated_value?: number;
  residual?: number;
}

// ── SE Result ──────────────────────────────────────────────────────

export interface SeResult {
  converged: boolean;
  iterations: number;
  se_time_seconds: number;
  final_increment: number;
  factorization: string;
  tolerance: number;
  max_iterations: number;
  vm_mae: number;
  vm_max_error: number;
  va_mae_deg: number;
  va_max_error_deg: number;
  buses: BusResult[];
  global_status?: GlobalStatus;
}

export interface EstimateRequest {
  factorization: string;
  formulation?: string;
  max_iterations?: number;
  tolerance?: number;
  skip_obs_check?: boolean;
}

export type Factorization = 'DenseCholesky' | 'SparseCholesky' | 'SparseLU';

export type SolverFormulation =
  | 'NormalEquations'
  | 'OrthogonalQR'
  | 'PetersWilkinson'
  | 'EqualityConstrained'
  | 'FastDecoupled'
  | 'DcEstimation';

// ── Redundancy Analysis ─────────────────────────────────────────────

export interface GlobalRedundancy {
  n_buses: number;
  n_state_variables: number;
  n_measurements: number;
  degrees_of_freedom: number;
  redundancy_ratio: number;
  n_voltmeters: number;
  n_ammeters: number;
  n_wattmeters: number;
  n_varmeters: number;
  n_pmu_pairs: number;
  sufficient: boolean;
}

export interface MeasRedundancy {
  index: number;
  measurement_type: string;
  measurement_label: string;
  sub_system: string;
  w_ii: number;
  max_abs_k_ik: number;
  max_k_ik_partner?: number;
  redundancy_class: string;
  detection_reliable: boolean;
  coupling_indicator: number;
  associated_buses: number[];
}

export interface LocalRedundancy {
  bus_index: number;
  n_connected_measurements: number;
  min_w_ii: number;
  n_critical: number;
  n_simply_redundant: number;
  n_multiply_redundant: number;
}

export interface RedundancyResult {
  global: GlobalRedundancy;
  measurements: MeasRedundancy[];
  local: LocalRedundancy[];
  n_critical: number;
  n_simply_redundant: number;
  n_multiply_redundant: number;
  analysis_time_s: number;
}

export type DisplayLevel = 'critical' | 'weak' | 'single' | 'multiple';

export function getDisplayLevel(m: MeasRedundancy): DisplayLevel {
  if (m.redundancy_class === 'M0_Critical') return 'critical';
  if (!m.detection_reliable) return 'weak';
  if (m.redundancy_class === 'M1_SimplyRedundant') return 'single';
  return 'multiple';
}

export interface RedundancyRequest {
  w_ii_critical_threshold?: number;
  detection_sensitivity_min?: number;
  lambda_k?: number;
}
