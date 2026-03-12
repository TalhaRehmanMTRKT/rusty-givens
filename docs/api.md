# REST API Reference

The estimate service exposes a REST API on **port 3001**. All endpoints are prefixed with `/api`.

---

## Free Edition Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/api/network` | Network topology (buses and branches) |
| `GET` | `/api/true-state` | Reference power-flow state for error comparison |
| `POST` | `/api/estimate` | Run WLS state estimation |
| `GET` | `/api/last-result` | Most recent estimation result |
| `GET` | `/api/measurements` | Measurement export with estimated values and residuals |

---

## GET /api/network

Returns the network topology for visualization.

### Response

```json
{
  "n_buses": 2224,
  "n_branches": 3207,
  "slack_bus_index": 0,
  "base_mva": 100.0,
  "buses": [
    {
      "index": 0,
      "label": 1,
      "vn_kv": 400.0,
      "bus_type": 3,
      "geo_x": -1.234,
      "geo_y": 51.567
    }
  ],
  "branches": [
    { "index": 0, "from_bus": 1, "to_bus": 2 }
  ]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `bus_type` | `u8` | 1 = PQ, 2 = PV, 3 = Slack |
| `geo_x`, `geo_y` | `f64?` | Geographic coordinates (longitude, latitude). Absent for buses without location data. |

---

## GET /api/true-state

Returns the reference (power-flow) state used for accuracy comparison.

### Response

```json
{
  "voltage_magnitude": [1.0, 0.998, ...],
  "voltage_angle_deg": [0.0, -1.23, ...]
}
```

---

## POST /api/estimate

Runs WLS state estimation with the given configuration.

### Request Body

```json
{
  "formulation": "NormalEquations",
  "factorization": "SparseCholesky",
  "max_iterations": 50,
  "tolerance": 1e-4,
  "skip_obs_check": false,
  "obs_method": "Numerical",
  "zi_enabled": true,
  "zi_sigma": 1e-6,
  "zi_violation_threshold": 1e-3,
  "zi_buses": null
}
```

### Request Parameters

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `formulation` | `string?` | `"NormalEquations"` | Solver formulation: `NormalEquations` / `NE`, `OrthogonalQR` / `QR` / `Givens`, `PetersWilkinson` / `PW`, `EqualityConstrained` / `EC`, `FastDecoupled` / `FD`, `DcEstimation` / `DC` |
| `factorization` | `string` | `"SparseCholesky"` | Linear algebra backend: `SparseCholesky`, `SparseLU`, `DenseCholesky`. Used by NormalEquations and EqualityConstrained. |
| `max_iterations` | `number?` | `50` | Maximum Gauss-Newton iterations |
| `tolerance` | `number?` | `1e-4` | Convergence tolerance \( \max|\Delta x| \) |
| `skip_obs_check` | `bool?` | `false` | *(Pro only)* Skip the observability check before estimation |
| `obs_method` | `string?` | `"Numerical"` | *(Pro only)* Observability method: `Numerical` or `Topological` |
| `zi_enabled` | `bool?` | `true` | Enable zero-injection bus handling |
| `zi_sigma` | `number?` | `1e-6` | Standard deviation for virtual zero-injection measurements |
| `zi_violation_threshold` | `number?` | `1e-3` | Threshold (p.u.) for zero-injection violation reporting |
| `zi_buses` | `[number]?` | `null` | Explicit list of bus labels to treat as zero-injection. When set, bypasses automatic detection. |

### Response

```json
{
  "converged": true,
  "iterations": 4,
  "se_time_seconds": 0.087,
  "final_increment": 2.3e-6,
  "factorization": "NormalEquations/SparseCholesky",
  "tolerance": 1e-4,
  "max_iterations": 50,
  "vm_mae": 0.00042,
  "vm_max_error": 0.0031,
  "va_mae_deg": 0.012,
  "va_max_error_deg": 0.094,
  "buses": [
    {
      "index": 0,
      "label": 1,
      "est_vm": 1.00058,
      "est_va_deg": 0.0,
      "true_vm": 1.0,
      "true_va_deg": 0.0,
      "vm_error": 0.00058,
      "va_error_deg": 0.0
    }
  ],
  "bdd": null,
  "obs_check": null,
  "global_status": { ... },
  "zero_injection": { ... }
}
```

### Response Fields

| Field | Type | Description |
|-------|------|-------------|
| `converged` | `bool` | Whether the solver converged within the iteration limit |
| `iterations` | `number` | Number of Gauss-Newton iterations performed |
| `se_time_seconds` | `number` | Wall-clock solve time (seconds) |
| `final_increment` | `number` | \( \max|\Delta x| \) at the last iteration |
| `factorization` | `string` | Formulation/factorization used (e.g. `"NormalEquations/SparseCholesky"`) |
| `vm_mae` | `number` | Mean absolute error — voltage magnitude (p.u.) |
| `vm_max_error` | `number` | Maximum absolute error — voltage magnitude (p.u.) |
| `va_mae_deg` | `number` | Mean absolute error — voltage angle (degrees) |
| `va_max_error_deg` | `number` | Maximum absolute error — voltage angle (degrees) |
| `buses` | `[BusResult]` | Per-bus estimated vs. true state |
| `bdd` | `BddPayload?` | *(Pro only)* Inline Chi² and residual test results |
| `obs_check` | `ObsResultPayload?` | *(Pro only)* Observability check performed before estimation |
| `global_status` | `GlobalStatusPayload?` | Power flow results, measurement counts, voltage-level statistics (only when converged) |
| `zero_injection` | `ZeroInjectionReportPayload?` | Zero-injection violation report (only when ZI buses exist) |

### Global Status Object

When the estimation converges, `global_status` contains:

```json
{
  "timestamp": "2026-03-12T14:30:00Z",
  "n_buses": 2224,
  "n_branches": 3207,
  "n_state_variables": 4447,
  "objective": {
    "objective_value": 4312.5,
    "expected_value": 5937.0,
    "degrees_of_freedom": 5937
  },
  "measurement_counts": {
    "voltmeters": 120,
    "ammeters": 0,
    "wattmeters": 5132,
    "varmeters": 5132,
    "pmu_pairs": 0,
    "current_angle_meters": 0,
    "total": 10384
  },
  "per_voltage_level": [
    {
      "voltage_kv": 400.0,
      "n_buses": 156,
      "n_branches": 198,
      "measurement_counts": { ... }
    }
  ],
  "branch_flows": [
    {
      "branch_index": 0,
      "from_bus": 1,
      "to_bus": 2,
      "from": { "p": 1.23, "q": 0.45, "i_mag": 0.012 },
      "to":   { "p": -1.21, "q": -0.44, "i_mag": 0.012 },
      "p_loss": 0.02,
      "q_loss": 0.01
    }
  ],
  "bus_injections": [
    { "bus_index": 0, "p_inj": 3.45, "q_inj": 1.23 }
  ],
  "power_balance": {
    "total_p_loss": 0.87,
    "total_q_loss": 2.34,
    "total_p_generation": 45.6,
    "total_q_generation": 12.3,
    "total_p_load": 44.73,
    "total_q_load": 9.96
  }
}
```

### Zero-Injection Report Object

When zero-injection buses are detected:

```json
{
  "n_zi_buses": 42,
  "zi_virtual_pairs_injected": 42,
  "threshold_pu": 0.001,
  "all_clean": true,
  "violations": [],
  "zi_bus_injections": [
    {
      "bus_index": 15,
      "bus_label": 16,
      "p_estimated_pu": 0.000012,
      "q_estimated_pu": -0.000008
    }
  ]
}
```

If any estimated injection exceeds the threshold, `all_clean` is `false` and the `violations` array lists offending buses with `p_exceeds` / `q_exceeds` flags.

---

## GET /api/last-result

Returns the most recent `POST /api/estimate` response. Returns `404` if no estimation has been run yet.

---

## GET /api/measurements

Returns the full measurement set with estimated values and residuals (requires a prior SE run).

### Response

```json
[
  {
    "index": 0,
    "measurement_type": "voltmeter",
    "label": "V_bus1",
    "measured_value": 1.001,
    "standard_deviation": 0.004,
    "status": true,
    "estimated_value": 1.00058,
    "residual": 0.00042
  }
]
```

| Field | Type | Description |
|-------|------|-------------|
| `index` | `number` | Equation row index |
| `measurement_type` | `string` | `voltmeter`, `ammeter`, `wattmeter`, `varmeter`, `pmu_magnitude`, `pmu_angle` |
| `label` | `string` | Human-readable label (e.g. `V_bus1`, `P_br12_from`) |
| `measured_value` | `number` | Telemetered value \( z_i \) |
| `standard_deviation` | `number` | \( \sigma_i \) |
| `status` | `bool` | Whether the measurement is active |
| `estimated_value` | `number?` | \( h_i(\hat{x}) \) — only present for active measurements after SE |
| `residual` | `number?` | \( r_i = z_i - h_i(\hat{x}) \) — only present after SE |

---

## Pro Edition Endpoints

When compiled with `--features pro`, the following endpoints are additionally available. See the [Pro Edition](pro.md) page for a capabilities overview.

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/observability` | Run observability analysis |
| `POST` | `/api/redundancy` | Run redundancy analysis (supports `source: "estimate"` or `"bdd"`) |
| `POST` | `/api/bdd/run` | Run iterative bad data detection |
| `GET` | `/api/bdd/bad-data` | Active bad data set |
| `GET` | `/api/bdd/historical` | Historical bad data set |
| `GET` | `/api/bdd/alerts` | Current BDD alerts |
| `POST` | `/api/bdd/reset` | Clear BDD state |

---

## gRPC API

The same functionality is available via gRPC on **port 50051** using protobuf definitions in `proto/rusty_givens/v1/service.proto`.

### Free Edition Services

| Service | RPCs |
|---------|------|
| `EstimateService` | `GetNetwork`, `GetTrueState`, `RunEstimate`, `GetLastResult` |

### Pro Edition Services

| Service | RPCs |
|---------|------|
| `BddService` | `RunBdd` |
| `ObsService` | `RunObservability` |
| `RedService` | `RunRedundancy` |

---

## Examples

### Run estimation with Sparse Cholesky (default)

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{"factorization":"SparseCholesky","max_iterations":50,"tolerance":1e-4}'
```

### Run estimation with QR Givens formulation

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{"formulation":"OrthogonalQR","factorization":"SparseCholesky"}'
```

### Run estimation with explicit zero-injection buses

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{
    "factorization": "SparseCholesky",
    "zi_enabled": true,
    "zi_buses": [5, 12, 37, 104]
  }'
```

### Run DC estimation (active power only)

```bash
curl -X POST http://localhost:3001/api/estimate \
  -H 'Content-Type: application/json' \
  -d '{"formulation":"DcEstimation","factorization":"SparseCholesky"}'
```
