# Pro Edition

The Pro edition extends Rusty Givens with three analysis modules that build on top of the free SE kernel. These modules are compiled conditionally with `--features pro` and are available under a separate commercial license.

This page describes **what** the Pro edition can do. Implementation details are not covered in the public documentation.

---

## Observability Analysis

Observability Analysis determines whether the power system state can be uniquely estimated from the available measurement set.

### Capabilities

- **Decoupled P-θ / Q-V analysis** — the measurement set is split into two independent sub-models following the standard decoupling assumption:
    - **P-θ sub-model**: wattmeters (bus injections and branch flows), PMU bus voltage angles, and current angle meters constrain the bus voltage angles.
    - **Q-V sub-model**: varmeters (bus injections and branch flows), voltmeters, and PMU voltage magnitudes constrain the bus voltage magnitudes.
    - Overall observability requires **both** sub-models to be fully observable.

- **Two analysis methods**:
    - **Numerical** — based on DC gain matrix factorization; zero pivots indicate unobservable state variables.
    - **Topological** — graph-based approach using measurement-to-state assignment and Union-Find island detection.

- **Observable island identification** — when the system is not fully observable, the analysis identifies the separate observable islands (groups of buses that can be independently estimated) in each sub-model.

- **Per-bus and per-branch observability status** — each bus and branch is classified as observable or unobservable, with attribution to the sub-model(s) that cause unobservability.

- **P-Q pair validation** — verifies that active and reactive power measurements are properly paired at measurement locations. Reports unpaired wattmeters and varmeters.

- **Pseudo-measurement recommendations** — for each unobservable bus, the analysis recommends whether pseudo-measurements are needed, which sub-model(s) they should target, and whether nearby measurements exist for substitution or whether nominal values are the only option.

- **Observability gate** — when enabled, the estimate service runs an observability check before each SE run and rejects the request if the system is not fully observable, preventing unnecessary computation.

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/observability` | Run standalone observability analysis |

The observability result is also included in the SE response (`obs_check` field) when the gate is active.

---

## Redundancy Analysis

Redundancy Analysis classifies every measurement by its contribution to bad data detection and identification capability.

### Capabilities

- **Four-tier redundancy classification** — every measurement used in the estimation is assigned one of four classes:

    | Class | Meaning |
    |-------|---------|
    | **Critical** (M₀) | Removing this measurement would make parts of the system unobservable. A measurement error can neither be detected nor identified. |
    | **No Redundancy** (M₀′) | The measurement is slightly redundant, but the sensitivity to residuals of other measurements is too small for reliable error detection. |
    | **Simply Redundant** (M₁) | A measurement error can be detected but not uniquely identified — the error is strongly correlated with exactly one other measurement. |
    | **Multiply Redundant** (M₂) | A measurement error can be both detected and uniquely identified — no single partner dominates the correlation. |

- **Sigma-aware classification** — the redundancy level takes into account the individual measurement accuracy (σ). Two measurements of the same physical quantity with very different standard deviations may have very different redundancy levels, since a highly weighted measurement error produces smaller residuals in correlated partners.

- **Global redundancy metrics** — overall system statistics including total measurements, state variables, degrees of freedom, redundancy ratio, and a sufficiency flag.

- **Per-bus local redundancy** — for each bus: number of connected measurements, minimum detection sensitivity, and counts per redundancy class.

- **Per-branch local redundancy** — for each branch: number of connected measurements, minimum detection sensitivity, and counts per redundancy class.

- **Good/Bad marks per bus and branch** — each network element receives marks based on the redundancy of its associated measurements:
    - **Bad marks** are assigned for measurements classified as Critical, No Redundancy, or Simply Redundant.
    - **Good marks** are assigned for measurements classified as Multiply Redundant.
    - Marks are broken down **by measurement type** (P, Q, V, I, Angle_V, Angle_I) per element, as well as totalled across types.

- **Before and after Bad Data Treatment** — redundancy analysis can be run twice: once on the original measurement set (before BDD) and once on the reduced measurement set (after BDD has eliminated bad measurements). The consumer can compare the two snapshots to identify loss of redundancy:
    - Multiple redundancy → Single / No redundancy / Critical
    - Single redundancy → No redundancy / Critical
    - No redundancy → Critical

- **Per-measurement detail** — for each measurement: detection sensitivity (w_ii), maximum correlation coefficient (max |k_ik|), the correlated partner's equation row, coupling indicator, associated buses, and associated branches.

- **Filterable and sortable export** — the API response provides all redundancy information structured for display, filtering, and sorting by:
    - Redundancy class (before and after BDD)
    - Measurement type (P, Q, V, I, angles)
    - Network element (bus or branch association)
    - Loss of redundancy (by comparing before/after snapshots)

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/redundancy` | Run redundancy analysis. Use `"source": "estimate"` (default) for before-BDD or `"source": "bdd"` for after-BDD. |

### Typical Workflow

1. `POST /api/estimate` — run state estimation.
2. `POST /api/redundancy` with `{"source": "estimate"}` — get before-BDD redundancy snapshot.
3. `POST /api/bdd/run` — run iterative bad data detection (eliminates bad measurements).
4. `POST /api/redundancy` with `{"source": "bdd"}` — get after-BDD redundancy snapshot.
5. Compare the two results to identify redundancy loss.

---

## Bad Data Detection

Bad Data Detection (BDD) identifies and eliminates erroneous measurements through an iterative pipeline.

### Capabilities

- **Chi-squared global detection test** — tests the WLS objective function against a χ² distribution to determine whether bad data is likely present in the measurement set.

- **Residual-based identification** — two residual test types:
    - **Weighted residuals** \( |r_i| \cdot \sqrt{w_i} \) — fast, no matrix inversion required.
    - **Normalized residuals** \( |r_i| / \sqrt{\Omega_{ii}} \) — more accurate, accounts for residual covariance.

- **Iterative elimination** — when bad data is detected, the worst measurement is identified and eliminated. The SE is re-run with the reduced measurement set. This cycle repeats until:
    - No residual exceeds the threshold (measurement set is clean), or
    - A configurable maximum number of eliminations is reached.

- **Two elimination strategies**:
    - **Full re-estimation** — re-runs the complete SE after each elimination.
    - **Givens compensation** — applies a rank-1 downdate to the factored gain matrix, avoiding a full re-solve.

- **Bad marks** — after each elimination, bad marks are assigned to buses and branches associated with the eliminated measurement, by measurement category (P, Q, V, I, etc.).

- **Pseudo-measurement injection** — when an elimination would cause the system to become unobservable (measurement count falls below the minimum), pseudo-measurements are automatically injected at affected buses to maintain observability.

- **Post-BDD artifacts** — the solver artifacts from the final re-estimation after all eliminations are retained, enabling subsequent redundancy analysis on the reduced measurement set.

### Measurement Lifecycle Management

BDD includes a stateful lifecycle tracker that persists across multiple BDD runs, modeling the real-world SCADA operator workflow:

- **Alert generation** — when a measurement is identified as bad in *n* consecutive runs, an alert is raised.

- **Automatic deactivation** — after *m₁* consecutive identifications, the measurement is deactivated (permanently excluded from SE until released).

- **Release cycle** — after *m₂* consecutive clean runs (no identification), a deactivated measurement is released back into the active set.

- **Bad data set** — measurements identified more than *n₁* consecutive times are promoted to the active bad data set for monitoring.

- **Historical bad data set** — measurements that are released from the bad data set but have a history of identifications are tracked in the historical set.

All lifecycle parameters (*n*, *m₁*, *m₂*, *n₁*, *n₂*) are configurable per BDD run.

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/bdd/run` | Run iterative BDD pipeline |
| `GET` | `/api/bdd/bad-data` | View the active bad data set (currently flagged measurements) |
| `GET` | `/api/bdd/historical` | View the historical bad data set |
| `GET` | `/api/bdd/alerts` | View current BDD alerts |
| `POST` | `/api/bdd/reset` | Clear all BDD state (lifecycles, bad data sets, alerts) |

### Configurable Parameters

| Parameter | Description |
|-----------|-------------|
| Residual type | Weighted or Normalized |
| Residual threshold | Detection threshold for the largest residual |
| Chi² enabled | Toggle the global detection test |
| Chi² confidence | Confidence level for the Chi² threshold |
| Max eliminations | Maximum bad measurements to eliminate per run |
| Elimination strategy | Full re-estimation or Givens compensation |
| Pseudo-measurement injection | Automatically inject pseudo-measurements to maintain observability |
| Alert threshold (*n*) | Consecutive identifications before an alert |
| Deactivation threshold (*m₁*) | Consecutive identifications before deactivation |
| Release cycle (*m₂*) | Consecutive clean runs before release |
| Bad data set threshold (*n₁*) | Consecutive identifications for bad data set entry |
| Historical threshold (*n₂*) | Consecutive clean runs to move from bad data to historical |

---

## gRPC Services

The Pro edition registers three additional gRPC services on port 50051:

| Service | RPC | Description |
|---------|-----|-------------|
| `ObsService` | `RunObservability` | Observability analysis |
| `RedService` | `RunRedundancy` | Redundancy analysis |
| `BddService` | `RunBdd` | Iterative bad data detection |

Protobuf definitions are in `proto/rusty_givens/v1/service.proto`.
