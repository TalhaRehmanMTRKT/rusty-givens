# Pandapower GB Network → Rust Solver Conversion

This document explains how the pandapower GB network is converted into a format that the Rust state estimation solver can consume.

## Overview

The conversion pipeline is:

```
pandapower GBNetwork  →  extract_gb_network.py  →  gb_network.json  →  rusty-givens-io  →  PowerSystem + AcModel
```

The Python script `extract_gb_network.py` acts as the bridge: it loads the pandapower network, runs power flow, generates measurements, and exports a single JSON file. The Rust crate `rusty-givens-io` deserializes this JSON into `PowerSystem`, `MeasurementSet`, and `TrueState` structs that the solver uses.

---

## 1. Pandapower Network Model

Pandapower models the GB network with separate element types:

| Element   | Table      | Purpose                                      |
|-----------|------------|----------------------------------------------|
| Buses     | `net.bus`  | Nodes with voltage level (`vn_kv`), zone, geo |
| Lines     | `net.line` | Overhead/cable branches (π-model)            |
| Transformers | `net.trafo` | Two-winding transformers with tap/phase shift |
| Generators | `net.gen` | PV buses with P setpoint                      |
| Loads     | `net.load` | P, Q demand per bus                           |
| Shunts    | `net.shunt`| Fixed admittance (g, b) at buses              |
| Ext. grid | `net.ext_grid` | Slack bus(es)                            |

Units: MW, MVAr, kV, Ω/km, nF/km, etc. Base MVA is `net.sn_mva`.

---

## 2. Bus Conversion (`build_bus_data`)

**Pandapower → JSON:**

| Pandapower source              | JSON field       | Notes                                                |
|--------------------------------|------------------|------------------------------------------------------|
| `net.bus.index`                | `label`          | Bus ID                                               |
| `net.ext_grid["bus"]`          | `bus_type: 3`    | Slack if bus is in ext_grid                          |
| `net.gen["bus"]`               | `bus_type: 2`    | PV if bus has generator (and not slack)             |
| else                           | `bus_type: 1`   | PQ                                                   |
| `net.load` aggregated by bus  | `p_demand_pu`, `q_demand_pu` | Sum of loads at bus, converted to p.u.      |
| `net.shunt` aggregated by bus | `g_shunt_pu`, `b_shunt_pu`  | Shunt admittance in p.u.; pandapower Q sign flipped |
| `net.bus["vn_kv"]`             | `vn_kv`          | Nominal voltage (kV)                                 |
| `net.bus["geo"]`               | `geo_x`, `geo_y` | Parsed from GeoJSON if present                       |
| —                              | `vm_init`, `va_init` | Fixed at 1.0, 0.0 for initialization          |

**Rust consumption:** `BusJson` → `Bus` with `active_demand`, `reactive_demand`, `shunt_conductance`, `shunt_susceptance`, etc. Bus labels are mapped to sequential indices via `bus_index`.

---

## 3. Branch Conversion (`build_branch_data`)

Branches are a **unified representation** of both lines and transformers. The Rust solver treats them identically as π-model branches with optional tap and phase shift.

### 3.1 Lines

**Pandapower line parameters (per km):**

- `r_ohm_per_km`, `x_ohm_per_km` → series R, X
- `c_nf_per_km` → line charging (capacitive susceptance)
- `g_us_per_km` → conductance (if present)

**Conversion:**

1. Total impedance: `r_ohm = r_ohm_per_km * length_km`, `x_ohm = x_ohm_per_km * length_km`
2. Base impedance at from-bus voltage: `z_base = vn_from² / base_mva`
3. Per-unit: `r_pu = r_ohm / z_base`, `x_pu = x_ohm / z_base`
4. Shunt susceptance (π-model, total): `b_pu = 2π × 50 × c_nf × 1e-9 × z_base`
5. Shunt conductance: `g_pu = g_us × 1e-6 × z_base`
6. Tap: `tap_ratio = 1.0`, `shift_angle = 0.0` for lines

### 3.2 Transformers

**Pandapower trafo parameters:**

- `vn_hv_kv`, `vn_lv_kv` — nominal voltages
- `sn_mva` — rated power
- `vk_percent`, `vkr_percent` — short-circuit voltage and resistive part
- `i0_percent`, `pfe_kw` — no-load current and iron losses
- `tap_pos`, `tap_neutral`, `tap_step_percent` — tap changer
- `shift_degree` — phase shift (degrees)

**Conversion:**

1. Series impedance (p.u. on system base):
   - `zk = vk_percent/100 × (base_mva/sn_mva)`
   - `rk = vkr_percent/100 × (base_mva/sn_mva)`
   - `xk = √(zk² − rk²)`

2. Magnetizing shunt:
   - `ym = i0_percent/100 × (sn_mva/base_mva)`
   - `gm = pfe_kw/1000 / base_mva`
   - `bm = −√(ym² − gm²)` (inductive)

3. Tap ratio: `1 + (tap_pos − tap_neutral) × tap_step_percent/100`
4. Phase shift: `shift_angle = shift_degree` in radians
5. Branch direction: `from_bus = hv_bus`, `to_bus = lv_bus`
6. Shunt admittance in π-model: total `bm` and `gm` split equally → `susceptance = bm×2`, `conductance = gm×2` (as stored for the equivalent π-model)

**Rust consumption:** `BranchJson` → `Branch` with `resistance`, `reactance`, `susceptance`, `conductance`, `tap_ratio`, `shift_angle`. The `build_ac_model` function in `rusty-givens-core` builds the admittance matrix and measurement Jacobians from these parameters.

---

## 4. Measurements

Measurements are generated from the power flow solution (`net.res_bus`, `net.res_line`) with additive Gaussian noise:

| Type       | Location              | Noise (σ)   |
|------------|-----------------------|-------------|
| Voltmeters | All buses             | 0.4%        |
| Wattmeters | Bus injections, 80% of line flows | 1%  |
| Varmeters  | Same as wattmeters    | 1%          |
| Ammeters   | 50% of lines (from end) | 1%        |
| PMUs       | ~10% of buses         | 0.2% mag, 0.1 rad angle |

**Convention:** Pandapower uses load convention (positive P = consumed). The JSON uses injection convention (positive P = generated into network), so bus injections are negated.

**Rust consumption:** `MeasurementsJson` → `MeasurementSet` with voltmeters, ammeters, wattmeters, varmeters, PMUs. Branch labels in the JSON correspond to the sequential branch labels assigned during extraction (lines first, then trafos).

---

## 5. JSON Schema (Rust DTOs)

The Rust `rusty-givens-io` crate expects:

- **CaseJson:** `name`, `description`, `n_buses`, `n_branches`, `slack_bus_index`, `base_mva`, `buses`, `branches`, `measurements`, `true_state`
- **BusJson:** `label`, `bus_type`, `p_demand_pu`, `q_demand_pu`, `g_shunt_pu`, `b_shunt_pu`, `vm_init`, `va_init`, `vn_kv`, `geo_x?`, `geo_y?`
- **BranchJson:** `label`, `from_bus`, `to_bus`, `resistance`, `reactance`, `susceptance`, `conductance`, `tap_ratio`, `shift_angle`, `status`
- **TrueStateJson:** `voltage_magnitude`, `voltage_angle` (radians)

---

## 6. Summary

| Aspect        | Pandapower                         | Rust solver                         |
|---------------|------------------------------------|-------------------------------------|
| Units         | MW, MVAr, kV, Ω, nF, etc.          | Per-unit (base MVA)                  |
| Branches      | Separate `line` and `trafo` tables | Unified `Branch` with π-model        |
| Bus types     | ext_grid, gen, load → implicit     | Explicit 1=PQ, 2=PV, 3=Slack         |
| Shunt sign    | Q positive = inductive            | b_shunt_pu = −Q/base_mva             |
| Power sign    | Load convention                    | Injection convention in JSON        |

The conversion ensures the Rust solver receives a consistent, per-unit AC network model that matches the formulation used by JuliaGrid and pandapower for cross-validation.
