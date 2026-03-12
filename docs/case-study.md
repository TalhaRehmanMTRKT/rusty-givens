# Case Study

## GB Transmission Network

The default case study is a reduced version of the Great Britain (GB) transmission network, originally sourced from Bukhsh & McKinnon (2013) via pandapower.

| Metric | Value |
|--------|-------|
| **Buses** | 2,224 |
| **Branches** | 3,207 |
| **Measurements** | ~10,384 |
| **State variables** | 4,447 |
| **Voltage levels** | 6.6, 11, 22, 33, 66, 132, 275, 400 kV |
| **Base MVA** | 100 |

The case file is located at `case_study/gb_network.json`.

### Measurement Composition

The measurement set is synthetically generated from the power-flow solution (true state) with Gaussian noise added according to typical SCADA measurement accuracies:

- **Wattmeters** — bus injections and branch flows (P)
- **Varmeters** — bus injections and branch flows (Q)
- **Voltmeters** — bus voltage magnitudes (|V|)

No ammeters, PMUs, or current angle meters are included in the default case.

---

## EHV Network (275/400 kV)

A further-reduced version retaining only the **Extra-High Voltage** grid (275 kV and 400 kV):

| Metric | Full GB | EHV Reduced |
|--------|---------|-------------|
| Buses | 2,224 | 793 |
| Branches | 3,207 | 1,039 |
| Measurements | ~10,384 | 4,290 |
| State variables | 4,447 | 1,585 |
| Redundancy ratio | ~2.33 | ~2.71 |

Sub-transmission and distribution networks (≤ 132 kV) are replaced by **equivalent constant-power injections** at boundary buses. These injections represent the aggregated net power exchange between the EHV grid and the lower-voltage networks.

The EHV case file is located at `case_study/ehv_network/gb_ehv_network.json`.

### Regenerating the EHV Case

```bash
python case_study/ehv_network/extract_ehv_network.py
```

This script reads the full GB network, removes all non-EHV buses and branches, and places equivalent injections at the boundary buses.

---

## Network Parameters

Raw network parameters (exported from pandapower as CSV) are available in `case_study/network_params/`:

| File | Contents |
|------|----------|
| `bus.csv` | Bus data: label, nominal voltage, type, demand |
| `line.csv` | Line parameters: R, X, B, length, max loading |
| `trafo.csv` | Transformer parameters: tap ratio, phase shift, impedance |
| `gen.csv` | Generator data: scheduled P, Q limits, voltage setpoint |
| `load.csv` | Load data: active and reactive demand |
| `shunt.csv` | Shunt compensation data |
| `ext_grid.csv` | External grid (slack bus) data |
| `voltage_levels.csv` | Summary of buses and branches per voltage level |

### Regenerating Parameters

```bash
python case_study/export_network_params.py
```

---

## Python Tooling

Several Python scripts are provided for case extraction, comparison, and validation:

| Script | Purpose |
|--------|---------|
| `extract_gb_network.py` | Extract the full GB network from pandapower to JSON |
| `extract_ehv_network.py` | Extract the EHV-only reduced network |
| `export_network_params.py` | Export raw network parameters to CSV |
| `compare_solver_formulations.py` | Compare SE results across all six formulations |
| `compare_se.py` | Compare Rusty Givens vs. pandapower SE results |
| `run_julia_se.jl` | Run JuliaGrid SE for cross-validation |

---

## Data Format

Case files are JSON with the following top-level structure:

```json
{
  "info": {
    "n_buses": 2224,
    "n_branches": 3207,
    "slack_bus_index": 0,
    "base_mva": 100.0
  },
  "buses": [ ... ],
  "branches": [ ... ],
  "measurements": {
    "voltmeters": [ ... ],
    "ammeters": [ ... ],
    "wattmeters": [ ... ],
    "varmeters": [ ... ],
    "pmus": [ ... ],
    "current_angle_meters": [ ... ]
  },
  "true_state": {
    "voltage_magnitude": [ ... ],
    "voltage_angle": [ ... ]
  }
}
```

The I/O layer (`rusty-givens-io`) deserializes this format via `load_case()`.
