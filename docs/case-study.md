# Case Study

## GB Network

The default case is a reduced version of the Great Britain transmission network:

- **Buses**: 2,224
- **Branches**: 3,207
- **Measurements**: ~10,384

The case file is located at `case_study/gb_network.json`.

## EHV Network (275/400 kV)

A further-reduced version retaining only the Extra-High-Voltage (275 kV + 400 kV) grid:

| Metric | Full GB | EHV reduced |
|--------|---------|--------------|
| Buses | 2,224 | 793 |
| Branches | 3,207 | 1,039 |
| Measurements | ~14,000 | 4,290 |
| State variables | 4,447 | 1,585 |

Sub-transmission and distribution networks (≤132 kV) are replaced by **equivalent constant-power injections** at boundary buses.

See `case_study/ehv_network/README.md` for details.

## Network parameters

Case-specific parameters (e.g. base MVA, voltage base) are documented in `case_study/network_params/README.md`.
