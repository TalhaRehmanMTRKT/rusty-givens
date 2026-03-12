"""
Extract the pandapower GB network, run power flow to obtain the true system state,
generate noisy measurements, and export everything as JSON for both JuliaGrid and Rust.

The GB network: 2224 buses, 3207 branches, 394 generators.
Source: W. A. Bukhsh, K. McKinnon, "Network data of real transmission networks", 2013.
"""

import json
import time
import numpy as np
import pandapower as pp
import pandapower.networks as pn

np.random.seed(42)

# ── Measurement noise standard deviations ──
VOLTAGE_MAG_STD = 0.004      # 0.4% for voltmeters
ACTIVE_POWER_STD = 0.01      # 1% for wattmeters
REACTIVE_POWER_STD = 0.01    # 1% for varmeters
CURRENT_MAG_STD = 0.01       # 1% for ammeters
PMU_MAG_STD = 0.002          # 0.2% for PMU magnitude
PMU_ANG_STD = 0.001          # 0.1 rad (~0.06°) for PMU angle


def main():
    print("=" * 60)
    print("GB Network Extraction for State Estimation Case Study")
    print("=" * 60)

    # ── 1. Load GB network ──
    print("\n[1] Loading pandapower GB network...")
    t0 = time.time()
    net = pn.GBnetwork()
    print(f"    Loaded in {time.time() - t0:.2f}s")
    print(f"    Buses:      {len(net.bus)}")
    print(f"    Lines:      {len(net.line)}")
    print(f"    Trafos:     {len(net.trafo)}")
    print(f"    Generators: {len(net.gen)}")
    print(f"    Ext. grids: {len(net.ext_grid)}")
    print(f"    Loads:      {len(net.load)}")
    print(f"    Shunts:     {len(net.shunt)}")

    # ── 2. Review P, Q values for loads and generators ──
    print("\n[2] Reviewing load/generation values from case data...")

    # Use the original case data. Pandapower's GBnetwork comes with realistic
    # P/Q values from the National Grid dataset.
    total_load_p = net.load.loc[net.load["in_service"], "p_mw"].sum()
    total_load_q = net.load.loc[net.load["in_service"], "q_mvar"].sum()
    total_gen_p = net.gen.loc[net.gen["in_service"], "p_mw"].sum()
    ext_grid_p = total_load_p - total_gen_p
    print(f"    Total load:   P = {total_load_p:.1f} MW, Q = {total_load_q:.1f} MVAr")
    print(f"    Total gen:    P = {total_gen_p:.1f} MW")
    print(f"    Ext grid P:   ~{ext_grid_p:.1f} MW (slack picks up remainder)")

    # ── 3. Run AC power flow ──
    print("\n[3] Running AC power flow...")
    t0 = time.time()
    pp.runpp(net, algorithm="nr", max_iteration=50, tolerance_mva=1e-6)
    pf_time = time.time() - t0
    print(f"    Power flow converged in {pf_time:.3f}s")

    # Extract true state from power flow results
    true_vm = net.res_bus["vm_pu"].values
    true_va = np.deg2rad(net.res_bus["va_degree"].values)
    print(f"    V range: [{true_vm.min():.4f}, {true_vm.max():.4f}] p.u.")
    print(f"    θ range: [{np.rad2deg(true_va.min()):.2f}, {np.rad2deg(true_va.max()):.2f}] deg")

    # ── 4. Build bus/branch data ──
    print("\n[4] Building bus/branch model...")
    bus_data = build_bus_data(net)
    branch_data = build_branch_data(net)
    print(f"    Exported {len(bus_data)} buses, {len(branch_data)} branches")

    # ── 5. Generate measurements ──
    print("\n[5] Generating noisy measurements from power flow results...")
    measurements = generate_measurements(net, true_vm, true_va)
    n_meas = (len(measurements["voltmeters"]) +
              len(measurements["ammeters"]) +
              len(measurements["wattmeters"]) +
              len(measurements["varmeters"]) +
              2 * len(measurements["pmus"]))
    print(f"    Voltmeters:  {len(measurements['voltmeters'])}")
    print(f"    Ammeters:    {len(measurements['ammeters'])}")
    print(f"    Wattmeters:  {len(measurements['wattmeters'])}")
    print(f"    Varmeters:   {len(measurements['varmeters'])}")
    print(f"    PMUs:        {len(measurements['pmus'])}")
    print(f"    Total scalar equations: {n_meas}")

    # ── 6. Find slack bus ──
    slack_bus = int(net.ext_grid.at[net.ext_grid.index[0], "bus"])
    slack_idx = list(net.bus.index).index(slack_bus)

    # ── 7. Export to JSON ──
    print("\n[6] Exporting to JSON...")
    case_data = {
        "name": "GB Network (pandapower GBnetwork)",
        "description": (
            "Great Britain transmission network. "
            "2224 buses, 3207 branches, 394 generators. "
            "Source: W.A. Bukhsh & K. McKinnon, 2013."
        ),
        "n_buses": len(bus_data),
        "n_branches": len(branch_data),
        "slack_bus_index": slack_idx,
        "base_mva": float(net.sn_mva),
        "buses": bus_data,
        "branches": branch_data,
        "measurements": measurements,
        "true_state": {
            "voltage_magnitude": true_vm.tolist(),
            "voltage_angle": true_va.tolist(),
        },
    }

    out_path = "case_study/gb_network.json"
    with open(out_path, "w") as f:
        json.dump(case_data, f, indent=2)
    import os
    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"    Written to {out_path} ({size_mb:.1f} MB)")

    print("\n" + "=" * 60)
    print("Extraction complete.")
    print("=" * 60)


def build_bus_data(net):
    """Extract bus data as a list of dicts."""
    buses = []
    for idx in net.bus.index:
        bus_type = 1  # PQ default
        if idx in net.ext_grid["bus"].values:
            bus_type = 3  # Slack
        elif idx in net.gen["bus"].values:
            bus_type = 2  # PV

        # Aggregate loads at this bus
        load_mask = net.load["bus"] == idx
        p_demand = float(net.load.loc[load_mask, "p_mw"].sum()) if load_mask.any() else 0.0
        q_demand = float(net.load.loc[load_mask, "q_mvar"].sum()) if load_mask.any() else 0.0

        # Shunt at this bus
        shunt_mask = net.shunt["bus"] == idx
        g_shunt = float(net.shunt.loc[shunt_mask, "p_mw"].sum()) if shunt_mask.any() else 0.0
        b_shunt = float(net.shunt.loc[shunt_mask, "q_mvar"].sum()) if shunt_mask.any() else 0.0

        # Convert shunt from MW/MVAr to p.u. (at nominal voltage)
        base_mva = float(net.sn_mva)
        g_shunt_pu = g_shunt / base_mva
        b_shunt_pu = -b_shunt / base_mva  # pandapower convention: positive q = inductive

        # Geo coordinates from pandapower GeoJSON column
        geo_x = None
        geo_y = None
        geo_raw = net.bus.at[idx, "geo"]
        if geo_raw is not None and isinstance(geo_raw, str):
            geo_obj = json.loads(geo_raw)
            coords = geo_obj.get("coordinates", [])
            if len(coords) >= 2:
                geo_x = float(coords[0])
                geo_y = float(coords[1])

        bus_dict = {
            "label": int(idx),
            "bus_type": bus_type,
            "p_demand_pu": p_demand / base_mva,
            "q_demand_pu": q_demand / base_mva,
            "g_shunt_pu": g_shunt_pu,
            "b_shunt_pu": b_shunt_pu,
            "vm_init": 1.0,
            "va_init": 0.0,
            "vn_kv": float(net.bus.at[idx, "vn_kv"]),
        }
        if geo_x is not None:
            bus_dict["geo_x"] = geo_x
            bus_dict["geo_y"] = geo_y
        buses.append(bus_dict)
    return buses


def build_branch_data(net):
    """Extract line and transformer data as branches."""
    branches = []
    base_mva = float(net.sn_mva)
    label = 1

    # Lines
    for idx in net.line.index:
        if not net.line.at[idx, "in_service"]:
            continue
        from_bus = int(net.line.at[idx, "from_bus"])
        to_bus = int(net.line.at[idx, "to_bus"])
        vn_from = float(net.bus.at[from_bus, "vn_kv"])

        length = float(net.line.at[idx, "length_km"])
        r_ohm = float(net.line.at[idx, "r_ohm_per_km"]) * length
        x_ohm = float(net.line.at[idx, "x_ohm_per_km"]) * length
        c_nf = float(net.line.at[idx, "c_nf_per_km"]) * length
        g_us = float(net.line.at[idx, "g_us_per_km"]) * length if "g_us_per_km" in net.line.columns else 0.0

        z_base = vn_from ** 2 / base_mva
        r_pu = r_ohm / z_base
        x_pu = x_ohm / z_base
        b_pu = 2.0 * np.pi * 50.0 * c_nf * 1e-9 * z_base  # capacitive susceptance
        g_pu = g_us * 1e-6 * z_base

        branches.append({
            "label": label,
            "from_bus": from_bus,
            "to_bus": to_bus,
            "resistance": r_pu,
            "reactance": x_pu,
            "susceptance": b_pu,
            "conductance": g_pu,
            "tap_ratio": 1.0,
            "shift_angle": 0.0,
            "status": True,
        })
        label += 1

    # Transformers
    for idx in net.trafo.index:
        if not net.trafo.at[idx, "in_service"]:
            continue
        hv_bus = int(net.trafo.at[idx, "hv_bus"])
        lv_bus = int(net.trafo.at[idx, "lv_bus"])

        sn_mva = float(net.trafo.at[idx, "sn_mva"])
        vn_hv = float(net.trafo.at[idx, "vn_hv_kv"])
        vn_lv = float(net.trafo.at[idx, "vn_lv_kv"])
        vk_percent = float(net.trafo.at[idx, "vk_percent"])
        vkr_percent = float(net.trafo.at[idx, "vkr_percent"])
        pfe_kw = float(net.trafo.at[idx, "pfe_kw"])
        i0_percent = float(net.trafo.at[idx, "i0_percent"])

        # Series impedance in p.u. (on system base)
        zk = vk_percent / 100.0 * (base_mva / sn_mva)
        rk = vkr_percent / 100.0 * (base_mva / sn_mva)
        xk = np.sqrt(max(zk**2 - rk**2, 0.0))

        # Shunt (magnetizing) admittance
        if sn_mva > 0 and i0_percent > 0:
            ym = i0_percent / 100.0 * (sn_mva / base_mva)
            gm = (pfe_kw / 1000.0) / base_mva
            bm = -np.sqrt(max(ym**2 - gm**2, 0.0))
        else:
            gm = 0.0
            bm = 0.0

        # Tap ratio
        tap_pos = net.trafo.at[idx, "tap_pos"] if "tap_pos" in net.trafo.columns and not np.isnan(net.trafo.at[idx, "tap_pos"]) else 0
        tap_step_percent = net.trafo.at[idx, "tap_step_percent"] if "tap_step_percent" in net.trafo.columns and not np.isnan(net.trafo.at[idx, "tap_step_percent"]) else 0
        tap_neutral = net.trafo.at[idx, "tap_neutral"] if "tap_neutral" in net.trafo.columns and not np.isnan(net.trafo.at[idx, "tap_neutral"]) else 0
        tap_ratio = 1.0 + (tap_pos - tap_neutral) * tap_step_percent / 100.0
        if tap_ratio == 0:
            tap_ratio = 1.0

        shift = float(net.trafo.at[idx, "shift_degree"]) if "shift_degree" in net.trafo.columns else 0.0
        shift_rad = np.deg2rad(shift)

        branches.append({
            "label": label,
            "from_bus": hv_bus,
            "to_bus": lv_bus,
            "resistance": rk,
            "reactance": xk,
            "susceptance": bm * 2,   # total shunt (split equally in π-model)
            "conductance": gm * 2,
            "tap_ratio": tap_ratio,
            "shift_angle": shift_rad,
            "status": True,
        })
        label += 1

    return branches


def generate_measurements(net, true_vm, true_va):
    """
    Generate a realistic measurement set from power flow results:
    - Voltage magnitude at every bus (voltmeter)
    - Active/reactive power injection at every load/gen bus (wattmeter/varmeter)
    - Active/reactive power flow at from-bus end of 80% of branches
    - Current magnitude at from-bus end of 50% of branches
    - PMUs at ~10% of buses
    """
    base_mva = float(net.sn_mva)
    bus_list = list(net.bus.index)
    bus_to_idx = {b: i for i, b in enumerate(bus_list)}

    measurements = {
        "voltmeters": [],
        "ammeters": [],
        "wattmeters": [],
        "varmeters": [],
        "pmus": [],
    }

    # ── Voltmeters at all buses ──
    for i, bus_idx in enumerate(bus_list):
        true_val = float(true_vm[i])
        noise = np.random.normal(0, VOLTAGE_MAG_STD)
        measurements["voltmeters"].append({
            "label": f"V_{bus_idx}",
            "bus": int(bus_idx),
            "magnitude": true_val + noise,
            "variance": VOLTAGE_MAG_STD ** 2,
        })

    # ── Active/reactive power injections at all buses ──
    # NOTE: pandapower res_bus uses the load convention (positive = consumed),
    # but the standard power-flow / JuliaGrid h(x) uses the injection convention
    # (positive = generated into the network).  We negate here to match.
    for i, bus_idx in enumerate(bus_list):
        p_inj = -float(net.res_bus.at[bus_idx, "p_mw"]) / base_mva
        q_inj = -float(net.res_bus.at[bus_idx, "q_mvar"]) / base_mva

        noise_p = np.random.normal(0, ACTIVE_POWER_STD)
        noise_q = np.random.normal(0, REACTIVE_POWER_STD)

        measurements["wattmeters"].append({
            "label": f"P_inj_{bus_idx}",
            "location": {"Bus": int(bus_idx)},
            "active": p_inj + noise_p,
            "variance": ACTIVE_POWER_STD ** 2,
        })
        measurements["varmeters"].append({
            "label": f"Q_inj_{bus_idx}",
            "location": {"Bus": int(bus_idx)},
            "reactive": q_inj + noise_q,
            "variance": REACTIVE_POWER_STD ** 2,
        })

    # ── Active/reactive power flows at from-bus end of 80% of lines ──
    line_indices = list(net.line.index[net.line["in_service"]])
    n_flow = int(0.8 * len(line_indices))
    flow_lines = np.random.choice(line_indices, n_flow, replace=False)

    for line_idx in flow_lines:
        p_from = float(net.res_line.at[line_idx, "p_from_mw"]) / base_mva
        q_from = float(net.res_line.at[line_idx, "q_from_mvar"]) / base_mva

        # Find which branch label this corresponds to
        br_label = int(line_idx) + 1  # 1-indexed

        noise_p = np.random.normal(0, ACTIVE_POWER_STD)
        noise_q = np.random.normal(0, REACTIVE_POWER_STD)

        measurements["wattmeters"].append({
            "label": f"P_flow_{line_idx}",
            "location": {"Branch": {"branch": br_label, "end": "From"}},
            "active": p_from + noise_p,
            "variance": ACTIVE_POWER_STD ** 2,
        })
        measurements["varmeters"].append({
            "label": f"Q_flow_{line_idx}",
            "location": {"Branch": {"branch": br_label, "end": "From"}},
            "reactive": q_from + noise_q,
            "variance": REACTIVE_POWER_STD ** 2,
        })

    # ── Current magnitude at from-bus end of 50% of lines ──
    n_current = int(0.5 * len(line_indices))
    current_lines = np.random.choice(line_indices, n_current, replace=False)

    for line_idx in current_lines:
        i_from = float(net.res_line.at[line_idx, "i_from_ka"])
        from_bus = int(net.line.at[line_idx, "from_bus"])
        vn = float(net.bus.at[from_bus, "vn_kv"])
        i_base = base_mva / (np.sqrt(3) * vn)
        i_pu = i_from / i_base

        br_label = int(line_idx) + 1
        noise = np.random.normal(0, CURRENT_MAG_STD)

        measurements["ammeters"].append({
            "label": f"I_{line_idx}",
            "branch": br_label,
            "end": "From",
            "magnitude": max(i_pu + noise, 0.001),
            "variance": CURRENT_MAG_STD ** 2,
            "square": False,
        })

    # ── PMUs at ~10% of buses (evenly spaced) ──
    n_pmu = max(1, int(0.10 * len(bus_list)))
    pmu_buses = np.random.choice(bus_list, n_pmu, replace=False)

    for bus_idx in pmu_buses:
        i = bus_to_idx[bus_idx]
        vm_true = float(true_vm[i])
        va_true = float(true_va[i])

        noise_m = np.random.normal(0, PMU_MAG_STD)
        noise_a = np.random.normal(0, PMU_ANG_STD)

        measurements["pmus"].append({
            "label": f"PMU_{bus_idx}",
            "location": {"Bus": int(bus_idx)},
            "magnitude": vm_true + noise_m,
            "angle": va_true + noise_a,
            "variance_magnitude": PMU_MAG_STD ** 2,
            "variance_angle": PMU_ANG_STD ** 2,
            "coordinate": "Rectangular",
            "correlated": False,
            "square": False,
        })

    return measurements


if __name__ == "__main__":
    main()
