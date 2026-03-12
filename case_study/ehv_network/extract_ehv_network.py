"""
Extract the 275/400 kV Extra-High-Voltage (EHV) transmission network from
the full pandapower GB network and represent the sub-transmission /
distribution networks as equivalent power injections at boundary buses.

Equivalent-injection approach
─────────────────────────────
1.  Run AC power flow on the *full* GB network.
2.  Identify **boundary transformers**: branches where the HV side is ≥ 275 kV
    and the LV side is < 275 kV.
3.  At each HV-side boundary bus, sum the active and reactive power flowing
    through all boundary transformers connected to that bus.  This aggregate
    power represents the entire sub-transmission/distribution network as a
    single constant-power load (or generation if power flows upward).
4.  Augment the bus demand at each boundary bus by that aggregate power.
5.  Retain only buses ≥ 275 kV and branches whose *both* endpoints are
    in the retained set.  Generate fresh measurements from the true state
    restricted to the retained buses.

Output
──────
    case_study/ehv_network/gb_ehv_network.json

Usage
─────
    python case_study/ehv_network/extract_ehv_network.py
"""

import json
import os
import sys
import time
import numpy as np
import pandapower as pp
import pandapower.networks as pn

np.random.seed(42)

EHV_THRESHOLD_KV = 275.0

VOLTAGE_MAG_STD  = 0.004
ACTIVE_POWER_STD = 0.01
REACTIVE_POWER_STD = 0.01
CURRENT_MAG_STD  = 0.01
PMU_MAG_STD      = 0.002
PMU_ANG_STD      = 0.001


def main():
    print("=" * 70)
    print("EHV Network Extraction with Equivalent Injections")
    print("=" * 70)

    # ── 1. Load and run power flow on full GB network ────────────────
    print("\n[1] Loading pandapower GB network...")
    t0 = time.time()
    net = pn.GBnetwork()
    print(f"    Loaded in {time.time() - t0:.2f}s")
    print(f"    Full network: {len(net.bus)} buses, "
          f"{len(net.line)} lines, {len(net.trafo)} trafos")

    print("\n[2] Running AC power flow on full network...")
    pp.runpp(net, algorithm="nr", max_iteration=50, tolerance_mva=1e-6)
    true_vm_full = net.res_bus["vm_pu"].values.copy()
    true_va_full = np.deg2rad(net.res_bus["va_degree"].values.copy())
    print(f"    Converged.  V range: [{true_vm_full.min():.4f}, "
          f"{true_vm_full.max():.4f}] p.u.")

    base_mva = float(net.sn_mva)
    bus_list_full = list(net.bus.index)
    bus_to_seqidx = {b: i for i, b in enumerate(bus_list_full)}

    # ── 2. Identify retained EHV buses and boundary transformers ─────
    print("\n[3] Identifying EHV buses and boundary transformers...")
    ehv_bus_set = set(net.bus.index[net.bus["vn_kv"] >= EHV_THRESHOLD_KV])
    print(f"    EHV buses (≥{EHV_THRESHOLD_KV} kV): {len(ehv_bus_set)}")

    boundary_trafos = []
    for idx in net.trafo.index:
        if not net.trafo.at[idx, "in_service"]:
            continue
        hv_bus = int(net.trafo.at[idx, "hv_bus"])
        lv_bus = int(net.trafo.at[idx, "lv_bus"])
        vn_hv = float(net.bus.at[hv_bus, "vn_kv"])
        vn_lv = float(net.bus.at[lv_bus, "vn_kv"])
        if vn_hv >= EHV_THRESHOLD_KV and vn_lv < EHV_THRESHOLD_KV:
            boundary_trafos.append(idx)

    print(f"    Boundary transformers (EHV→sub-transmission): {len(boundary_trafos)}")

    # ── 3. Compute equivalent injections at boundary buses ───────────
    print("\n[4] Computing equivalent injections at boundary buses...")
    equiv_p_mw = {}   # bus_id → additional P demand (MW)
    equiv_q_mvar = {}  # bus_id → additional Q demand (MVAr)

    for tidx in boundary_trafos:
        hv_bus = int(net.trafo.at[tidx, "hv_bus"])
        # p_hv_mw: power flowing into the trafo at the HV side (positive = HV→LV)
        p_hv = float(net.res_trafo.at[tidx, "p_hv_mw"])
        q_hv = float(net.res_trafo.at[tidx, "q_hv_mvar"])
        equiv_p_mw[hv_bus] = equiv_p_mw.get(hv_bus, 0.0) + p_hv
        equiv_q_mvar[hv_bus] = equiv_q_mvar.get(hv_bus, 0.0) + q_hv

    n_boundary_buses = len(equiv_p_mw)
    total_equiv_p = sum(equiv_p_mw.values())
    total_equiv_q = sum(equiv_q_mvar.values())
    print(f"    Boundary buses receiving equiv. injection: {n_boundary_buses}")
    print(f"    Total equiv. P injection: {total_equiv_p:.1f} MW")
    print(f"    Total equiv. Q injection: {total_equiv_q:.1f} MVAr")

    # ── 4. Build reduced bus data ────────────────────────────────────
    print("\n[5] Building reduced EHV network...")
    ehv_buses_sorted = sorted(ehv_bus_set)
    old_to_new = {old: new for new, old in enumerate(ehv_buses_sorted)}

    bus_data = []
    for new_idx, old_idx in enumerate(ehv_buses_sorted):
        bus_type = 1  # PQ
        if old_idx in net.ext_grid["bus"].values:
            bus_type = 3
        elif old_idx in net.gen["bus"].values:
            bus_type = 2

        # Original demand at this bus
        load_mask = net.load["bus"] == old_idx
        p_demand = float(net.load.loc[load_mask, "p_mw"].sum()) if load_mask.any() else 0.0
        q_demand = float(net.load.loc[load_mask, "q_mvar"].sum()) if load_mask.any() else 0.0

        # Add equivalent injection from removed distribution network
        p_demand += equiv_p_mw.get(old_idx, 0.0)
        q_demand += equiv_q_mvar.get(old_idx, 0.0)

        # Shunt
        shunt_mask = net.shunt["bus"] == old_idx
        g_shunt = float(net.shunt.loc[shunt_mask, "p_mw"].sum()) if shunt_mask.any() else 0.0
        b_shunt = float(net.shunt.loc[shunt_mask, "q_mvar"].sum()) if shunt_mask.any() else 0.0

        # Geo coordinates
        geo_x, geo_y = None, None
        geo_raw = net.bus.at[old_idx, "geo"]
        if geo_raw is not None and isinstance(geo_raw, str):
            geo_obj = json.loads(geo_raw)
            coords = geo_obj.get("coordinates", [])
            if len(coords) >= 2:
                geo_x, geo_y = float(coords[0]), float(coords[1])

        entry = {
            "label": int(old_idx),
            "bus_type": bus_type,
            "p_demand_pu": p_demand / base_mva,
            "q_demand_pu": q_demand / base_mva,
            "g_shunt_pu": g_shunt / base_mva,
            "b_shunt_pu": -b_shunt / base_mva,
            "vm_init": 1.0,
            "va_init": 0.0,
            "vn_kv": float(net.bus.at[old_idx, "vn_kv"]),
        }
        if geo_x is not None:
            entry["geo_x"] = geo_x
            entry["geo_y"] = geo_y
        bus_data.append(entry)

    # ── 5. Build reduced branch data ─────────────────────────────────
    branch_data = []
    label = 1

    # Lines (only if both endpoints are EHV)
    for idx in net.line.index:
        if not net.line.at[idx, "in_service"]:
            continue
        fb = int(net.line.at[idx, "from_bus"])
        tb = int(net.line.at[idx, "to_bus"])
        if fb not in ehv_bus_set or tb not in ehv_bus_set:
            continue

        vn_from = float(net.bus.at[fb, "vn_kv"])
        length = float(net.line.at[idx, "length_km"])
        r_ohm = float(net.line.at[idx, "r_ohm_per_km"]) * length
        x_ohm = float(net.line.at[idx, "x_ohm_per_km"]) * length
        c_nf  = float(net.line.at[idx, "c_nf_per_km"]) * length
        g_us  = (float(net.line.at[idx, "g_us_per_km"]) * length
                 if "g_us_per_km" in net.line.columns else 0.0)

        z_base = vn_from ** 2 / base_mva
        branch_data.append({
            "label": label,
            "from_bus": fb,
            "to_bus": tb,
            "resistance": r_ohm / z_base,
            "reactance": x_ohm / z_base,
            "susceptance": 2.0 * np.pi * 50.0 * c_nf * 1e-9 * z_base,
            "conductance": g_us * 1e-6 * z_base,
            "tap_ratio": 1.0,
            "shift_angle": 0.0,
            "status": True,
        })
        label += 1

    # Transformers (only INTERNAL EHV: both sides ≥ 275 kV)
    for idx in net.trafo.index:
        if not net.trafo.at[idx, "in_service"]:
            continue
        hv_bus = int(net.trafo.at[idx, "hv_bus"])
        lv_bus = int(net.trafo.at[idx, "lv_bus"])
        if hv_bus not in ehv_bus_set or lv_bus not in ehv_bus_set:
            continue

        sn = float(net.trafo.at[idx, "sn_mva"])
        vk = float(net.trafo.at[idx, "vk_percent"])
        vkr = float(net.trafo.at[idx, "vkr_percent"])
        pfe = float(net.trafo.at[idx, "pfe_kw"])
        i0 = float(net.trafo.at[idx, "i0_percent"])

        zk = vk / 100.0 * (base_mva / sn)
        rk = vkr / 100.0 * (base_mva / sn)
        xk = np.sqrt(max(zk**2 - rk**2, 0.0))

        if sn > 0 and i0 > 0:
            ym = i0 / 100.0 * (sn / base_mva)
            gm = (pfe / 1000.0) / base_mva
            bm = -np.sqrt(max(ym**2 - gm**2, 0.0))
        else:
            gm, bm = 0.0, 0.0

        tp = net.trafo.at[idx, "tap_pos"] if "tap_pos" in net.trafo.columns and not np.isnan(net.trafo.at[idx, "tap_pos"]) else 0
        ts = net.trafo.at[idx, "tap_step_percent"] if "tap_step_percent" in net.trafo.columns and not np.isnan(net.trafo.at[idx, "tap_step_percent"]) else 0
        tn = net.trafo.at[idx, "tap_neutral"] if "tap_neutral" in net.trafo.columns and not np.isnan(net.trafo.at[idx, "tap_neutral"]) else 0
        tap = 1.0 + (tp - tn) * ts / 100.0
        if tap == 0:
            tap = 1.0

        shift = float(net.trafo.at[idx, "shift_degree"]) if "shift_degree" in net.trafo.columns else 0.0

        branch_data.append({
            "label": label,
            "from_bus": hv_bus,
            "to_bus": lv_bus,
            "resistance": rk,
            "reactance": xk,
            "susceptance": bm * 2,
            "conductance": gm * 2,
            "tap_ratio": tap,
            "shift_angle": np.deg2rad(shift),
            "status": True,
        })
        label += 1

    print(f"    Retained buses:   {len(bus_data)}")
    print(f"    Retained branches: {len(branch_data)}")

    # ── 6. True state for retained buses ─────────────────────────────
    true_vm = [float(true_vm_full[bus_to_seqidx[b]]) for b in ehv_buses_sorted]
    true_va = [float(true_va_full[bus_to_seqidx[b]]) for b in ehv_buses_sorted]

    # ── 7. Find slack bus in reduced network ─────────────────────────
    slack_pp = int(net.ext_grid.at[net.ext_grid.index[0], "bus"])
    if slack_pp not in ehv_bus_set:
        sys.exit(f"ERROR: Slack bus {slack_pp} is not in the EHV set!")
    slack_new_idx = ehv_buses_sorted.index(slack_pp)
    print(f"    Slack bus: label={slack_pp}, new index={slack_new_idx} "
          f"({net.bus.at[slack_pp, 'vn_kv']} kV)")

    # ── 8. Generate measurements for reduced network ─────────────────
    print("\n[6] Generating measurements for reduced EHV network...")
    measurements = generate_measurements_ehv(
        net, ehv_buses_sorted, bus_to_seqidx, branch_data,
        true_vm, true_va, base_mva, equiv_p_mw, equiv_q_mvar,
    )
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
    print(f"    Total scalar: {n_meas}")

    # ── 9. Export JSON ───────────────────────────────────────────────
    print("\n[7] Exporting JSON...")
    case = {
        "name": "GB EHV Network (275/400 kV) with Equivalent Injections",
        "description": (
            f"Reduced GB transmission network: {len(bus_data)} EHV buses "
            f"(275+400 kV), {len(branch_data)} branches. "
            f"Distribution networks represented as equivalent constant-power "
            f"injections at {n_boundary_buses} boundary buses."
        ),
        "n_buses": len(bus_data),
        "n_branches": len(branch_data),
        "slack_bus_index": slack_new_idx,
        "base_mva": base_mva,
        "buses": bus_data,
        "branches": branch_data,
        "measurements": measurements,
        "true_state": {
            "voltage_magnitude": true_vm,
            "voltage_angle": true_va,
        },
    }

    out_dir = "case_study/ehv_network"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "gb_ehv_network.json")
    with open(out_path, "w") as f:
        json.dump(case, f, indent=2)
    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"    Written to {out_path} ({size_mb:.1f} MB)")

    # ── 10. Write boundary-bus summary CSV ───────────────────────────
    csv_path = os.path.join(out_dir, "boundary_injections.csv")
    with open(csv_path, "w") as f:
        f.write("bus_label,vn_kv,n_boundary_trafos,equiv_p_mw,equiv_q_mvar,"
                "orig_p_demand_mw,orig_q_demand_mvar,"
                "total_p_demand_mw,total_q_demand_mvar\n")
        for bus_id in sorted(equiv_p_mw.keys()):
            vn = float(net.bus.at[bus_id, "vn_kv"])
            n_bt = sum(1 for t in boundary_trafos
                       if int(net.trafo.at[t, "hv_bus"]) == bus_id)
            load_mask = net.load["bus"] == bus_id
            orig_p = float(net.load.loc[load_mask, "p_mw"].sum()) if load_mask.any() else 0.0
            orig_q = float(net.load.loc[load_mask, "q_mvar"].sum()) if load_mask.any() else 0.0
            total_p = orig_p + equiv_p_mw[bus_id]
            total_q = orig_q + equiv_q_mvar[bus_id]
            f.write(f"{bus_id},{vn},{n_bt},{equiv_p_mw[bus_id]:.4f},"
                    f"{equiv_q_mvar[bus_id]:.4f},{orig_p:.4f},{orig_q:.4f},"
                    f"{total_p:.4f},{total_q:.4f}\n")
    print(f"    Boundary injection summary: {csv_path}")

    print("\n" + "=" * 70)
    print("EHV extraction complete.")
    print(f"  Full network:     2224 buses, 3207 branches")
    print(f"  Reduced network:  {len(bus_data)} buses, {len(branch_data)} branches")
    print(f"  Boundary buses:   {n_boundary_buses} (with equiv. injections)")
    print(f"  Measurements:     {n_meas} scalar equations")
    print("=" * 70)


def generate_measurements_ehv(net, ehv_buses, bus_to_seqidx, branch_data,
                               true_vm, true_va, base_mva,
                               equiv_p_mw, equiv_q_mvar):
    """Generate measurements for the reduced EHV network."""
    measurements = {
        "voltmeters": [], "ammeters": [],
        "wattmeters": [], "varmeters": [], "pmus": [],
    }
    ehv_idx_map = {b: i for i, b in enumerate(ehv_buses)}

    # ── Voltmeters at all EHV buses ──
    for i, bus_id in enumerate(ehv_buses):
        vm_true = true_vm[i]
        noise = np.random.normal(0, VOLTAGE_MAG_STD)
        measurements["voltmeters"].append({
            "label": f"V_{bus_id}",
            "bus": int(bus_id),
            "magnitude": vm_true + noise,
            "variance": VOLTAGE_MAG_STD ** 2,
        })

    # ── Bus injection wattmeters/varmeters ──
    # The "true" injection in the reduced network accounts for equiv. injections
    for i, bus_id in enumerate(ehv_buses):
        full_idx = bus_to_seqidx[bus_id]
        # Full-network injection (injection convention: positive = generated)
        p_inj_full = -float(net.res_bus.at[bus_id, "p_mw"]) / base_mva
        q_inj_full = -float(net.res_bus.at[bus_id, "q_mvar"]) / base_mva

        # The equivalent demand absorbs the flow that was going to distribution.
        # In the reduced network the net injection = full injection minus boundary flows.
        p_equiv = equiv_p_mw.get(bus_id, 0.0) / base_mva
        q_equiv = equiv_q_mvar.get(bus_id, 0.0) / base_mva
        p_inj_reduced = p_inj_full - p_equiv
        q_inj_reduced = q_inj_full - q_equiv

        noise_p = np.random.normal(0, ACTIVE_POWER_STD)
        noise_q = np.random.normal(0, REACTIVE_POWER_STD)
        measurements["wattmeters"].append({
            "label": f"P_inj_{bus_id}",
            "location": {"Bus": int(bus_id)},
            "active": p_inj_reduced + noise_p,
            "variance": ACTIVE_POWER_STD ** 2,
        })
        measurements["varmeters"].append({
            "label": f"Q_inj_{bus_id}",
            "location": {"Bus": int(bus_id)},
            "reactive": q_inj_reduced + noise_q,
            "variance": REACTIVE_POWER_STD ** 2,
        })

    # ── Branch flow wattmeters/varmeters (80% of retained lines) ──
    # Collect line branches with their original pandapower line index
    line_branches = []
    for br in branch_data:
        if br["tap_ratio"] == 1.0 and br["shift_angle"] == 0.0:
            line_branches.append(br)

    n_flow = int(0.8 * len(line_branches))
    if n_flow > 0:
        flow_indices = np.random.choice(len(line_branches), n_flow, replace=False)
        for fi in flow_indices:
            br = line_branches[fi]
            fb, tb = br["from_bus"], br["to_bus"]
            fi_ehv, ti_ehv = ehv_idx_map[fb], ehv_idx_map[tb]
            # Compute true power flow on this branch from the true state
            p_from, q_from = compute_branch_flow(
                br, true_vm[fi_ehv], true_va[fi_ehv],
                true_vm[ti_ehv], true_va[ti_ehv],
            )
            noise_p = np.random.normal(0, ACTIVE_POWER_STD)
            noise_q = np.random.normal(0, REACTIVE_POWER_STD)
            measurements["wattmeters"].append({
                "label": f"P_flow_br{br['label']}",
                "location": {"Branch": {"branch": br["label"], "end": "From"}},
                "active": p_from + noise_p,
                "variance": ACTIVE_POWER_STD ** 2,
            })
            measurements["varmeters"].append({
                "label": f"Q_flow_br{br['label']}",
                "location": {"Branch": {"branch": br["label"], "end": "From"}},
                "reactive": q_from + noise_q,
                "variance": REACTIVE_POWER_STD ** 2,
            })

    # ── Ammeters (50% of retained lines) ──
    n_current = int(0.5 * len(line_branches))
    if n_current > 0:
        current_indices = np.random.choice(len(line_branches), n_current, replace=False)
        for ci in current_indices:
            br = line_branches[ci]
            fb = br["from_bus"]
            fi_ehv = ehv_idx_map[fb]
            ti_ehv = ehv_idx_map[br["to_bus"]]
            p_from, q_from = compute_branch_flow(
                br, true_vm[fi_ehv], true_va[fi_ehv],
                true_vm[ti_ehv], true_va[ti_ehv],
            )
            s_from = np.sqrt(p_from**2 + q_from**2)
            i_pu = s_from / true_vm[fi_ehv] if true_vm[fi_ehv] > 0 else 0.0
            noise = np.random.normal(0, CURRENT_MAG_STD)
            measurements["ammeters"].append({
                "label": f"I_br{br['label']}",
                "branch": br["label"],
                "end": "From",
                "magnitude": max(i_pu + noise, 0.001),
                "variance": CURRENT_MAG_STD ** 2,
                "square": False,
            })

    # ── PMUs at ~10% of EHV buses ──
    n_pmu = max(1, int(0.10 * len(ehv_buses)))
    pmu_idx = np.random.choice(len(ehv_buses), n_pmu, replace=False)
    for pi in pmu_idx:
        bus_id = ehv_buses[pi]
        noise_m = np.random.normal(0, PMU_MAG_STD)
        noise_a = np.random.normal(0, PMU_ANG_STD)
        measurements["pmus"].append({
            "label": f"PMU_{bus_id}",
            "location": {"Bus": int(bus_id)},
            "magnitude": true_vm[pi] + noise_m,
            "angle": true_va[pi] + noise_a,
            "variance_magnitude": PMU_MAG_STD ** 2,
            "variance_angle": PMU_ANG_STD ** 2,
            "coordinate": "Rectangular",
            "correlated": False,
            "square": False,
        })

    return measurements


def compute_branch_flow(br, vi, theta_i, vj, theta_j):
    """Compute active/reactive power flow at the from-bus end of a π-model branch."""
    r, x = br["resistance"], br["reactance"]
    b_sh = br["susceptance"]
    g_sh = br["conductance"]
    t = br["tap_ratio"]
    phi = br["shift_angle"]

    z2 = r*r + x*x
    if z2 < 1e-20:
        return 0.0, 0.0
    g_s = r / z2
    b_s = -x / z2

    delta = theta_i - theta_j - phi
    cos_d = np.cos(delta)
    sin_d = np.sin(delta)

    p_from = (vi**2 / t**2) * (g_s + g_sh / 2) - (vi * vj / t) * (g_s * cos_d + b_s * sin_d)
    q_from = -(vi**2 / t**2) * (b_s + b_sh / 2) - (vi * vj / t) * (g_s * sin_d - b_s * cos_d)
    return p_from, q_from


if __name__ == "__main__":
    main()
