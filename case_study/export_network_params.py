"""
Export pandapower GB network parameters to CSV files for all voltage levels.

Creates case_study/network_params/ with:
  - bus.csv          : All buses with vn_kv, zone, type, in_service, etc.
  - line.csv         : All lines with from_bus, to_bus, r/x/c, length, voltage level
  - trafo.csv        : All transformers with hv/lv buses, ratings, tap, etc.
  - gen.csv          : Generators
  - load.csv         : Loads
  - shunt.csv        : Shunts
  - ext_grid.csv     : External grid (slack)
  - voltage_levels.csv : Summary of voltage levels and element counts

Run from project root:
  python case_study/export_network_params.py
"""

import os
import sys

# Avoid matplotlib cache issues when importing pandapower
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib_cache")

import pandas as pd
import pandapower.networks as pn


def main():
    print("Loading pandapower GB network...")
    net = pn.GBnetwork()

    out_dir = "case_study/network_params"
    os.makedirs(out_dir, exist_ok=True)
    print(f"Output directory: {out_dir}")

    # ── Bus: add voltage level column (already vn_kv)
    bus_df = net.bus.copy()
    bus_df["voltage_level_kv"] = bus_df["vn_kv"]
    bus_df.to_csv(f"{out_dir}/bus.csv", index=True, index_label="bus_id")
    print(f"  bus.csv: {len(bus_df)} rows")

    # ── Line: add voltage level from from_bus
    line_df = net.line.copy()
    vn_map = net.bus["vn_kv"].to_dict()
    line_df["vn_from_kv"] = line_df["from_bus"].map(vn_map)
    line_df["vn_to_kv"] = line_df["to_bus"].map(vn_map)
    line_df["voltage_level_kv"] = line_df["vn_from_kv"]  # primary voltage
    line_df.to_csv(f"{out_dir}/line.csv", index=True, index_label="line_id")
    print(f"  line.csv: {len(line_df)} rows")

    # ── Trafo: has vn_hv_kv, vn_lv_kv
    trafo_df = net.trafo.copy()
    trafo_df["voltage_level_hv_kv"] = trafo_df["vn_hv_kv"]
    trafo_df["voltage_level_lv_kv"] = trafo_df["vn_lv_kv"]
    trafo_df.to_csv(f"{out_dir}/trafo.csv", index=True, index_label="trafo_id")
    print(f"  trafo.csv: {len(trafo_df)} rows")

    # ── Gen
    gen_df = net.gen.copy()
    gen_df["vn_kv"] = gen_df["bus"].map(vn_map)
    gen_df.to_csv(f"{out_dir}/gen.csv", index=True, index_label="gen_id")
    print(f"  gen.csv: {len(gen_df)} rows")

    # ── Load
    load_df = net.load.copy()
    load_df["vn_kv"] = load_df["bus"].map(vn_map)
    load_df.to_csv(f"{out_dir}/load.csv", index=True, index_label="load_id")
    print(f"  load.csv: {len(load_df)} rows")

    # ── Shunt
    shunt_df = net.shunt.copy()
    shunt_df["vn_kv"] = shunt_df["bus"].map(vn_map)
    shunt_df.to_csv(f"{out_dir}/shunt.csv", index=True, index_label="shunt_id")
    print(f"  shunt.csv: {len(shunt_df)} rows")

    # ── Ext grid
    ext_df = net.ext_grid.copy()
    ext_df["vn_kv"] = ext_df["bus"].map(vn_map)
    ext_df.to_csv(f"{out_dir}/ext_grid.csv", index=True, index_label="ext_grid_id")
    print(f"  ext_grid.csv: {len(ext_df)} rows")

    # ── Voltage levels summary
    vn_levels = sorted(net.bus["vn_kv"].unique())
    summary_rows = []
    for vn in vn_levels:
        n_bus = (net.bus["vn_kv"] == vn).sum()
        n_line = (line_df["vn_from_kv"] == vn).sum() + (line_df["vn_to_kv"] == vn).sum()
        n_trafo_hv = (trafo_df["vn_hv_kv"] == vn).sum()
        n_trafo_lv = (trafo_df["vn_lv_kv"] == vn).sum()
        n_gen = (gen_df["vn_kv"] == vn).sum()
        n_load = (load_df["vn_kv"] == vn).sum()
        n_shunt = (shunt_df["vn_kv"] == vn).sum()
        summary_rows.append({
            "voltage_level_kv": vn,
            "n_buses": n_bus,
            "n_lines_touching": n_line,
            "n_trafos_hv_side": n_trafo_hv,
            "n_trafos_lv_side": n_trafo_lv,
            "n_generators": n_gen,
            "n_loads": n_load,
            "n_shunts": n_shunt,
        })
    summary_df = pd.DataFrame(summary_rows)
    summary_df.to_csv(f"{out_dir}/voltage_levels.csv", index=False)
    print(f"  voltage_levels.csv: {len(summary_df)} voltage levels")

    print("\nDone. CSV files written to", out_dir)


if __name__ == "__main__":
    main()
