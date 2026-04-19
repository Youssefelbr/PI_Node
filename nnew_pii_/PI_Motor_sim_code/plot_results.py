#!/usr/bin/env python3
"""
plot_results.py — HIL test result visualiser

Usage:
    python3 plot_results.py                        # plots the latest CSV in logs/
    python3 plot_results.py logs/log_TC_001_*.csv  # plots a specific file
    python3 plot_results.py logs/log_TC_001.csv logs/log_TC_002.csv  # overlay several

Dependencies:
    pip install matplotlib pandas
"""

import sys
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def find_latest_csv(logs_dir="logs"):
    files = glob.glob(os.path.join(logs_dir, "*.csv"))
    if not files:
        print(f"No CSV files found in {logs_dir}/")
        sys.exit(1)
    return [max(files, key=os.path.getmtime)]


def plot_csv(ax_speed, ax_cmd, path, label_suffix=""):
    df = pd.read_csv(path)
    t = df["t_ms"] / 1000.0   # seconds

    tc_name = os.path.basename(path).split("_log_")[-1].replace(".csv", "")
    tc_id   = os.path.basename(path).split("_")[1]  # e.g. TC_001

    # ── speed subplot ──────────────────────────────────────────────
    ax_speed.step(t, df["phase_vset"], where="post",
                  linestyle="--", linewidth=1.5,
                  label=f"VSET {label_suffix}")
    ax_speed.plot(t, df["vv_model"], linewidth=1.8,
                  label=f"VV model {label_suffix}")
    ax_speed.plot(t, df["vv_sent"], linewidth=1.0, alpha=0.5,
                  label=f"VV sent (uint16) {label_suffix}")

    # mark phase boundaries (where VSET changes)
    vset_changes = df.index[df["phase_vset"].diff().fillna(0) != 0].tolist()
    for idx in vset_changes:
        ax_speed.axvline(t.iloc[idx], color="gray", linestyle=":", linewidth=0.8)

    # ── cmd subplot ─────────────────────────────────────────────────
    ax_cmd.plot(t, df["cmd"], linewidth=1.5, label=f"CMD {label_suffix}")
    ax_cmd.axhline( 500, color="red",  linestyle="--", linewidth=0.8, alpha=0.6)
    ax_cmd.axhline(-500, color="red",  linestyle="--", linewidth=0.8, alpha=0.6)
    ax_cmd.axhline(   0, color="gray", linestyle="-",  linewidth=0.5, alpha=0.4)

    for idx in vset_changes:
        ax_cmd.axvline(t.iloc[idx], color="gray", linestyle=":", linewidth=0.8)

    return tc_id


def main():
    paths = sys.argv[1:] if len(sys.argv) > 1 else find_latest_csv()

    n = len(paths)
    fig, axes = plt.subplots(
        2, n,
        figsize=(6 * n, 7),
        sharex="col",
        squeeze=False,
    )
    fig.suptitle("HIL Test Results — Speed Regulator", fontsize=13, fontweight="bold")

    for col, path in enumerate(paths):
        ax_speed = axes[0][col]
        ax_cmd   = axes[1][col]

        tc_id = plot_csv(ax_speed, ax_cmd, path)

        # formatting
        ax_speed.set_title(tc_id, fontsize=11)
        ax_speed.set_ylabel("Speed (km/h)")
        ax_speed.legend(fontsize=8, loc="lower right")
        ax_speed.grid(True, alpha=0.3)
        ax_speed.yaxis.set_minor_locator(ticker.AutoMinorLocator())

        ax_cmd.set_ylabel("CMD (accel)")
        ax_cmd.set_xlabel("Time (s)")
        ax_cmd.legend(fontsize=8, loc="upper right")
        ax_cmd.grid(True, alpha=0.3)
        ax_cmd.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        ax_cmd.set_ylim(-600, 600)

    plt.tight_layout()

    # save alongside the first CSV
    out_dir = os.path.dirname(paths[0]) or "."
    out_path = os.path.join(out_dir, "plot_" + os.path.basename(paths[0]).replace(".csv", ".png"))
    plt.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")

    plt.show()


if __name__ == "__main__":
    main()
