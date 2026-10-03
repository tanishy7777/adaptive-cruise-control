"""Run from repository root: python python/run_experiments.py."""
import argparse
import csv
import json
import os
from pathlib import Path
os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parents[1] / ".mplconfig"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from acc import ROOT, COLUMNS, inputs, simulate, metrics

BLUE, ORANGE, GREEN = "#2255c7", "#e67929", "#168b75"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.alpha": .18, "figure.facecolor": "#fafbfd"})


def plot_scenario(x, s, path):
    t, v, vl, gap, desired, u, a, mode, present, ref, ttc = x.T
    active = present.astype(bool)
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True, layout="constrained")
    fig.suptitle(s["title"], x=.08, ha="left", fontsize=19, weight="bold")
    ax[0].plot(t, v*3.6, color=BLUE, label="Ego")
    ax[0].plot(t, np.where(active, vl*3.6, np.nan), color=ORANGE, label="Lead")
    ax[0].axhline(s["set_speed"]*3.6, color="#7b8497", ls=":", label="Set speed")
    ax[0].set_ylabel("Speed [km/h]")
    ax[1].plot(t, np.where(active, gap, np.nan), color=BLUE, label="Actual gap")
    ax[1].plot(t, desired, color=GREEN, ls="--", label="Desired gap")
    ax[1].axhline(0, color="#bd3b48", lw=1)
    ax[1].set_ylabel("Gap [m]")
    ax[2].plot(t, u, color=ORANGE, lw=1.2, label="Command")
    ax[2].plot(t, a, color=BLUE, label="Actual acceleration")
    ax[2].axhline(-4, color="#bd3b48", ls=":", label="Braking limit")
    ax[2].set_ylabel("Acceleration [m/s²]")
    ax[3].step(t, mode, where="post", color=GREEN, lw=1.8, label="Supervisor")
    ax[3].set_yticks([0, 1, 2], ["CRUISE", "FOLLOW", "EMERGENCY"])
    ax[3].set_ylim(-.3, 2.4)
    ax[3].set_xlabel("Time [s]")
    for axis in ax:
        axis.legend(loc="upper right", ncol=3, fontsize=9)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_table(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    out = args.output
    for sub in ["traces", "figures"]:
        (out/sub).mkdir(parents=True, exist_ok=True)
    p, scenarios = inputs()
    baseline = []
    for s in scenarios:
        x = simulate(s, p)
        baseline.append(metrics(x, s, p))
        np.savetxt(out/"traces"/(s["name"]+".csv"), x, delimiter=",",
                   header=",".join(COLUMNS), comments="", fmt="%.10g")
        plot_scenario(x, s, out/"figures"/(s["name"]+".png"))
    write_table(out/"baseline_metrics.csv", baseline)
    comparison = []
    for headway in [1., 1.5, 2.]:
        for s in scenarios:
            cfg = dict(p, headway=headway)
            comparison.append(metrics(simulate(s, cfg), s, cfg))
    write_table(out/"headway_metrics.csv", comparison)
    controllers = []
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, layout="constrained")
    for name, ki, kd in [("P", 0, 0), ("PI", p["ki"], 0), ("PID", p["ki"], p["kd"])]:
        cfg = dict(p, ki=ki, kd=kd)
        s = scenarios[2]
        x = simulate(s, cfg)
        controllers.append(dict(controller=name, **metrics(x, s, cfg)))
        axes[0].plot(x[:, 0], x[:, 1]*3.6, label=name)
        axes[1].plot(x[:, 0], x[:, 3]-x[:, 4], label=name)
    axes[0].plot(x[:, 0], x[:, 2]*3.6, "k:", label="Lead")
    axes[0].set_ylabel("Speed [km/h]")
    axes[1].set_ylabel("Gap − desired [m]")
    axes[1].set_xlabel("Time [s]")
    for ax in axes:
        ax.legend(ncol=4)
    fig.suptitle("P / PI / PID · identical supervisor and actuator constraints", fontsize=16)
    fig.savefig(out/"figures/controller_comparison.png", dpi=160)
    plt.close(fig)
    write_table(out/"controller_metrics.csv", controllers)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, layout="constrained")
    for headway in [1., 1.5, 2.]:
        x = simulate(scenarios[2], dict(p, headway=headway))
        axes[0].plot(x[:, 0], x[:, 3], label=f"{headway:.1f} s")
        axes[1].plot(x[:, 0], x[:, 6], label=f"{headway:.1f} s")
    axes[0].set_ylabel("Actual gap [m]")
    axes[1].set_ylabel("Acceleration [m/s²]")
    axes[1].set_xlabel("Time [s]")
    for ax in axes:
        ax.legend(title="Headway", ncol=3)
    fig.suptitle("Headway sensitivity · 80 → 50 km/h lead slowdown", fontsize=17)
    fig.savefig(out/"figures/headway_comparison.png", dpi=160)
    plt.close(fig)
    unavoidable = dict(scenarios[3], name="unavoidable_collision", title="Infeasible initial gap",
                       ego_speed=25, lead_speed=0, initial_gap=6, duration=8, events=[[0, 0]],
                       settle_after=0)
    limitation = metrics(simulate(unavoidable, p), unavoidable, p)
    (out/"limitation.json").write_text(json.dumps(limitation, indent=2)+"\n")
    manifest = {"engine": "Python reference", "sample_time_s": p["dt"],
                "parameters": p, "baseline": baseline, "limitation": limitation}
    (out/"summary.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps(baseline, indent=2))


if __name__ == "__main__":
    main()
