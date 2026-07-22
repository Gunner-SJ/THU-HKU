#!/usr/bin/env python3
"""Plot bike_balance_executor CSV response for pole-debug sessions.

Does not change control. Reads existing CSV columns written by balance_executor.

Usage:
  ros2 run bike_controller plot_bike_log --latest
  ros2 run bike_controller plot_bike_log --csv ~/bike_logs/pole/bike_mit_log_....csv
"""

from __future__ import annotations

import argparse
import glob
import os
import sys


def _latest_csv(log_dir: str) -> str:
    pattern = os.path.join(os.path.expanduser(log_dir), "bike_mit_log_*.csv")
    files = sorted(glob.glob(pattern), key=os.path.getmtime)
    if not files:
        raise FileNotFoundError(f"No CSV matching {pattern}")
    return files[-1]


def plot_csv(csv_path: str, out_path: str | None = None) -> str:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    csv_path = os.path.expanduser(csv_path)
    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    if data.size == 0:
        raise ValueError(f"Empty CSV: {csv_path}")
    if data.ndim == 0:
        data = np.array([data])

    t = np.asarray(data["t"], dtype=float)
    roll = np.asarray(data["roll_deg"], dtype=float)
    roll_rate = np.asarray(data["roll_rate_dps"], dtype=float)
    steer = np.asarray(data["steer_deg"], dtype=float)
    steer_tgt = np.asarray(data["steer_target_deg"], dtype=float)
    steer_rate = np.asarray(data["steer_rate_ref_dps"], dtype=float)
    speed = np.asarray(data["speed_ms"], dtype=float)
    speed_ref = np.asarray(data["speed_ref_ms"], dtype=float)
    sat = np.asarray(data["saturated"], dtype=float)

    fig, axes = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
    fig.suptitle(f"Bike response\n{os.path.basename(csv_path)}", fontsize=12)

    ax = axes[0]
    ax.plot(t, roll, color="#1f77b4", lw=1.2, label="roll")
    ax.set_ylabel("Roll [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[1]
    ax.plot(t, roll_rate, color="#ff7f0e", lw=1.0, label="roll_rate")
    ax.set_ylabel("Roll rate [deg/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[2]
    ax.plot(t, steer, color="#2ca02c", lw=1.2, label="steer meas")
    ax.plot(t, steer_tgt, color="#d62728", lw=1.0, ls="--", label="steer tgt")
    ax.set_ylabel("Steer [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[3]
    ax.plot(t, steer_rate, color="#9467bd", lw=1.0, label="steer_rate cmd")
    ax.set_ylabel("Steer rate [deg/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[4]
    ax.plot(t, speed, color="#8c564b", lw=1.2, label="speed meas")
    ax.plot(t, speed_ref, color="#7f7f7f", lw=1.0, ls="--", label="speed ref")
    if np.any(sat > 0):
        ax2 = ax.twinx()
        ax2.fill_between(t, 0, sat, color="#e377c2", alpha=0.25, label="saturated")
        ax2.set_ylabel("sat")
        ax2.set_ylim(-0.05, 1.2)
        ax2.legend(loc="upper left", fontsize=8)
    ax.set_ylabel("Speed [m/s]")
    ax.set_xlabel("t [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + "_response.png"
    else:
        out_path = os.path.expanduser(out_path)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def _open_image(path: str) -> None:
    """Open PNG with the desktop default viewer (best-effort)."""
    import subprocess

    path = os.path.abspath(path)
    for cmd in (
        ["xdg-open", path],
        ["gio", "open", path],
        ["eog", path],
    ):
        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            print(f"[plot_bike_log] opened: {path}")
            return
        except FileNotFoundError:
            continue
    print(f"[plot_bike_log] could not auto-open (no viewer); see {path}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    default_log = os.path.expanduser("~/ws_ros2/bike_response_logs")
    p = argparse.ArgumentParser(description="Plot bike CSV response curves")
    p.add_argument("--csv", default="", help="CSV path")
    p.add_argument("--log-dir", default=default_log, help="Log directory")
    p.add_argument("--latest", action="store_true", help="Plot newest bike_mit_log_*.csv")
    p.add_argument("--out", default="", help="Output PNG path (optional)")
    p.add_argument("--open", action="store_true", help="Open PNG after saving")
    args = p.parse_args(argv)

    try:
        if args.csv:
            csv_path = args.csv
        else:
            csv_path = _latest_csv(args.log_dir)
        out = plot_csv(csv_path, args.out or None)
    except Exception as exc:
        print(f"[plot_bike_log] FAILED: {exc}", file=sys.stderr)
        return 1

    print(f"[plot_bike_log] saved: {out}")
    print(f"[plot_bike_log] source: {os.path.expanduser(csv_path)}")
    if args.open:
        _open_image(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
