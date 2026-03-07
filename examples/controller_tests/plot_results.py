#!/usr/bin/env python3
"""Load controller .npz results, print summary metrics, and generate plots.

Usage examples:
  /usr/bin/python3 examples/controller_tests/plot_results.py \
      --file examples/controller_tests/results/cascaded_pd_statistics.npz

  /usr/bin/python3 examples/controller_tests/plot_results.py \
      --file examples/controller_tests/results/nonlinear_statistics.npz \
      --save-prefix examples/controller_tests/results/nonlinear
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def _required_keys(data: np.lib.npyio.NpzFile, keys: list[str]) -> None:
    missing = [k for k in keys if k not in data.files]
    if missing:
        raise KeyError(f"Missing required keys in npz: {missing}. Found keys: {data.files}")


def _print_summary(data: np.lib.npyio.NpzFile, last_n: int) -> None:
    t = data["time"]
    p = data["p"]
    p_ref = data["desired_p"]
    e_p = data["ep"]

    n = min(last_n, len(t))

    print("=== Result Summary ===")
    print(f"samples: {len(t)}")
    print(f"duration [s]: {float(t[-1]) if len(t) else 0.0:.3f}")
    print(f"last position [x y z]: {p[-1]}")
    print(f"last desired  [x y z]: {p_ref[-1]}")
    print(f"last error    [x y z]: {e_p[-1]}")
    print(f"mean abs error over last {n} samples [x y z]: {np.mean(np.abs(e_p[-n:]), axis=0)}")

    if "eR" in data.files:
        e_r = data["eR"]
        print(f"mean abs attitude error over last {n} samples [x y z]: {np.mean(np.abs(e_r[-n:]), axis=0)}")

    if "yaw_rate" in data.files:
        yaw_rate = data["yaw_rate"]
        print(f"last yaw rate [rad/s]: {float(yaw_rate[-1]) if len(yaw_rate) else 0.0:.4f}")
        print(f"mean abs yaw rate over last {n} samples [rad/s]: {float(np.mean(np.abs(yaw_rate[-n:]))) if len(yaw_rate) else 0.0:.4f}")


def _plot(data: np.lib.npyio.NpzFile, title: str, save_prefix: str | None) -> None:
    t = data["time"]
    p = data["p"]
    p_ref = data["desired_p"]
    e_p = data["ep"]

    fig1, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(t, p[:, 0], label="x")
    axs[0].plot(t, p[:, 1], label="y")
    axs[0].plot(t, p[:, 2], label="z")
    axs[0].plot(t, p_ref[:, 0], "--", label="x_ref")
    axs[0].plot(t, p_ref[:, 1], "--", label="y_ref")
    axs[0].plot(t, p_ref[:, 2], "--", label="z_ref")
    axs[0].set_ylabel("Position [m]")
    axs[0].set_title(title)
    axs[0].grid(True)
    axs[0].legend(ncol=3)

    axs[1].plot(t, e_p[:, 0], label="e_x")
    axs[1].plot(t, e_p[:, 1], label="e_y")
    axs[1].plot(t, e_p[:, 2], label="e_z")
    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Position Error [m]")
    axs[1].grid(True)
    axs[1].legend(ncol=3)

    fig1.tight_layout()

    fig2 = plt.figure(figsize=(7, 6))
    ax2 = fig2.add_subplot(111)
    ax2.plot(p[:, 0], p[:, 1], label="actual XY")
    ax2.plot(p_ref[:, 0], p_ref[:, 1], "--", label="desired XY")
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    ax2.set_title(f"Top View XY: {title}")
    ax2.grid(True)
    ax2.axis("equal")
    ax2.legend()
    fig2.tight_layout()

    fig3 = None
    if "yaw_rate" in data.files:
        yaw_rate = data["yaw_rate"]
        yaw_ref = data["desired_yaw_rate"] if "desired_yaw_rate" in data.files else None
        yaw_body_ref = data["desired_body_z_rate"] if "desired_body_z_rate" in data.files else None

        fig3 = plt.figure(figsize=(10, 4.5))
        ax3 = fig3.add_subplot(111)
        ax3.plot(t, yaw_rate, label="w_z (actual)")
        if yaw_ref is not None:
            ax3.plot(t, yaw_ref, "--", label="yaw_rate_ref")
        if yaw_body_ref is not None:
            ax3.plot(t, yaw_body_ref, ":", label="body_z_rate_ref")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Yaw Rate [rad/s]")
        ax3.set_title(f"Yaw Rate: {title}")
        ax3.grid(True)
        ax3.legend()
        fig3.tight_layout()

    if save_prefix:
        save_base = Path(save_prefix)
        save_base.parent.mkdir(parents=True, exist_ok=True)
        fig1_path = str(save_base) + "_time_series.png"
        fig2_path = str(save_base) + "_xy.png"
        fig1.savefig(fig1_path, dpi=150)
        fig2.savefig(fig2_path, dpi=150)
        print(f"saved: {fig1_path}")
        print(f"saved: {fig2_path}")
        if fig3 is not None:
            fig3_path = str(save_base) + "_yaw_rate.png"
            fig3.savefig(fig3_path, dpi=150)
            print(f"saved: {fig3_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read and visualize Pegasus controller .npz results")
    parser.add_argument("--file", required=True, help="Path to .npz results file")
    parser.add_argument(
        "--last-n",
        type=int,
        default=200,
        help="Number of last samples used for average error summary (default: 200)",
    )
    parser.add_argument(
        "--save-prefix",
        default=None,
        help="If set, saves plots as '<prefix>_time_series.png' and '<prefix>_xy.png'",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open plot windows (useful for headless runs)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    file_path = Path(args.file)

    if not file_path.exists():
        raise FileNotFoundError(f"Results file not found: {file_path}")

    data = np.load(file_path)
    _required_keys(data, ["time", "p", "desired_p", "ep"])

    _print_summary(data, max(args.last_n, 1))
    _plot(data, title=file_path.name, save_prefix=args.save_prefix)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
