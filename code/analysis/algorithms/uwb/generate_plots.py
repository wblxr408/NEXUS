#!/usr/bin/env python3
"""Generate trajectory and error comparison plots for UWB algorithms."""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).parents[4]
sys.path.insert(0, str(REPO_ROOT / "code" / "analysis"))
sys.path.insert(0, str(REPO_ROOT / "simulation"))

from algorithms.router import AlgorithmRouter, register_default_algorithms
from generators.range_simulation import load_anchors, simulate_ranges


def main():
    register_default_algorithms()
    anchors = load_anchors(REPO_ROOT / "simulation" / "configs" / "uwb_simulation_config.yaml")

    t = np.linspace(0.0, 10.0, 100)
    positions = np.column_stack([
        0.5 + 3.0 * (1 - np.cos(2 * np.pi * t / 10)) / 2,
        0.5 + 3.5 * (1 - np.cos(4 * np.pi * t / 10)) / 2,
        np.full_like(t, 1.20),
    ])
    stamps = (t * int(1e9)).astype(np.int64)

    algorithm_names = [
        ("uwb.matlab.trilateration", "Trilateration"),
        ("uwb.matlab.multilateration", "Multilateration"),
        ("uwb.matlab.taylor", "Taylor Series"),
        ("uwb.matlab.ekf", "EKF"),
        ("uwb.matlab.ukf", "UKF"),
    ]

    fig_dir = REPO_ROOT / "outputs" / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    for sigma in [0.0, 0.05]:
        frames = simulate_ranges(anchors, positions, stamps, sigma_m=sigma, random_seed=42)

        fig_traj, ax_traj = plt.subplots(figsize=(8, 8))
        ax_traj.plot(positions[:, 0], positions[:, 1], 'k-', linewidth=2, label='Ground Truth')
        for name, label in algorithm_names:
            runner = AlgorithmRouter(name).spec.runner
            if hasattr(runner, "_state"):
                del runner._state
            estimates = []
            for frame in frames:
                result = AlgorithmRouter(name).run(observation_frame=frame)
                estimates.append(result.estimate)
            est = np.array(estimates)
            ax_traj.plot(est[:, 0], est[:, 1], linewidth=1, alpha=0.8, label=label)

        for anchor in anchors:
            ax_traj.plot(anchor.position_m[0], anchor.position_m[1], 'r^', markersize=10)
        ax_traj.set_title(f'Trajectory Comparison (sigma={sigma} m)')
        ax_traj.set_xlabel('X (m)'); ax_traj.set_ylabel('Y (m)')
        ax_traj.legend(); ax_traj.set_aspect('equal'); ax_traj.grid(True, alpha=0.3)
        traj_path = fig_dir / f'fig_uwb_trajectory_sigma{sigma:.2f}_v01.png'
        fig_traj.savefig(traj_path, dpi=150, bbox_inches='tight')
        plt.close(fig_traj)

        fig_err, ax_err = plt.subplots(figsize=(10, 5))
        for name, label in algorithm_names:
            runner = AlgorithmRouter(name).spec.runner
            if hasattr(runner, "_state"):
                del runner._state
            errors = []
            for index, frame in enumerate(frames):
                result = AlgorithmRouter(name).run(observation_frame=frame)
                if result.valid:
                    errors.append(np.linalg.norm(result.estimate - positions[index, :2]))
                else:
                    errors.append(np.nan)
            ax_err.plot(t, errors, label=label, alpha=0.8)
        ax_err.set_title(f'Position Error vs Time (sigma={sigma} m)')
        ax_err.set_xlabel('Time (s)'); ax_err.set_ylabel('Error (m)')
        ax_err.legend(); ax_err.grid(True, alpha=0.3)
        err_path = fig_dir / f'fig_uwb_error_sigma{sigma:.2f}_v01.png'
        fig_err.savefig(err_path, dpi=150, bbox_inches='tight')
        plt.close(fig_err)

    print(f'Plots saved to {fig_dir}')


if __name__ == "__main__":
    main()
