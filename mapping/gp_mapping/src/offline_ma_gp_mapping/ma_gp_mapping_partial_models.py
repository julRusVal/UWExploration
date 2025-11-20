#!/usr/bin/env python3
"""
Utility script for training a full-map baseline GP model plus N sub-models that
only observe a portion of the map. Inspired by ma_gp_mapping_example.py but
structured for repeatable experiments.
"""

import argparse
import os
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml

from gp_mapping.gp import SVGP


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train a baseline GP map and N partial sub-models.")
    parser.add_argument(
        "--dataset",
        required=True,
        help="Path to an Nx3 .npy file describing [x, y, z] survey points.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory where models, figures, and metadata will be stored.",
    )
    parser.add_argument(
        "--agent-count",
        type=int,
        default=2,
        help="Number of sub-models/agents to train.",
    )
    parser.add_argument(
        "--movement-axis",
        choices=["x", "y"],
        default="y",
        help="Mission movement axis; determines how sub-maps are partitioned.",
    )
    parser.add_argument(
        "--coverage-ratio",
        type=float,
        default=0.5,
        help="Fraction (0, 1] of each agent sub-map that is observable.",
    )
    parser.add_argument(
        "--n-inducing",
        type=int,
        default=400,
        help="Number of inducing points per GP model.",
    )
    parser.add_argument("--n-samples", type=int, default=1000)
    parser.add_argument("--max-iter", type=int, default=500)
    parser.add_argument("--learning-rate", type=float, default=1e-1)
    parser.add_argument("--rtol", type=float, default=1e-12)
    parser.add_argument("--n-window", type=int, default=2000)
    parser.add_argument(
        "--auto-tuning",
        action="store_true",
        help="Enable SVGP's automatic jitter tuning.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print SVGP training progress for each model.",
    )
    parser.add_argument(
        "--skip-plots",
        action="store_true",
        help="Skip generating gp.plot() figures to reduce runtime.",
    )
    return parser.parse_args()


def generate_agent_sub_maps(
    map_mins: np.ndarray,
    map_maxs: np.ndarray,
    agent_count: int,
    survey_points: np.ndarray,
    movement_axis: str = "y",
) -> List[np.ndarray]:
    """Split survey points into equal-width strips per agent."""
    if movement_axis.lower() == "y":
        dimension_ind = 0
    else:
        dimension_ind = 1

    map_min = map_mins[dimension_ind]
    map_max = map_maxs[dimension_ind]

    agent_span = (map_max - map_min) / agent_count
    boundaries = [map_min + i * agent_span for i in range(agent_count + 1)]

    agent_sub_maps: List[np.ndarray] = []

    for i in range(agent_count):
        mask = (survey_points[:, dimension_ind] >= boundaries[i]) & (
            survey_points[:, dimension_ind] < boundaries[i + 1]
        )
        agent_points = survey_points[mask]
        agent_sub_maps.append(agent_points)

    return agent_sub_maps


def _movement_dimension(movement_axis: str) -> int:
    return 1 if movement_axis.lower() == "y" else 0


def select_agent_portion(
    agent_points: np.ndarray, movement_axis: str, coverage_ratio: float
) -> Tuple[np.ndarray, float]:
    """Return subset of agent_points and the cutoff coordinate."""
    if coverage_ratio <= 0 or coverage_ratio > 1:
        raise ValueError("coverage_ratio must be within (0, 1].")
    dimension_ind = _movement_dimension(movement_axis)
    axis_values = agent_points[:, dimension_ind]
    axis_min = np.min(axis_values)
    axis_max = np.max(axis_values)
    cutoff = axis_min + coverage_ratio * (axis_max - axis_min)
    mask = axis_values <= cutoff
    return agent_points[mask], cutoff


def train_svgp_model(
    survey_points: np.ndarray,
    *,
    n_inducing: int,
    n_samples: int,
    max_iter: int,
    learning_rate: float,
    rtol: float,
    n_window: int,
    auto_tuning: bool,
    verbose: bool,
) -> Tuple[SVGP, np.ndarray]:
    """Train a SVGP on the provided survey points."""
    inputs = survey_points[:, 0:2]
    targets = survey_points[:, 2]

    gp = SVGP(n_inducing=n_inducing, batch_bins=1, inducing_bins=1)
    gp.fit(
        inputs,
        targets,
        covariances=None,
        n_samples=n_samples,
        max_iter=max_iter,
        learning_rate=learning_rate,
        rtol=rtol,
        n_window=n_window,
        auto=auto_tuning,
        verbose=verbose,
    )

    return gp, np.asarray(gp.loss)


def save_loss_plot(loss_array: np.ndarray, fname: str) -> None:
    fig, ax = plt.subplots(1)
    ax.plot(loss_array, "k-")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("ELBO")
    ax.set_yscale("log")
    plt.tight_layout()
    fig.savefig(fname, bbox_inches="tight", dpi=1000)
    plt.close(fig)


def persist_model_artifacts(
    gp: SVGP,
    loss: np.ndarray,
    output_dir: str,
    name: str,
    points: np.ndarray,
    *,
    skip_plots: bool,
) -> None:
    os.makedirs(output_dir, exist_ok=True)
    model_path = os.path.join(output_dir, f"{name}.pth")
    loss_path = os.path.join(output_dir, f"{name}_loss.npy")
    gp.save(model_path)
    np.save(loss_path, loss)

    if skip_plots:
        return

    x = points[:, 0]
    y = points[:, 1]
    post_name_complete = os.path.join(output_dir, f"{name}_post.npy")
    gp.save_posterior(
        1000, min(x), max(x), min(y), max(y), post_name_complete, verbose=False
    )

    inputs = points[:, 0:2]
    targets = points[:, 2]
    post_plot = os.path.join(output_dir, f"{name}.png")
    gp.plot(inputs, targets, post_plot, n=100, n_contours=100)
    loss_plot_name = os.path.join(output_dir, f"{name}_loss.png")
    save_loss_plot(loss, loss_plot_name)


def main() -> None:
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    survey_points = np.load(args.dataset)
    if survey_points.ndim != 2 or survey_points.shape[1] < 3:
        raise ValueError("Dataset must be an Nx3 array of [x, y, z] samples.")

    map_mins = np.min(survey_points, axis=0)
    map_maxs = np.max(survey_points, axis=0)

    baseline_dir = os.path.join(args.output_dir, "baseline")
    sub_model_dir = os.path.join(args.output_dir, "sub_models")
    os.makedirs(baseline_dir, exist_ok=True)
    os.makedirs(sub_model_dir, exist_ok=True)

    baseline_gp, baseline_loss = train_svgp_model(
        survey_points,
        n_inducing=args.n_inducing,
        n_samples=args.n_samples,
        max_iter=args.max_iter,
        learning_rate=args.learning_rate,
        rtol=args.rtol,
        n_window=args.n_window,
        auto_tuning=args.auto_tuning,
        verbose=args.verbose,
    )
    persist_model_artifacts(
        baseline_gp,
        baseline_loss,
        baseline_dir,
        "baseline",
        survey_points,
        skip_plots=args.skip_plots,
    )

    agent_maps = generate_agent_sub_maps(
        map_mins=map_mins,
        map_maxs=map_maxs,
        agent_count=args.agent_count,
        survey_points=survey_points,
        movement_axis=args.movement_axis,
    )

    metadata = {
        "dataset": os.path.abspath(args.dataset),
        "agent_count": args.agent_count,
        "movement_axis": args.movement_axis,
        "coverage_ratio": args.coverage_ratio,
        "n_inducing": args.n_inducing,
        "max_iter": args.max_iter,
        "learning_rate": args.learning_rate,
        "rtol": args.rtol,
        "n_window": args.n_window,
        "n_samples": args.n_samples,
        "auto_tuning": args.auto_tuning,
        "baseline_artifacts": {
            "directory": os.path.abspath(baseline_dir),
            "model": os.path.join(os.path.abspath(baseline_dir), "baseline.pth"),
        },
        "sub_models": [],
    }

    for agent_idx, agent_points in enumerate(agent_maps):
        if agent_points.size == 0:
            print(f"[WARN] No points assigned to agent {agent_idx}, skipping.")
            continue

        partial_points, cutoff = select_agent_portion(
            agent_points, args.movement_axis, args.coverage_ratio
        )
        if partial_points.size == 0:
            print(
                f"[WARN] coverage ratio removed all points for agent {agent_idx}, skipping."
            )
            continue

        agent_dir = os.path.join(sub_model_dir, f"agent_{agent_idx}")
        gp, loss = train_svgp_model(
            partial_points,
            n_inducing=args.n_inducing,
            n_samples=args.n_samples,
            max_iter=args.max_iter,
            learning_rate=args.learning_rate,
            rtol=args.rtol,
            n_window=args.n_window,
            auto_tuning=args.auto_tuning,
            verbose=args.verbose,
        )
        persist_model_artifacts(
            gp,
            loss,
            agent_dir,
            f"agent_{agent_idx}",
            partial_points,
            skip_plots=args.skip_plots,
        )
        metadata["sub_models"].append(
            {
                "agent_index": agent_idx,
                "full_point_count": int(agent_points.shape[0]),
                "portion_point_count": int(partial_points.shape[0]),
                "coverage_cutoff": float(cutoff),
                "artifacts": {
                    "directory": os.path.abspath(agent_dir),
                    "model": os.path.join(
                        os.path.abspath(agent_dir), f"agent_{agent_idx}.pth"
                    ),
                },
            }
        )

    metadata_path = os.path.join(args.output_dir, "summary.yaml")
    with open(metadata_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(metadata, handle)

    print("Baseline and partial models saved to", os.path.abspath(args.output_dir))


if __name__ == "__main__":
    main()
