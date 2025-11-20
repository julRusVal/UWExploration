from typing import List, Optional

import open3d as o3d
import numpy as np
import torch

import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.tri import Triangulation


import gp_mapping.gp as gp


def plot_3d_point_cloud(points: np.ndarray, title: str = "3D Point Cloud", color: List[float] = [0, 0, 1]) -> None:
    """
    Plots a 3D point cloud using Open3D.

    Parameters:
    points (numpy.ndarray): A 2D array of shape (n_points, 3) representing the 3D coordinates of the points.
    title (str): The title of the plot.
    color (list): The color of the points in RGB format.
    """
    if not isinstance(points, np.ndarray):
        raise ValueError("Points should be a numpy array")
    if points.shape[1] != 3:
        raise ValueError("Points array must have shape (n_points, 3)")
    
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.paint_uniform_color(color)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title)
    vis.add_geometry(point_cloud)
    vis.run()
    vis.destroy_window()

def plot_complete(svgp_model: gp.SVGP, inputs, targets, fname, n=80, n_contours=50):
    '''
    Plots:
        ax[0]: raw inputs and targets,
        ax[1]: posterior predictive mean,
        ax[2]: posterior predictive variance
    inputs: (n,2) numpy array of inputs
    output: (n,) numpy array of targets
    fname: path to save plot at (extension determines file type, e.g. .png or .pdf)
    n: determines n² number of sampling locations to plot GP posterior
    n_contours: number of contours to show output magnitude with
    '''

    # sanity
    assert inputs.shape[0] == targets.shape[0]
    assert inputs.shape[1] == 2

    # toggle evaluation mode
    svgp_model.likelihood.eval()
    svgp_model.eval()
    torch.cuda.empty_cache()

    # posterior sampling locations
    inputsg = [
        np.linspace(min(inputs[:,0]), max(inputs[:,0]), n),
        np.linspace(min(inputs[:,1]), max(inputs[:,1]), n)
    ]
    inputst = np.meshgrid(*inputsg)
    s = inputst[0].shape
    inputst = [_.flatten() for _ in inputst]
    inputst = np.vstack(inputst).transpose()
    inputst = torch.from_numpy(inputst).to(svgp_model.device).float()

    # sample
    with torch.no_grad():
        outputs = svgp_model(inputst)
        outputs = svgp_model.likelihood(outputs)
        mean = outputs.mean.cpu().numpy().reshape(s)
        variance = outputs.variance.cpu().numpy().reshape(s)

    # plot raw, mean, and variance
    levels = np.linspace(min(targets), max(targets), n_contours)
    fig, ax = plt.subplots(3, sharex=True, sharey=True)
    cr = ax[0].scatter(inputs[:,0], inputs[:,1], c=targets, cmap='viridis', s=0.4, edgecolors='none')
    # cm = ax[1].contourf(*inputsg, mean, levels=n_contours)
    cm = ax[1].contourf(*inputsg, mean, levels=levels)
    cv = ax[2].contourf(*inputsg, variance, levels=n_contours)
    indpts = svgp_model.variational_strategy.inducing_points.data.cpu().numpy()
    ax[2].plot(indpts[:,0], indpts[:,1], 'ko', markersize=1, alpha=0.2)

    # colorbars
    fig.colorbar(cr, ax=ax[0])
    fig.colorbar(cm, ax=ax[1])
    fig.colorbar(cv, ax=ax[2])

    # formatting
    ax[0].set_aspect('equal')
    ax[0].set_title('Raw data')
    ax[0].set_ylabel('$y~[m]$')
    ax[1].set_aspect('equal')
    ax[1].set_title('Mean')
    ax[1].set_ylabel('$y~[m]$')
    ax[2].set_aspect('equal')
    ax[2].set_title('Variance')
    ax[2].set_xlabel('$x~[m]$')
    ax[2].set_ylabel('$y~[m]$')
    plt.tight_layout()

    # save
    fig.savefig(fname, bbox_inches='tight', dpi=1000)

    # Free up GPU mem
    del inputst
    torch.cuda.empty_cache()

def plot_loss(svgp_model: gp.SVGP, fname: str) -> None:

    # plot
    fig, ax = plt.subplots(1)
    ax.plot(svgp_model.loss, 'k-')

    # format
    ax.set_xlabel('Iteration')
    ax.set_ylabel('ELBO')
    #ax.set_yscale('log')
    plt.tight_layout()

    # save
    fig.savefig(fname, bbox_inches='tight', dpi=1000)

def plot_two_arrays(arr1, arr2, label1="Array 1", label2="Array 2", xlabel="X-axis", ylabel="Y-axis", title="Plot of Two Arrays"):
    """
    Plots two (N,) arrays on the same figure with labels.

    Parameters:
    - arr1: 1D NumPy array for the first dataset.
    - arr2: 1D NumPy array for the second dataset.
    - label1: Label for the first dataset (default is "Array 1").
    - label2: Label for the second dataset (default is "Array 2").
    - xlabel: Label for the x-axis.
    - ylabel: Label for the y-axis.
    - title: Title of the plot.
    """
    # Create the x values for the arrays, assuming they're indices
    x = np.arange(len(arr1))

    # Plot both arrays
    plt.plot(x, arr1, label=label1)
    plt.plot(x, arr2, label=label2)

    # Set labels and title
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)

    # Add a legend
    plt.legend()

    # Show the plot
    plt.show()

def plot_survey_and_inducing_points(survey_points: np.ndarray,
                                    inducing_points: Optional[np.ndarray] = None,
                                    output_path: Optional[str] = None,
                                    title: Optional[str] = None) -> None:
    """
    Create a simple mesh-style visualization of survey points with optional inducing points.

    Parameters
    ----------
    survey_points : np.ndarray
        Array of shape (N, 3) containing [x, y, z] samples.
    inducing_points : np.ndarray, optional
        Array of shape (M, 2) containing [x, y] inducing points to overlay.
    output_path : str, optional
        Path to save the plot if provided. When None, the figure is shown interactively.
    title : str, optional
        Custom plot title.
    """
    survey_points = np.asarray(survey_points)
    if survey_points.ndim != 2 or survey_points.shape[1] < 3:
        raise ValueError("survey_points must be an (N, 3) array.")

    x, y, z = survey_points[:, 0], survey_points[:, 1], survey_points[:, 2]
    fig, ax = plt.subplots(figsize=(7, 6))

    # tri = Triangulation(x, y)
    # ax.triplot(tri, color="#cccccc", linewidth=0.1, alpha=0.7)
    # scatter = ax.scatter(x, y, c=z, cmap='jet', s=1, edgecolors='none')
    scatter = ax.scatter(x, y, c=z, cmap="viridis", s=8, edgecolors="none")
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label("Survey value")

    if inducing_points is not None:
        inducing_points = np.asarray(inducing_points)
        if inducing_points.ndim != 2 or inducing_points.shape[1] != 2:
            raise ValueError("inducing_points must be an (M, 2) array.")
        ax.scatter(
            inducing_points[:, 0],
            inducing_points[:, 1],
            s=35,
            facecolors="none",
            edgecolors="red",
            linewidths=1.0,
            label="Inducing points",
        )
        ax.legend(loc="upper right")

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title or "Survey Map with Inducing Points")
    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, dpi=300)
        plt.close(fig)
    else:
        plt.show()