from typing import List

import open3d as o3d
import torch
import matplotlib.pyplot as plt
import numpy as np

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