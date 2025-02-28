#!/usr/bin/env python3

# from process import process
import os
from gp import SVGP
import numpy as np
from optparse import OptionParser
import numpy as np
import open3d as o3d


def train_svgp(gp_inputs_type, survey_name, inducing_bins=1, batch_bins=1, return_info=False):
    """
    Accepts the file name of either compressed np arrays, .npz, or an np array, .npy.


    Parameters:
    - survey_name: The name of the file containing the survey data.
    - inducing_bins: The number of inducing bins to use in the GP.
    - batch_bins: The number of batch bins to use in the GP.
    - return_info: A boolean flag to return the [GP object, inputs, targets, covs] or not.
    """
    if not os.path.isfile(survey_name):
        raise ValueError("The provided file does not exist.")
    
    print(f"Loading {survey_name}")
    cloud = np.load(survey_name)

    out_path = os.path.dirname(survey_name)
    print(f"Output path {out_path}")

    # Check for proper format:
    # The input is either a compress .npz that contains "points" and "convs"
    # or is a raw point cloud
    if hasattr(cloud, 'keys'):
        npz_keys = list(cloud.keys())
        if 'points' in npz_keys:
            points = cloud['points']
        elif 'beams' in npz_keys:
            points = cloud['beams']
        else:
            raise ValueError("The provided .npz file does not contain 'points' or 'beams'.")
        # Check if convs is present
        if 'covs' not in cloud.keys():
            gp_inputs_type = 'di'
    else:
        points = cloud
        gp_inputs_type = 'di'
    
    # pcd = o3d.io.read_point_cloud(survey_name)
    # pcd = pcd.uniform_down_sample(every_k_points=3)
    # points = np.asarray(pcd.points)
    inputs = points[:, [0,1]]
    print("Inputs ", inputs.shape)
    targets = points[:,2]
    print("Targets ", targets.shape)
    
    print(gp_inputs_type)
    if gp_inputs_type == 'di':
        name = "svgp_di"
        covariances = None
    else:
        ## UI
        covariances = cloud['covs']
        print("Covariances ", covariances.shape)
        name = "svgp_ui"    

    # initialise GP with 1000 inducing points
    gp = SVGP(400, inducing_bins=inducing_bins, batch_bins=batch_bins)
    gp.fit(inputs, targets, covariances=covariances, n_samples=1000, 
            max_iter=1000, learning_rate=1e-1, rtol=1e-12, n_window=2000, 
            auto=False, verbose=True)
   
    # Save GP
    print("Saving trained GP")
    gp_name = os.path.join(out_path, name + '.pth')
    gp.save(gp_name)

    # save figures
    print("Plotting results")
    gp.plot(inputs, targets, name + '.png',
             n=100, n_contours=100)
    loss_plot_name = os.path.join(out_path, name + '_loss.png')
    gp.plot_loss(loss_plot_name)
    
    # Save loss for tunning of stopping criterion
    loss_data_name = os.path.join(out_path, name + '_loss.npy')
    np.save(loss_data_name, np.asarray(gp.loss))

    # Save posterior
    print("Saving posterior")
    x = inputs[:,0]
    y = inputs[:,1]
    post_data_name = os.path.join(out_path, name + '_post.npy')
    gp.save_posterior(1000, min(x), max(x), min(y), max(y), 
                      post_data_name, verbose=False)

    if return_info:
        return {
            "gp": gp,
            "inputs": inputs,
            "targets": targets,
            "covariances": covariances
        }

    return

def trace_kernel(gp_path):

    gp = SVGP.load(1000, gp_path)
    gp.likelihood.eval()
    gp.eval()

    print("Kernel")
    ip = gp.variational_strategy.inducing_points.data
    print(np.trace(gp.cov(ip).cpu().numpy()))


def load_plot(gp_path, survey_name, trajectory_name):
    gp = SVGP.load(400, gp_path)
    gp.likelihood.eval()
    gp.eval()

    pcd = o3d.io.read_point_cloud(survey_name)
    pcd = pcd.uniform_down_sample(every_k_points=3)
    points = np.asarray(pcd.points)

    inputs = points[:, [0,1]]
    print("Inputs ", inputs.shape)
    targets = points[:,2]
    print("Targets ", targets.shape)

    track_file = np.load(trajectory_name)
    track = track_file["track_position"]

    name = "svgp_di"
    gp.plot(inputs, targets, name + '.png',
             n=100, n_contours=100, track=track)


if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--gp_inputs", dest="gp_inputs",
                  default="di", help="di or ui inputs for training.")
    parser.add_option("--survey_name", dest="survey_name",
                  default="", help="Name for folder to store results.")
    parser.add_option("--gp", dest="gp",
                  default="", help="GP already trained")
    parser.add_option("--trajectory", dest="track",
                  default="", help="AUV track")

    (options, args) = parser.parse_args()
    gp_inputs_type = options.gp_inputs
    survey_name = options.survey_name
    gp_path = options.gp
    track_path = options.track

    train_svgp(gp_inputs_type, survey_name)

    # load_plot(gp_path, survey_name, track_path)
    # trace_kernel(survey_name)
