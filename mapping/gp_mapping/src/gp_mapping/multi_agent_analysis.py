import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm, Normalize
from Consistancy_plot import compute_consistency_metrics

"""
compare the consistancy metrics between the three models produced by the multi-agent method

Note: for now this should look bad as there is no fusing going on
"""

def plot_side_by_side_with_shared_colorbar(data1, data2, cmap='viridis',
                                           title1="title 1", title2="title 2",
                                           filename=""):
    """
    Plot two NumPy arrays side-by-side with a shared color bar.
    The left plot uses linear normalization, and the right plot uses log normalization.

    Parameters:
    - data1: NumPy array for the left plot (linear normalization).
    - data2: NumPy array for the right plot (log normalization).
    - cmap: Colormap for the plots (default is 'viridis').
    - title1: Title for the left plot.
    - title2: Title for the right plot.
    """


    # Set up the figure and axes for side-by-side plots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

    # First plot with linear normalization
    im1 = ax1.imshow(np.transpose(data1), origin='lower', norm=LogNorm(), cmap=cmap)  #, norm=Normalize())
    ax1.set_title(title1.title())

    # Second plot with log normalization to emphasize lower values
    im2 = ax2.imshow(np.transpose(data2), origin='lower', norm=LogNorm(), cmap=cmap)  #, norm=LogNorm())
    ax2.set_title(title2.title())

    # Add a shared color bar across both plots
    cbar = fig.colorbar(im2, ax=[ax1, ax2], orientation='vertical', fraction=0.046, pad=0.04)
    cbar.set_label("Value")

    if isinstance(filename, str) and len(filename) > 0:
        plt.savefig(filename, bbox_inches='tight', dpi=1000)
        plt.close()
    else:
        plt.show()


# Parameters
agent_count = 3
resolution = 1
plotting=True

# Model data
agent_models_root_dir = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_output"

# reference data
ref_root_dir = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets"
ref_path = os.path.join(ref_root_dir, "lost_targets/pcl_super_cleaned.npy")  # path to .npy file with reference points

# Output
output_root_dir = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_output"

# Perform consistency metrics
# Load the required data
baseline_points = np.load(os.path.join(agent_models_root_dir, f"model_baseline_post.npy"))[:, :3]
aggregated_points = np.load(os.path.join(agent_models_root_dir, f"model_aggregated_post.npy"))[:, :3]
aggregated_no_train_points = np.load(os.path.join(agent_models_root_dir, f"model_aggregated_no_train_post.npy"))[:, :3] # "model_aggregated_no_train_post.npy"

# Remember to exclude the last column in the .npy file, corresponding the learned uncertainty
agent_names = []
agent_models_points = []
for i in range(agent_count):
    name = f"model_{i}_post"
    # Remember to exclude the last column in the .npy file, corresponding the learned uncertainty
    agent_points = np.load(os.path.join(agent_models_root_dir, f"{name}.npy"))[:, :3]

    agent_names.append(name)
    agent_models_points.append(agent_points)

#No remembering needed for the reference
ref_points = np.load(ref_path)

# Compute the individual metrics
agent_metrics = []
for i in range(agent_count):
    print(f"Agent {i} Metrics")
    plot_file_name = os.path.join(output_root_dir, agent_names[i] + "_consistency.png")
    metrics = compute_consistency_metrics(agent_models_points[i][:, :3], ref_points, resolution,
                                          return_matrix=True, plot=plotting, label=agent_names[i],
                                          filename=plot_file_name)

    agent_metrics.append(metrics)

# Combined metrics
print("Combined Metrics")
label = "combined"
plot_file_name = os.path.join(output_root_dir, f"{label}_consistency.png")
combined_metrics = compute_consistency_metrics(np.concatenate(agent_models_points, axis=0)[:, :3], ref_points, resolution,
                                               return_matrix=True, plot=plotting, label=label.title(),
                                               filename=plot_file_name)

# baseline metrics
print("baseline Metrics")
label = " baseline"
plot_file_name = os.path.join(output_root_dir, f"{label}_consistency.png")
baseline_metrics = compute_consistency_metrics(baseline_points, ref_points, resolution,
                                               return_matrix=True, plot=plotting, label=label.title(),
                                               filename=plot_file_name)

# Combined metrics
print("Aggregated Metrics")
label = "aggregated"
plot_file_name = os.path.join(output_root_dir, f"{label}_consistency.png")
aggregated_metrics = compute_consistency_metrics(aggregated_points, ref_points, resolution,
                                               return_matrix=True, plot=plotting, label=label.title(),
                                               filename=plot_file_name)

# Combined metrics
print("Aggregated No Train Metrics")
label = "aggregated_no_train"
plot_file_name = os.path.join(output_root_dir, f"{label}_consistency.png")
aggregated_no_train_metrics = compute_consistency_metrics(aggregated_no_train_points, ref_points, resolution,
                                               return_matrix=True, plot=plotting, label=label.title(),
                                               filename=plot_file_name)


# baseline / aggregated plot
label = "comparison"
plot_file_name = os.path.join(output_root_dir, f"{label}_consistency.png")
plot_side_by_side_with_shared_colorbar(baseline_metrics['consistency_matrix'], aggregated_metrics['consistency_matrix'],
                                       title1="baseline", title2="aggregated", filename=plot_file_name)