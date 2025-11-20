import os
import numpy as np
import matplotlib.pyplot as plt

from gp_mapping_utils.svgp_plotting import plot_two_arrays

if __name__ == "__main__":
    # Example usage
    # User input
    data_dir = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_output"
    base_loss_file = "model_baseline_loss.npy"
    agg_loss_file = "model_aggregated_loss.npy"
    base_label = "Baseline"
    agg_label = "Aggregated"

    # Form full paths with some simple checking
    base_loss_file_path = os.path.join(data_dir, base_loss_file)
    agg_loss_file_path = os.path.join(data_dir, agg_loss_file)
    
    data_paths = [base_loss_file_path, agg_loss_file_path]
    
    for data_path in data_paths:
        if not os.path.isfile(data_path):
            print(f"Data path {data_path} does not exist.")
            exit(1)
    
    # Load the data
    base_loss = np.load(os.path.join(data_dir, "model_baseline_loss.npy"))
    agg_loss = np.load(os.path.join(data_dir, "model_aggregated_loss.npy"))
    
    # Plot the data
    plot_two_arrays(base_loss, agg_loss, label1=base_label, label2=agg_label,
                    xlabel="Iteration", ylabel="ELBO",
                    title=f"Comparison of {base_label} and {agg_label} models".title())
