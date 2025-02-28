import os
import numpy as np
import matplotlib.pyplot as plt



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
