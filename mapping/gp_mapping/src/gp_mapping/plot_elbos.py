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

# Example usage
base_loss = np.load("/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_output/model_baseline_loss.npy")
agg_loss = np.load("/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_output/model_aggregated_loss.npy")

base_label = "Baseline"
agg_label = "Aggregated"
plot_two_arrays(base_loss, agg_loss, label1=base_label, label2=agg_label,
                xlabel="Iteration", ylabel="ELBO",
                title=f"Comparison of {base_label} and {agg_label} models".title())
