import os
import re
from operator import index

import numpy as np
import pickle
import metrics
import matplotlib.pyplot as plt

from system_helperss import load_yaml

def extract_numbers_from_template(directory):
    """
    Extracts numbers from file names in a directory matching the pattern
    "scenario_<number>_agent_<number>_transfer_<number>.npy".

    Parameters:
        directory (str): Path to the directory containing files.

    Returns:
        np.ndarray: An Nx3 array where each row contains the extracted numbers.
    """
    # Define the regex pattern to match the file template
    pattern = re.compile(r"scenario_(\d+)_agent_(\d+)_transfer_(\d+)_loss\.npy")

    extracted_numbers = []

    # Iterate through files in the directory
    for file_name in os.listdir(directory):
        match = pattern.match(file_name)
        if match:
            # Extract numbers from the matched file name
            numbers = tuple(map(int, match.groups()))
            extracted_numbers.append(numbers)

    # Convert the extracted list of tuples into an Nx3 NumPy array
    if extracted_numbers:
        return np.array(extracted_numbers)
    else:
        return np.empty((0, 3))  # Return an empty array if no matches are found

def check_results(index, directory_path):
    index = 0
    loss_name = f"scenario_{extracted_array[index][0]}_agent_{extracted_array[index][1]}_transfer_{extracted_array[index][2]}_loss.npy"
    elbo_name = f"scenario_{extracted_array[index][0]}_agent_{extracted_array[index][1]}_transfer_{extracted_array[index][2]}_post.npy"

    loss_array = np.load(os.path.join(directory_path, loss_name))
    smoothing_window = 5 # please make odd
    smoothed_loss = np.convolve(loss_array, np.ones(smoothing_window) / smoothing_window, mode='valid')
    convergence_index = metrics.detect_convergence(smoothed_loss, threshold=0.01, window=5)
    # correct for smoothing window
    convergence_index = int(convergence_index + smoothing_window//2)

    print(convergence_index)

    # plot
    fig, ax = plt.subplots(1)
    ax.plot(loss_array, 'k-')

    # Add a vertical line for convergence
    if convergence_index > 0:
        ax.axvline(x=convergence_index, color='red', linestyle='--', label=f'Convergence {index}')

    # format
    ax.set_xlabel('Iteration')
    ax.set_ylabel('ELBO')
    # ax.set_yscale('log')
    plt.legend()
    plt.tight_layout()

    # Show the plot
    plt.show()

# === Parameters ===
# Data parameters
root_output_dir = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping"
method = 'i'  # select method
model_pickle_path = None  # if set to None will load from directory
param_dicts_pickle_path = None  # if left as None will load from directory
model_losses_pickle_path = None  # if left as None will load from directory
parameters_yaml_path = None  # if left as None will load from directory

# Convergence parameters
smoothing_window = 5  # please make odd, set to 0 to not perform smoothing
convergence_threshold = 0.02
convergence_window = 5

ignore_first_transfer = True

# === Setup ===
if method == 'b':
    directory_path = os.path.join(root_output_dir, "baseline_scenario_output")
    model_pickle_path = None
elif method =='i':
    directory_path = os.path.join(root_output_dir, "independent_scenario_output")
else:
    directory_path = os.path.join(root_output_dir, "federated_scenario_output")

# Load the models and param dictionaries and meta-data
# Pickled models
if model_pickle_path is None:
    model_pickle_path = os.path.join(directory_path, "models.pickle")

if model_pickle_path is not None and os.path.isfile(model_pickle_path):
    with open(model_pickle_path, 'rb') as f:
        models = pickle.load(f)
else:
    models = []

# Pickled param dicts
if param_dicts_pickle_path is None:
    param_dicts_pickle_path = os.path.join(directory_path, "model_param_dicts.pickle")

if param_dicts_pickle_path is not None and os.path.isfile(param_dicts_pickle_path):
    with open(param_dicts_pickle_path, 'rb') as f:
        model_param_dicts = pickle.load(f)
else:
    model_param_dicts = []

# Pickled losses
if model_losses_pickle_path is None:
    model_losses_pickle_path = os.path.join(directory_path, "model_losses.pickle")

if model_losses_pickle_path is not None and os.path.isfile(model_losses_pickle_path):
    with open(model_losses_pickle_path, 'rb') as f:
        model_losses = pickle.load(f)
else:
    model_losses = []

# Parameters
if parameters_yaml_path is None:
    parameters_yaml_path = os.path.join(directory_path, "parameters.yaml")

parameters = load_yaml(parameters_yaml_path)
print(parameters)

# Extract the numbers from the file names
if os.path.exists(directory_path):
    extracted_array = extract_numbers_from_template(directory_path)
    print("Extracted Numbers:\n", extracted_array)
else:
    print(f"Error: Directory '{directory_path}' does not exist.")
    assert False

# Status
# - Extracts numbers from file names in a directory matching the pattern
# "scenario_<number>_agent_<number>_transfer_<number>.npy".
# - Returns an Nx3 array where each row contains the extracted numbers.

# Also attempting to lode the models pickle
# if this fails an empty list will be returned

# TODO: Check models pickling, doesnt appear all of the models are being saved or this was from an old run
# TODO: Lets do some plotting!!!

transfer_counts = []  # List of length N where N is the number of total transfers explored
all_convergences = []  # List of list of length N (N from above), with inner list contains Number of Convergences for each transfer
avg_convergences = []  # N lemgth list of average convergences
std_convergences = []  # N length list of standard deviations

# sort into scenario, agent, transfer
scenarios = np.unique(extracted_array[:, 0])
for scenario_i in scenarios:
    scenario_specific_convergences = []
    scenario_specific_transfers = []  # for debugging
    scenario_specific_agents = []  # for debugging
    transfers = np.unique(extracted_array[extracted_array[:, 0] == scenario_i, 2])
    for transfer_i in transfers:
        # Not sure if this is an issue, but this will skip the first transfer
        if ignore_first_transfer and transfer_i == 0:
            continue

        # Only consider elements that match the current case
        filtered_array = extracted_array[(extracted_array[:, 0] == scenario_i) & (extracted_array[:, 2] == transfer_i)]
        agents = np.unique(filtered_array[:, 1])

        for agent_i in agents:
            # print(f"Scenario {scenario_i}, Agent {agent_i}, Transfer {transfer_i}")
            loss_name = f"scenario_{scenario_i}_agent_{agent_i}_transfer_{transfer_i}_loss.npy"
            elbo_name = f"scenario_{scenario_i}_agent_{agent_i}_transfer_{transfer_i}_post.npy"

            loss_array = np.load(os.path.join(directory_path, loss_name))
            smoothing_window = 5  # please make odd
            smoothed_loss = np.convolve(loss_array, np.ones(smoothing_window) / smoothing_window, mode='valid')
            convergence_index = metrics.detect_convergence(smoothed_loss,
                                                           threshold=convergence_threshold, window=convergence_window)
            if convergence_index < 0:
                # if convergence is not detected, skip
                continue
            # correct for smoothing window
            convergence_index = int(convergence_index + smoothing_window // 2)

            scenario_specific_convergences.append(convergence_index)
            scenario_specific_transfers.append(transfer_i)
            scenario_specific_agents.append(agent_i)

    transfer_counts.append(scenario_i)
    all_convergences.append(scenario_specific_convergences)
    avg_convergences.append(np.mean(scenario_specific_convergences))
    std_convergences.append(np.std(scenario_specific_convergences))

    if True:
        print(f"Scenario {scenario_i}:")
        print(f"Transfers: {np.unique(scenario_specific_transfers)}")
        print(f"Agents: {np.unique(scenario_specific_agents)}")
        print("Convergences:", scenario_specific_convergences)

# Plot
fig, ax = plt.subplots(1)
# ax.plot(transfer_counts, avg_convergences, 'k-')
ax.errorbar(transfer_counts, avg_convergences, yerr=std_convergences, fmt='k.', capsize=5)

# format
ax.set_xlabel('Scenario Transfer Count')
ax.set_ylabel('Average Convergence')
plt.tight_layout()
plt.show()


print("Transfer Count:", transfer_counts)
print("Average Convergences:", avg_convergences)

if True:
    check_results(index=0, directory_path=directory_path)

print("done!!")