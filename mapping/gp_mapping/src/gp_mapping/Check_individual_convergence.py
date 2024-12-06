import numpy as np
import matplotlib.pyplot as plt
import os

import metrics

directory_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_scenario_output/"
scenario_i = int(7)
agent_i = int(0)
transfer_i = int(4)
loss_name = f"scenario_{scenario_i}_agent_{agent_i}_transfer_{transfer_i}_loss.npy"

#elbo_name = f"scenario_{extracted_array[index][0]}_agent_{extracted_array[index][1]}_transfer_{extracted_array[index][2]}_post.npy"

smoothing_window = 5 # please make odd
convergence_threshold = 0.02
convergence_window = 5

loss_array = np.load(os.path.join(directory_path, loss_name))

smoothed_loss = np.convolve(loss_array, np.ones(smoothing_window) / smoothing_window, mode='valid')
convergence_index = metrics.detect_convergence(smoothed_loss, threshold=convergence_threshold, window=convergence_window)
# correct for smoothing window
convergence_index = int(convergence_index + smoothing_window//2)

print(convergence_index)

# plot
fig, ax = plt.subplots(1)
ax.plot(loss_array, 'k-')

# Add a vertical line for convergence
if convergence_index > 0:
    ax.axvline(x=convergence_index, color='red', linestyle='--', label=f'Convergence {convergence_index}')

# format
ax.set_xlabel('Iteration')
ax.set_ylabel('ELBO')
# ax.set_yscale('log')
plt.legend()
plt.tight_layout()

# Show the plot
plt.show()