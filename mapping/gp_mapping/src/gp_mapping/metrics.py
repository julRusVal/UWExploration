import numpy as np


def detect_convergence(loss_values, threshold=1e-4, window=5):
    """
    Detects when convergence occurs in a sequence of loss/ELBO values.

    All values within a window must be within `threshold` of each other.

    Parameters:
        loss_values (array-like): An array of loss or ELBO values.
        threshold (float): The change threshold below which the values are considered converged.
        window (int): Number of consecutive iterations required for convergence.

    Returns:
        int: Index where convergence occurs (-1 if not converged).
    """
    if len(loss_values) < window + 1:
        return -1  # Not enough data to check convergence.

    min_change = np.inf

    for i in range(len(loss_values) - window + 1):
        recent_losses = loss_values[i:i + window]
        changes = np.abs(np.diff(recent_losses))
        if np.all(changes < threshold):
            return i + window
        else:
            if np.min(changes) < min_change:
                min_change = np.min(changes)

    print(f"Convergence not reached within the given threshold: {min_change} - {threshold}")
    return -1  # No convergence detected.

def check_convergence(losses, threshold=0.01, window=10):
    """
    Determines the epoch at which convergence occurs given a threshold and window size.

    Checks if the difference between the maximum and minimum in losses over a window is less than the threshold.

    Parameters:
    - losses (np.array): An (n,) array of loss values.
    - threshold (float): The maximum allowable rate of change in the loss values for convergence.
    - window (int): The number of recent epochs to consider for convergence.

    Returns:
    - convergence_epoch (int): The epoch at which convergence occurs, or -1 if convergence is not reached.
    """
    n = len(losses)

    # Ensure we have enough losses to check
    if n < window:
        print("Not enough data points to determine convergence.")
        return -1

    # Iterate through the array to check for convergence over the defined window
    for i in range(n - window + 1):
        recent_losses = losses[i:i + window]
        max_loss = np.max(recent_losses)
        min_loss = np.min(recent_losses)

        # Calculate the change in losses over the window
        loss_change = max_loss - min_loss

        # Check if the change is within the convergence threshold
        if loss_change < threshold:
            print(f"Convergence occurred at epoch {i + window - 1}")
            return i + window - 1  # Return the last epoch within the window

    print("Convergence not reached within the given threshold.")
    return -1