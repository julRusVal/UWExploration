import pandas as pd
import matplotlib.pyplot as plt
import csv

# Define the path to the CSV file
csv_file_path = '/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/[0.0, 0.0, 0.0, 0.0, 0.0, 0.000001]_[1.0, 1.0, 0.0, 0.0, 0.0, 0.0]_0.001_0.00174533/20231218-182311/auv_1.csv'

# Read the CSV file
data = pd.read_csv(csv_file_path)

# Plotting the graphs

# Plot 1: Distance and Bearing Errors Over Time
plt.figure(figsize=(12, 10))
plt.subplot(2, 2, 1)
plt.plot(data['left_distance_errors'], label='Left Distance Error')
plt.plot(data['right_distance_errors'], label='Right Distance Error')
plt.plot(data['left_bearing_errors'], label='Left Bearing Error')
plt.plot(data['right_bearing_errors'], label='Right Bearing Error')
plt.xlabel('Callback Iteration')
plt.ylabel('Error [m] or [deg]')
plt.legend()
plt.title('Distance and Bearing Errors Over Time')
plt.ylim(0, 1)
plt.grid(True)

# Plot 2: Covariance Sum Over Time
plt.subplot(2, 2, 2)
plt.plot(data['ego_cov_list'], label='Ego x & y Covariance Sum')
plt.plot(data['left_cov_list'], label='Left x & y Covariance Sum')
plt.plot(data['right_cov_list'], label='Right x & y Covariance Sum')
plt.xlabel('Callback Iteration')
plt.ylabel('Covariance Sum')
plt.legend()
plt.title('Covariance Sum Over Time')
plt.grid(True)

# Plot 3: Absolute Positional Error Over Time
plt.subplot(2, 2, 3)
plt.plot(data['ego_abs_error'], label='Ego Absolute Positional Error')
plt.plot(data['left_abs_error'], label='Left Absolute Positional Error')
plt.plot(data['right_abs_error'], label='Right Absolute Position Error')
plt.xlabel('Callback Iteration')
plt.ylabel('Absolute Positional Error [m]')
plt.legend()
plt.title('Absolute Positional Error Over Time')
plt.grid(True)

# Adjust layout
plt.tight_layout()

# Show all plots
plt.show()