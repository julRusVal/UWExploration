import pandas as pd
import matplotlib.pyplot as plt
import csv

# Define the path to the CSV file
csv_file_path = '/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/0.0, 0.0, 0.0, 0.0, 0.0, 0.000001_1.0, 1.0, 0.0, 0.0, 0.0, 0.0_0.001_0.00174533/stats.csv'
alpha_e_bars = 0.2
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

# Check if columns with '_std' suffix exist and plot error bars if available
if 'left_distance_errors_std' in data.columns and 'right_distance_errors_std' in data.columns \
        and 'left_bearing_errors_std' in data.columns and 'right_bearing_errors_std' in data.columns:
    plt.errorbar(data.index, data['left_distance_errors'], yerr=data['left_distance_errors_std'], fmt='o', label='Left Distance Error Std', alpha=alpha_e_bars)
    plt.errorbar(data.index, data['right_distance_errors'], yerr=data['right_distance_errors_std'], fmt='o', label='Right Distance Error Std', alpha=alpha_e_bars)
    plt.errorbar(data.index, data['left_bearing_errors'], yerr=data['left_bearing_errors_std'], fmt='o', label='Left Bearing Error Std', alpha=alpha_e_bars)
    plt.errorbar(data.index, data['right_bearing_errors'], yerr=data['right_bearing_errors_std'], fmt='o', label='Right Bearing Error Std', alpha=alpha_e_bars)

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

# Check if columns with '_std' suffix exist and plot error bars if available
if 'ego_cov_list_std' in data.columns and 'left_cov_list_std' in data.columns and 'right_cov_list_std' in data.columns:
    plt.errorbar(data.index, data['ego_cov_list'], yerr=data['ego_cov_list_std'], fmt='o', label='Ego Covariance Sum Std', alpha=alpha_e_bars)
    plt.errorbar(data.index, data['left_cov_list'], yerr=data['left_cov_list_std'], fmt='o', label='Left Covariance Sum Std', alpha=alpha_e_bars)
    plt.errorbar(data.index, data['right_cov_list'], yerr=data['right_cov_list_std'], fmt='o', label='Right Covariance Sum Std', alpha=alpha_e_bars)

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

# Check if columns with '_std' suffix exist and plot error bars if available
if 'ego_abs_error_std' in data.columns and 'left_abs_error_std' in data.columns and 'right_abs_error_std' in data.columns:
    plt.errorbar(data.index, data['ego_abs_error'], yerr=data['ego_abs_error_std'], fmt='o', label='Ego Abs. Pos. Error Std', alpha=alpha_e_bars)
    plt.errorbar(data.index, data['left_abs_error'], yerr=data['left_abs_error_std'], fmt='o', label='Left Abs. Pos. Error Std', alpha=alpha_e_bars)
    plt.errorbar(data.index, data['right_abs_error'], yerr=data['right_abs_error_std'], fmt='o', label='Right Abs. Pos. Error Std', alpha=alpha_e_bars)

# Adjust layout
plt.tight_layout()

# Show all plots
plt.show()

#_______________________________________________________________________________
# import pandas as pd
# import matplotlib.pyplot as plt
# import csv

# # Define the path to the CSV file
# csv_file_path = '/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/0.0, 0.0, 0.0, 0.0, 0.0, 0.000001_1.0, 1.0, 0.0, 0.0, 0.0, 0.0_0.001_0.00174533/row_wise_means.csv'

# # Read the CSV file
# data = pd.read_csv(csv_file_path)

# # Plotting the graphs

# # Plot 1: Distance and Bearing Errors Over Time
# plt.figure(figsize=(12, 10))
# plt.subplot(2, 2, 1)
# plt.plot(data['left_distance_errors'], label='Left Distance Error')
# plt.plot(data['right_distance_errors'], label='Right Distance Error')
# plt.plot(data['left_bearing_errors'], label='Left Bearing Error')
# plt.plot(data['right_bearing_errors'], label='Right Bearing Error')
# plt.xlabel('Callback Iteration')
# plt.ylabel('Error [m] or [deg]')
# plt.legend()
# plt.title('Distance and Bearing Errors Over Time')
# plt.ylim(0, 1)
# plt.grid(True)

# # Plot 2: Covariance Sum Over Time
# plt.subplot(2, 2, 2)
# plt.plot(data['ego_cov_list'], label='Ego x & y Covariance Sum')
# plt.plot(data['left_cov_list'], label='Left x & y Covariance Sum')
# plt.plot(data['right_cov_list'], label='Right x & y Covariance Sum')
# plt.xlabel('Callback Iteration')
# plt.ylabel('Covariance Sum')
# plt.legend()
# plt.title('Covariance Sum Over Time')
# plt.grid(True)

# # Plot 3: Absolute Positional Error Over Time
# plt.subplot(2, 2, 3)
# plt.plot(data['ego_abs_error'], label='Ego Absolute Positional Error')
# plt.plot(data['left_abs_error'], label='Left Absolute Positional Error')
# plt.plot(data['right_abs_error'], label='Right Absolute Position Error')
# plt.xlabel('Callback Iteration')
# plt.ylabel('Absolute Positional Error [m]')
# plt.legend()
# plt.title('Absolute Positional Error Over Time')
# plt.grid(True)

# # Adjust layout
# plt.tight_layout()

# # Show all plots
# plt.show()