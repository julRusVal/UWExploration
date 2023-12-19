import numpy as np

motion_cov_list = np.array([1e-5,1])#, 1e-6, 1e-7])
resampling_cov_list = np.array([10])#, 1, 0.1])
fls_range_std_list = np.array([1e-2])#, 1e-3, 1e-4])
fls_angle_std_list = np.array([np.deg2rad(1)])#, np.deg2rad(0.1), np.deg2rad(0.01)])

params = np.meshgrid(motion_cov_list, resampling_cov_list, fls_range_std_list, fls_angle_std_list)
print(params[0].shape)
print(params[0][0][0][0][0])
print(params)