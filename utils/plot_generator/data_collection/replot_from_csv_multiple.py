import pandas as pd
import matplotlib.pyplot as plt
# plt.rcParams.update({'font.size': 12})  # Sets default font size to 12
import csv

def plot_csv(csv_file_paths,dataset_names, vlines_x=[],separate_windows=False,t_cut=0,lw=4):
    if separate_windows:
        plt.rcParams.update({'font.size': 24})  # Sets default font size to 24
    else:
        plt.rcParams.update({'font.size': 12})  # Sets default font size to 12
    alpha_e_bars = 0.2
    alpha_v_lines = 0.5
    # data = pd.read_csv(csv_file_path)

    if not separate_windows:
        plt.figure(figsize=(12, 10))

    # Plot 1: Distance and Bearing Errors Over Time

    if separate_windows:
        plt.figure(figsize=(12, 10))
        plt.subplot(1, 1, 1)
    else:
        plt.subplot(2, 2, 1)

    for x in vlines_x:
        plt.axvline(x=x, color='k', linestyle='--',alpha=alpha_v_lines)
    for i, csv_file_path in enumerate(csv_file_paths):
        data = pd.read_csv(csv_file_path)
        dataset_name = dataset_names[i]
        if "l_d_e_t" in data.columns:
            # plt.plot(data["l_d_e_t"].values,data['left_distance_errors'].values+data['left_bearing_errors'].values, label='Left CAPE '+dataset_name, linewidth=lw)
            plt.plot(data["r_d_e_t"].values,data['right_distance_errors'].values+data['right_bearing_errors'].values, label='Right CAPE '+dataset_name, linewidth=lw)
            plt.xlabel('Time [s]')
        else:
            # plt.plot(data['left_distance_errors']+data['left_bearing_errors'], label='Left Error Sum', linewidth=lw )
            plt.plot(data['right_distance_errors']+data['right_bearing_errors'], label='Right Error Sum', linewidth=lw)
            plt.xlabel('Callback Iteration')
        plt.ylabel('CAPE [-]')
        # plt.legend()
        plt.title('Composite Average Pose Error Over Time')
        plt.grid(True)
        plt.xlim(right=t_cut)
    # Check if columns with '_std' suffix exist and plot error bars if available
        if 'left_distance_errors_std' in data.columns and 'right_distance_errors_std' in data.columns \
                and 'left_bearing_errors_std' in data.columns and 'right_bearing_errors_std' in data.columns:
            # plt.errorbar(data.index, data['left_distance_errors'], yerr=data['left_distance_errors_std'], fmt='o', label='Left Distance Error Std', alpha=alpha_e_bars)
            # plt.errorbar(data.index, data['right_distance_errors'], yerr=data['right_distance_errors_std'], fmt='o', label='Right Distance Error Std', alpha=alpha_e_bars)
            # plt.errorbar(data.index, data['left_bearing_errors'], yerr=data['left_bearing_errors_std'], fmt='o', label='Left Bearing Error Std', alpha=alpha_e_bars)
            # plt.errorbar(data.index, data['right_bearing_errors'], yerr=data['right_bearing_errors_std'], fmt='o', label='Right Bearing Error Std', alpha=alpha_e_bars)
            try:
                # plt.errorbar(data['l_d_e_t'].values, data['left_distance_errors']+data['left_bearing_errors'], yerr=data['left_distance_errors_std']+data['left_bearing_errors_std'], fmt='o', label='Left Distance and Bearing Error Std', alpha=alpha_e_bars)
                plt.errorbar(data['r_d_e_t'].values, data['right_distance_errors']+data['right_bearing_errors'], yerr=data['right_distance_errors_std']+data['right_bearing_errors_std'], fmt='o', label='$\sigma_{right}$ over 4 runs', alpha=alpha_e_bars)
            except:
                # plt.errorbar(data.index, data['left_distance_errors']+data['left_bearing_errors'], yerr=data['left_distance_errors_std']+data['left_bearing_errors_std'], fmt='o', label='Left Distance and Bearing Error Std', alpha=alpha_e_bars)
                plt.errorbar(data.index, data['right_distance_errors']+data['right_bearing_errors'], yerr=data['right_distance_errors_std']+data['right_bearing_errors_std'], fmt='o', label='std', alpha=alpha_e_bars)
        # Plot 2: Covariance Sum Over Time
        # plt.subplot(2, 2, 2)
        plt.legend()
        
    # data = pd.read_csv(csv_file_paths[0])
    if separate_windows:
        plt.figure(figsize=(12, 10))
        plt.subplot(1, 1, 1)
    else:
        plt.subplot(2, 2, 2)
    for x in vlines_x:
        plt.axvline(x=x, color='k', linestyle='--',alpha=alpha_v_lines)
    for i, csv_file_path in enumerate(csv_file_paths):
        data = pd.read_csv(csv_file_path)
        dataset_name = dataset_names[i]
        if 'e_c_t' in data.columns:
            plt.plot(data['e_c_t'].values, data['ego_cov_list'].values, label='Ego PDI '+dataset_name, linewidth=lw)
            # plt.plot(data['l_c_t'].values, data['left_cov_list'].values, label='Left PDI '+dataset_name, linewidth=lw)
            plt.plot(data['r_c_t'].values, data['right_cov_list'].values, label='Right PDI '+dataset_name, linewidth=lw)
            plt.xlabel('Time [s]')
        else:
            plt.plot(data['ego_cov_list'], label='Ego x & y Covariance Sum')
            # plt.plot(data['left_cov_list'], label='Left x & y Covariance Sum')
            plt.plot(data['right_cov_list'], label='Right x & y Covariance Sum')
            plt.xlabel('Callback Iteration')
        plt.ylabel('PDI [m$^2$]')
        # plt.legend()
        plt.title('Particle Dispersion Index Over Time')
        plt.grid(True)
        plt.xlim(right=t_cut)

        # Check if columns with '_std' suffix exist and plot error bars if available
        if 'ego_cov_list_std' in data.columns and 'left_cov_list_std' in data.columns and 'right_cov_list_std' in data.columns:
            try:
                plt.errorbar(data['e_c_t'].values, data['ego_cov_list'], yerr=data['ego_cov_list_std'], fmt='o', label='$\sigma_{ego}$ over 4 runs', alpha=alpha_e_bars, linewidth=lw)
                # plt.errorbar(data['l_c_t'].values, data['left_cov_list'], yerr=data['left_cov_list_std'], fmt='o', label='Left Covariance Sum Std', alpha=alpha_e_bars, linewidth=lw)
                plt.errorbar(data['r_c_t'].values, data['right_cov_list'], yerr=data['right_cov_list_std'], fmt='o', label='$\sigma_{right}$ over 4 runs', alpha=alpha_e_bars, linewidth=lw)
            except:
                plt.errorbar(data.index, data['ego_cov_list'], yerr=data['ego_cov_list_std'], fmt='o', label='Ego Covariance Sum Std', alpha=alpha_e_bars)
                # plt.errorbar(data.index, data['left_cov_list'], yerr=data['left_cov_list_std'], fmt='o', label='Left Covariance Sum Std', alpha=alpha_e_bars)
                plt.errorbar(data.index, data['right_cov_list'], yerr=data['right_cov_list_std'], fmt='o', label='Right Covariance Sum Std', alpha=alpha_e_bars)
        plt.legend()
    # Plot 3: Distance Error Over Time for Mean Distance Between All Particles
    # plt.subplot(2, 2, 3)
    if separate_windows:
        plt.figure(figsize=(12, 10))
        plt.subplot(1, 1, 1)
    else:
        plt.subplot(2, 2, 3)
    for x in vlines_x:
        plt.axvline(x=x, color='k', linestyle='--', alpha=alpha_v_lines)
    for i, csv_file_path in enumerate(csv_file_paths):
        data = pd.read_csv(csv_file_path)
        dataset_name = dataset_names[i]
        if 'l_d_e_b_a_p_t' in data.columns:
            # plt.plot(data['l_d_e_b_a_p_t'].values, data['left_distance_errors_between_all_particles'].values, label='Left APDE '+dataset_name, linewidth=lw)
            plt.plot(data['r_d_e_b_a_p_t'].values, data['right_distance_errors_between_all_particles'].values, label='Right APDE '+dataset_name, linewidth=lw)
            plt.xlabel('Time [s]')
        elif "leftleft_distance_errors_between_all_particles" in data.columns:
            # plt.plot(data['left_distance_errors_between_all_particles'], label='Left Distance Error Mean Of All Particles')
            plt.plot(data['right_distance_errors_between_all_particles'], label='Right Distance Error Mean Of All Particles')
            plt.xlabel('Callback Iteration')
        else:
            print("Mean distance for all particles doesnt exist")
        plt.ylabel('APDE [m]')
        # plt.legend()
        plt.title('Average Particle Distance Error Over Time')
        plt.grid(True)
        plt.xlim(right=t_cut)
        # Check if columns with '_std' suffix exist and plot error bars if available
        if 'left_distance_errors_between_all_particles_std' in data.columns and 'right_distance_errors_between_all_particles_std' in data.columns:
            try:
                # plt.errorbar(data['l_d_e_b_a_p_t'].values, data['left_distance_errors_between_all_particles'], yerr=data['left_distance_errors_between_all_particles_std'], fmt='o', label='Left Distance Error Mean Of All Particles Std', alpha=alpha_e_bars)
                plt.errorbar(data['r_d_e_b_a_p_t'].values, data['right_distance_errors_between_all_particles'], yerr=data['right_distance_errors_between_all_particles_std'], fmt='o', label='$\sigma_{right}$ over 4 runs', alpha=alpha_e_bars)
            except:
                # plt.errorbar(data.index, data['left_distance_errors_between_all_particles'], yerr=data['left_distance_errors_between_all_particles_std'], fmt='o', label='Left Distance Error Mean Of All Particles Std', alpha=alpha_e_bars)
                plt.errorbar(data.index, data['right_distance_errors_between_all_particles'], yerr=data['right_distance_errors_between_all_particles_std'], fmt='o', label='$\sigma_{right}$ over 4 runs', alpha=alpha_e_bars)
        plt.legend()

    # Plot 4: Absolute Positional Error Over Time
    # plt.subplot(2, 2, 4)
    if separate_windows:
        plt.figure(figsize=(12, 10))
        plt.subplot(1, 1, 1)
    else:
        plt.subplot(2, 2, 4)
    for x in vlines_x:
        plt.axvline(x=x, color='k', linestyle='--', alpha=alpha_v_lines)
    
    for i, csv_file_path in enumerate(csv_file_paths):
        data = pd.read_csv(csv_file_path)
        dataset_name = dataset_names[i]
        if 'e_a_e_t' in data.columns:
            plt.plot(data['e_a_e_t'].values, data['ego_abs_error'].values, label='Ego APE '+dataset_name, linewidth=lw)
            # plt.plot(data['l_a_e_t'].values, data['left_abs_error'].values, label='Left APE '+dataset_name, linewidth=lw)
            plt.plot(data['r_a_e_t'].values, data['right_abs_error'].values, label='Right APE '+dataset_name, linewidth=lw)
            plt.xlabel('Time [s]')
        else:
            plt.plot(data['ego_abs_error'], label='Ego Absolute Positional Error')
            # plt.plot(data['left_abs_error'], label='Left Absolute Positional Error')
            plt.plot(data['right_abs_error'], label='Right Absolute Position Error')
            plt.xlabel('Callback Iteration')
        plt.ylabel('APE [m]')
        # plt.legend()
        plt.title('Absolute Position Error Over Time')
        plt.grid(True)
        plt.xlim(right=t_cut)
        # Check if columns with '_std' suffix exist and plot error bars if available
        if 'ego_abs_error_std' in data.columns and 'left_abs_error_std' in data.columns and 'right_abs_error_std' in data.columns:
            try:
                plt.errorbar(data['e_a_e_t'].values, data['ego_abs_error'], yerr=data['ego_abs_error_std'], fmt='o', label='$\sigma_{ego}$ over 4 runs', alpha=alpha_e_bars)
                # plt.errorbar(data['l_a_e_t'].values, data['left_abs_error'], yerr=data['left_abs_error_std'], fmt='o', label='Left Abs. Pos. Error Std', alpha=alpha_e_bars)
                plt.errorbar(data['r_a_e_t'].values, data['right_abs_error'], yerr=data['right_abs_error_std'], fmt='o', label='$\sigma_{right}$ over 4 runs', alpha=alpha_e_bars)
            except: 
                plt.errorbar(data.index, data['ego_abs_error'], yerr=data['ego_abs_error_std'], fmt='o', label='Ego Abs. Pos. Error Std', alpha=alpha_e_bars)
                # plt.errorbar(data.index, data['left_abs_error'], yerr=data['left_abs_error_std'], fmt='o', label='Left Abs. Pos. Error Std', alpha=alpha_e_bars)
                plt.errorbar(data.index, data['right_abs_error'], yerr=data['right_abs_error_std'], fmt='o', label='Right Abs. Pos. Error Std', alpha=alpha_e_bars)
        plt.legend()
    # Adjust layout
    plt.tight_layout()

    # Show all plots
    plt.show()

#DR only
DR = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240111_162124/my1e-05_rxy1.0_fr0.001_fa0.00174533/20240111_162125/auv_0.csv"
defaultx5 = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240111_211835/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/stats.csv"
defaultx50 = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240119_150728/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/stats.csv"
default = defaultx50
# default = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240111_211835/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/20240111_215030/auv_0.csv"
all_polyx5 = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_144137/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/stats.csv"
all_polyx50 = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240119_233258/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/stats.csv"
all_poly = all_polyx50
all_mono = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_160923/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/stats.csv"

default_realistic_comms = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_171042/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/stats.csv"
default_unlimited_comms = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_182056/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/stats.csv"

# names = ["DR only ","RBPF default"]
# plot_csv([DR,default],names,vlines_x=[78, 97, 320, 338],separate_windows=False,t_cut=350)

names = ["RBPF default","RBPF all poly"]
plot_csv([default,all_poly],names,vlines_x=[78, 97, 320, 338],separate_windows=True,t_cut=350)

# names = ["RBPF default","0", "1","2","3","4"]
# plot_csv([default,"/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_144137/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/20240118_144138/auv_0.csv",
#           "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_144137/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/20240118_144936/auv_0.csv",
#           "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_144137/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/20240118_145736/auv_0.csv",
#            "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_144137/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/20240118_150534/auv_0.csv",
#             "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240118_144137/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296/20240118_151333/auv_0.csv" ],
#             names,vlines_x=[78, 97, 320, 338],separate_windows=True,t_cut=350)

# names = ["RBPF default","RBPF all mono"]
# plot_csv([default,all_mono],names,vlines_x=[78, 97, 320, 338],separate_windows=True,t_cut=350)

# names = ["RBPF default","RBPF default+realistic comms"]
# plot_csv([default,default_realistic_comms],names,vlines_x=[78, 97, 320, 338],separate_windows=True,t_cut=350)

# names = ["RBPF default","RBPF default+unlimited comms"]
# plot_csv([default,default_unlimited_comms],names,vlines_x=[78, 97, 320, 338],separate_windows=False,t_cut=350)
