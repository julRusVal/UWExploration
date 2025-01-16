import pandas as pd
import glob
import numpy as np

def align_to_common_timebase(df, common_time, col):
    """
    Aligns the dataframe to the common time base using interpolation.
    """
    # df_time = df['{}_t'.format(col)]  # Extract the time column for the specific value column
    #df_time is the col + 12 columns to the right
    df_time = df[df.columns[df.columns.get_loc(col) + 12]]
    df_aligned = pd.DataFrame({
        'time': common_time,
        col: np.interp(common_time, df_time, df[col])
    })
    return df_aligned

# Define the base directory where folders containing CSV files are located
def gen_stats(base_directory):
# base_directory = '/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20231221_094131/my1e-05_rxy10_fr0.01_fa0.017453292519943295/'

    # Create an empty list to store file paths
    file_paths = []

    # Use glob to find CSV files in subdirectories
    # print(glob.glob(base_directory + '*/'))
    base_directory = base_directory + "/"
    for folder_name in glob.glob(base_directory + '*/'):
        # print("Found folder:", folder_name)
        csv_files = glob.glob(folder_name + '/auv_0.csv') #CHOOSE AUV HERE
        file_paths.extend(csv_files)

    # Print the found file paths (for debugging purposes)
    # print("File paths found:")
    # for path in file_paths:
    #     print(path)

    # Read all CSV files into a list of DataFrames
    dfs = [pd.read_csv(file) for file in file_paths]

    # # Check if any DataFrames were loaded
    # if not dfs:
    #     print("No CSV files found or unable to read CSV files.")
    # else:
    #     df_stats = pd.DataFrame()
    #     #iterate over each column in dfs[0]
    #     for col in dfs[0].columns:
    #         #iterate over each dataframe in dfs
    #         df_temp = pd.DataFrame()
    #         for i,df in enumerate(dfs):
    #             #append df[col] to df_temp
    #             # df_temp = df_temp.append(df[col])
    #             df_temp[i] = df[col]
    #         #calculate row wise mean and std for df_temp
    #         df_stats[col] = df_temp.mean(axis=1)
    #         df_stats[col+'_std'] = df_temp.std(axis=1)
        
    #     #save df_stats to csv
    #     df_stats.to_csv(base_directory + 'stats.csv', index=False)
    #     print("Saved stats to: ", base_directory + 'stats.csv')
    if not dfs:
        print("No CSV files found or unable to read CSV files.")
        return

    time_cols = [col for col in dfs[0].columns if col.endswith('_t')]
    value_cols = [col for col in dfs[0].columns if not col.endswith('_t')]

    # Using the first file's time data as a common scale
    common_time = dfs[0][time_cols[4]]  # Assuming all time columns are similar in the first file

    # df_stats = pd.DataFrame({'time': common_time})
    df_stats = pd.DataFrame()
    #add common_time for each column name in time_cols
    for col in time_cols:
        df_stats[col] = common_time

    for col in value_cols:
        aligned_dfs = [align_to_common_timebase(df, common_time, col) for df in dfs]
        df_temp = pd.concat([df[col] for df in aligned_dfs], axis=1)
        df_stats[col] = df_temp.mean(axis=1)
        df_stats[col + '_std'] = df_temp.std(axis=1)

    # Save the final DataFrame to CSV
    output_path = base_directory + 'statsCommonTNew.csv'
    df_stats.to_csv(output_path, index=False)
    print("Saved stats to:", output_path)
            
gen_stats('/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240121_090945/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296')