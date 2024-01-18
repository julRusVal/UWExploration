import pandas as pd
import glob

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
        csv_files = glob.glob(folder_name + '/auv_0.csv')
        file_paths.extend(csv_files)

    # Print the found file paths (for debugging purposes)
    # print("File paths found:")
    # for path in file_paths:
    #     print(path)

    # Read all CSV files into a list of DataFrames
    dfs = [pd.read_csv(file) for file in file_paths]

    # Check if any DataFrames were loaded
    if not dfs:
        print("No CSV files found or unable to read CSV files.")
    else:
        df_stats = pd.DataFrame()
        #iterate over each column in dfs[0]
        for col in dfs[0].columns:
            #iterate over each dataframe in dfs
            df_temp = pd.DataFrame()
            for i,df in enumerate(dfs):
                #append df[col] to df_temp
                # df_temp = df_temp.append(df[col])
                df_temp[i] = df[col]
            #calculate row wise mean and std for df_temp
            df_stats[col] = df_temp.mean(axis=1)
            df_stats[col+'_std'] = df_temp.std(axis=1)
        
        #save df_stats to csv
        df_stats.to_csv(base_directory + 'stats.csv', index=False)
        print("Saved stats to: ", base_directory + 'stats.csv')
            
gen_stats('/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection/test_run_20240111_211835/my1e-05_rxy0.1_fr0.0001_fa0.00017453292519943296')