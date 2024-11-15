import numpy as np
import open3d as o3d
import os


def npy_to_open3d_and_edit(npy_file, output_file_name):
    # Load the .npy point cloud file
    points = np.load(npy_file)

    # Check if the data is 2D and has exactly 3 columns for X, Y, Z
    if points.shape[1] != 3:
        raise ValueError("The .npy file must contain an Nx3 array for X, Y, Z coordinates.")

    # Convert to an Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Print instructions for interactive editing
    print("=== Open3D Point Cloud Interactive Editing ===")
    print("Instructions:")
    print("  - Rotate: Left-click and drag")
    print("  - Zoom: Scroll the mouse wheel")
    print("  - Pan: Right-click and drag")
    print("  - Select Points: Hold Shift + Left-click and drag a box around points")
    print("  - Crop Selected Points: After selecting, press 'C' to crop the selection")
    print("  - Reset Selection: Shift + Right-click to clear the selection\n")
    print("Close the window when done editing.")

    # Open the point cloud in interactive editing mode
    o3d.visualization.draw_geometries_with_editing([pcd])

    # Ask the user if they want to save the edited point cloud
    save_option = input("Do you want to save the edited point cloud? (y/n): ").strip().lower()

    if save_option == 'y':
        # Extract the directory of the original .npy file
        output_dir = os.path.dirname(npy_file)

        # Construct the full output path in the same directory
        output_file = os.path.join(output_dir, output_file_name)

        # Determine the output format based on file extension
        _, ext = os.path.splitext(output_file)
        ext = ext.lower()

        if ext == '.ply':
            o3d.io.write_point_cloud(output_file, pcd)
            print(f"Edited point cloud saved to {output_file} in PLY format.")

        elif ext == '.xyz':
            # Convert to .xyz format by saving points as a text file
            np.savetxt(output_file, np.asarray(pcd.points), fmt="%.6f", delimiter=" ")
            print(f"Edited point cloud saved to {output_file} in XYZ format.")

        elif ext == '.npy':
            # Convert to .npy format by saving points as a NumPy array
            np.save(output_file, np.asarray(pcd.points))
            print(f"Edited point cloud saved to {output_file} in NPY format.")

        else:
            print("Unsupported file format. Please use '.ply', '.xyz', or '.npy'.")
    else:
        print("Edited point cloud was not saved.")


# Example usage
npy_file = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy"  # Input .npy file
output_file_name = "pcl_edited.npy"  # Name of output file
npy_to_open3d_and_edit(npy_file, output_file_name)

# # Example usage
# npy_file = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy" # Input .npy file
# output_file = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl_edited.ply"  # Output .ply file after editing
# npy_to_open3d_and_edit(npy_file, output_file)
