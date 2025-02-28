import os
import yaml

# === File Management ===
def remove_files_in_directory(directory_path, verbose: bool = False):
    if not os.path.exists(directory_path):
        print(f"Directory does not exist: {directory_path}")
        return

    # Iterate over all items in the directory
    for item in os.listdir(directory_path):
        item_path = os.path.join(directory_path, item)

        # Check if it's a file and remove it
        if os.path.isfile(item_path):
            os.unlink(item_path)  # Delete the file
            if verbose:
                print(f"Deleted file: {item_path}")
        else:
            if verbose:
                print(f"Skipped directory or symlink: {item_path}")
    if verbose:
        print("All files in the directory have been removed.")

# === YAML ===
def load_yaml(file_path):
    """Loads a YAML file. Returns None if the file does not exist or is invalid."""
    if not os.path.exists(file_path):
        print(f"File does not exist: {file_path}")
        return None

    try:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)  # Use safe_load for secure parsing
            return data
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None
    
# === File Paths ===
def find_file_path(directory: str, filename: str) -> str:
    """
    Searches for a file with the given filename in the specified directory and its subdirectories.

    Args:
        directory (str): The directory to search in.
        filename (str): The name of the file to search for.

    Returns:
        str: The full path to the file if found, otherwise None.
    """
    for root, dirs, files in os.walk(directory):
        if filename in files:
            file_path = os.path.join(root, filename)
            print(f"File found: {file_path}")
            return file_path
    
    print(f"File not found: {filename}")
    return None