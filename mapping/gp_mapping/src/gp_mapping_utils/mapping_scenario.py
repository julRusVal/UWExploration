import numpy as np
from typing import Iterable, Optional, Sequence
import open3d as o3d


def generate_agent_sub_maps(map_mins, map_maxs, agent_count, survey_points, movement_axis='y'):
    """
    This will divide the map into sections of points for the different agents
    This will always divide the map into equal sections along the x axis
    """

    if movement_axis.lower() == 'y':
        # If movement along the y-axis, divide the map along the x-axis
        dimension_ind = 0
    else:
        # If movement along the x-axis, divide the map along the y-axis
        dimension_ind = 1

    map_min = map_mins[dimension_ind]
    map_max = map_maxs[dimension_ind]

    agent_span = (map_max - map_min) / agent_count
    boundaries = [map_min + i * agent_span for i in range(agent_count + 1)]

    agent_sub_maps = []

    for i in range(agent_count):
        # Filter points within the current boundary for agent i
        mask = (survey_points[:, dimension_ind] >= boundaries[i]) & (survey_points[:, dimension_ind] < boundaries[i + 1])
        agent_points = survey_points[mask]
        agent_sub_maps.append(agent_points)

    return agent_sub_maps

def generate_transfer_pairs(agents, meetings):
    """
    This looks pretty lame but it'll work for now.

    """
    if meetings == 0:
        return []

    pairs = []
    start = 0
    # end = agents - 2  # this accouns for 0 indexing and that we want to include the next agent
    end = agents - 1  # Confused how the above ever worked??
    current = 0
    step = 1
    for _ in range(meetings):
        pairs.append([current, current + step])
        current += step
        if current == end or current == start:
            step *= -1

    return pairs

def generate_transfer_coordinates(map_mins, map_maxs, transfer_count, movement_axis='y'):
    """
    This assumes the transfers all take place equally spaced along the movement axis.
    """
    if transfer_count == 0:
        return []

    if movement_axis.lower() == 'y':
        # If movement along the y-axis, divide the map along the y-axis
        dimension_ind = 1
    else:
        # If movement along the x-axis, divide the map along the x-axis
        dimension_ind = 0

    map_min = map_mins[dimension_ind]
    map_max = map_maxs[dimension_ind]
    transfer_step_size = (map_max - map_min) / transfer_count
    transfer_coordinates = [map_min + (i + 1) * transfer_step_size for i in range(transfer_count)]

    return transfer_coordinates


def generate_grid_inducing_points(map_mins: Sequence[float],
                                  map_maxs: Sequence[float],
                                  num_points: int,
                                  polygon: Optional[Iterable[Sequence[float]]] = None) -> np.ndarray:
    """
    Deterministically generate a grid of inducing points that cover the map bounds.

    Parameters
    ----------
    map_mins, map_maxs : Sequence[float]
        Lower/upper bounds for (x, y, ...) coordinates. Only the first two entries are used.
    num_points : int
        Desired number of inducing points.
    polygon : Iterable[(float, float)], optional
        Optional polygon describing an irregular operating region. Points outside the
        polygon are discarded. When fewer than three vertices are provided, the polygon
        is ignored.

    Returns
    -------
    np.ndarray
        Array of shape (num_points, 2) containing inducing point coordinates.
    """
    if num_points <= 0:
        raise ValueError("num_points must be positive.")

    bounds_min = np.asarray(map_mins, dtype=float)
    bounds_max = np.asarray(map_maxs, dtype=float)
    if bounds_min.shape[0] < 2 or bounds_max.shape[0] < 2:
        raise ValueError("map_mins and map_maxs must contain at least x and y components.")

    width = bounds_max[0] - bounds_min[0]
    height = bounds_max[1] - bounds_min[1]
    if width <= 0 or height <= 0:
        raise ValueError("Invalid bounds: max values must be greater than min values.")

    aspect = width / height
    cols = int(np.ceil(np.sqrt(num_points * aspect)))
    rows = int(np.ceil(num_points / cols))

    xs = np.linspace(bounds_min[0], bounds_max[0], cols)
    ys = np.linspace(bounds_min[1], bounds_max[1], rows)
    grid_x, grid_y = np.meshgrid(xs, ys)
    grid_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))

    if polygon is not None:
        polygon = list(polygon)
        if len(polygon) >= 3:
            region = Path(polygon)
            mask = region.contains_points(grid_points)
            grid_points = grid_points[mask]

    if grid_points.size == 0:
        raise ValueError("Polygon masking removed all inducing points.")

    if grid_points.shape[0] > num_points:
        idx = np.linspace(0, grid_points.shape[0] - 1, num_points, dtype=int)
        grid_points = grid_points[idx]
    elif grid_points.shape[0] < num_points:
        repeats = int(np.ceil(num_points / grid_points.shape[0]))
        grid_points = np.vstack([grid_points] * repeats)[:num_points]

    return grid_points

def generate_n_grid_points(map_mins: Sequence[float],
                           map_maxs: Sequence[float],
                           num_points: int,
                           polygon: Optional[Iterable[Sequence[float]]] = None) -> np.ndarray:
    
    # This is a method to generate grid points from the given mins and maxs
    # 
    # NOTE: This method is not working right now

    mins = np.asarray(map_mins, dtype=float)
    maxs = np.asarray(map_maxs, dtype=float)
    if mins.size < 2 or maxs.size < 2:
        raise ValueError("map_mins and map_maxs must contain at least [x, y].")

    minx, miny = mins[0], mins[1]
    maxx, maxy = maxs[0], maxs[1]

    # Rectangle corner coordinates (x, y, z=0), ordered CCW
    corners_locations = np.array([
        [minx, miny, 1.0],
        [minx, maxy, 0.0],
        [maxx, maxy, 0.0],
        [maxx, miny, 1.0],
    ], dtype=float)

    # Create Open3D point cloud and tetrahedral mesh
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(corners_locations)
    tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(
        pcd)
    alpha = 1000000000.0
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 
                                                                         alpha, 
                                                                         tetra_mesh, 
                                                                         pt_map)
    
    pcl = mesh.sample_points_poisson_disk(number_of_points=int(num_points))

    return np.asarray(pcl.points)[:, 0:2]

