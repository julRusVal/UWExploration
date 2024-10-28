# import gp_map_training
# from gp import SVGP
import numpy as np

'''
Simple example of a multi-agent GP mappping
This approach makes a lot of simplifications to the problem.
This is just a proof of concept.

The number of agent is given, by agent_count
'''


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

    agent_span = map_max - map_min / agent_count
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

    pairs = []
    start = 0
    end = agents - 2  # this accouns for 0 indexing and that we want to include the next agent
    current = 0
    step = 1
    for meeting_i in range(meetings):
        pairs.append([current, current + 1])
        current += step
        if current == end or current == start:
            step *= -1

    return pairs

def generate_transfer_coordinates(map_mins, map_maxs, transfer_count, movement_axis='y'):
    """
    This assumes the transfers all take place equally spaced along the movement axis.
    """

    if movement_axis.lower() == 'y':
        # If movement along the y-axis, divide the map along the y-axis
        dimension_ind = 1
    else:
        # If movement along the x-axis, divide the map along the x-axis
        dimension_ind = 0

    map_min = map_mins[dimension_ind]
    map_max = map_maxs[dimension_ind]
    transfer_step_size = map_max - map_min / transfer_count
    transfer_coordinates = [map_min + (i + 1) * transfer_step_size for i in range(transfer_count)]

    return transfer_coordinates

def train_svgp_simple(survey_points, covariances=None, verbose=False):
    """
    Stripped down version of the training example shown in gp_map_training

    """

    inputs = survey_points[:, [0, 1]]
    targets = survey_points[:, 2]

    # initialise GP with 1000 inducing points
    gp = SVGP(400)
    gp.fit(inputs, targets, covariances=covariances, n_samples=1000,
           max_iter=1000, learning_rate=1e-1, rtol=1e-12, n_window=2000,
           auto=False, verbose=verbose)

    return gp

# Define the mission
agent_count = 3  # nummber of agents
transfer_count = 3  # total number of transfers between agents
movement_axis = 'y'

# dataset
input_type = 'di'
file_path = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy"
file_path = "/Users/julian/KTH/Projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy"

# Load data as a Nx3 array of survey points, [x, y, z]
complete_survey_points = np.load(file_path)
map_mins = np.min(complete_survey_points, axis=0)
map_maxs = np.max(complete_survey_points, axis=0)

# Agent stuff here
agent_points = generate_agent_sub_maps(map_mins=map_mins,
                                       map_maxs=map_maxs,
                                       agent_count=agent_count,
                                       survey_points=complete_survey_points,
                                       movement_axis=movement_axis)

models = [[] for _ in range(agent_count)]

# Mission stuff here
"""
Mission in this example are very simplified, being only defined by the number of agents and the number of transfers.
The 
"""
transfer_pairs = generate_transfer_pairs(agents=agent_count, meetings=transfer_count)
transfer_coordinates = generate_transfer_coordinates(map_mins=map_mins, map_maxs=map_maxs,
                                                     transfer_count=transfer_count, movement_axis=movement_axis)

for transfer_id, (transfer_pair, transfer_coordinate) in enumerate(zip(transfer_pairs, transfer_coordinates)):
    print(transfer_id)
    print(transfer_pair)
    print(transfer_coordinates)

    # Check direction of movement
    if movement_axis.lower() == 'y':
        # If movement along the y-axis, divide the map along the x-axis
        dimension_ind = 0
    else:
        # If movement along the x-axis, divide the map along the y-axis
        dimension_ind = 1

    # Agent A
    agent_a_ind = transfer_pair[0]
    agent_a_survey_points = agent_points[agent_a_ind]
    agent_a_available_mask = agent_a_survey_points[:, dimension_ind] <= transfer_coordinate
    agent_a_available_data = agent_a_survey_points[agent_a_available_mask]
    # TODO figure out how to include previous models if available
    agent_a_gp = train_svgp_simple(survey_points=agent_a_survey_points,covariances=None, verbose=True)

    # Agent B
    agent_b_ind = transfer_pair[1]
    agent_b_survey_points = agent_points[agent_b_ind]
    agent_b_available_mask = agent_b_survey_points[:, dimension_ind] <= transfer_coordinate
    agent_b_available_data = agent_b_survey_points[agent_b_available_mask]
    # TODO figure out how to include previous models if available
    agent_b_gp = train_svgp_simple(survey_points=agent_b_survey_points, covariances=None, verbose=True)

    # Transfer model to the 'server' agent

    # aggregate

    # Pass aggregated model to the 'client' agent



# test_gp = train_svgp_simple(survey_points=agent_points[0],
#                             covariances=None, verbose=True)

print("Done!?!")











