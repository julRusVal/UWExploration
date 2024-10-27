import gp_map_training
import numpy as np

'''
Simple example of a multi-agent GP mappping
This approach makes a lot of simplifications to the problem.
This is just a proof of concept.

The number of agent is given, by agent_count
'''


def generate_meeting_pattern(agents, meetings):
    """
    This looks pretty lame but it'll work for now
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

def generate_agent_sub_maps(map_min_x, map_max_x, agent_count):
    """
    This will divide the map into sections of points for the different agents
    This will always divide the map into equal sections along the x axis
    """

    span = map_max_x - map_min_x / agent_count
    boundries = [i * span for i in range(agent_count + 1)]

    agent_sub_maps = []



# Define the mission
agent_count = 3  # nummber of agents
transfer_count = 3  # total number of transfers between agents
pattern = generate_meeting_pattern(agents=agent_count, meetings=transfer_count)

input_type = 'di'
file_path = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy"

complete_map_points = np.load(file_path)

map_mins = np.min(complete_map_points, axis=0)
map_maxs = np.max(complete_map_points, axis=0)

# Map stuff here
map_x_span = map_maxs[0] - map_mins[0]
map_y_span = map_maxs[1] - map_mins[1]










