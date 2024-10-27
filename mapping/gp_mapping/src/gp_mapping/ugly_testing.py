import gp_map_training
# from convergence import ExpMAStoppingCriterion

"""
Attempting to train a GP using the target dataset
"""

input_type = 'di'
file_path = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy"

gp_map_training.train_svgp(input_type, file_path)
