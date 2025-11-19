#!/usr/bin/env python3

# General imports
import os
import warnings
import time
from pathlib import Path
import ast
import copy
from collections import OrderedDict

# Data processing imports
import gpytorch.constraints
import matplotlib.pyplot as plt
import numpy as np, tqdm
import open3d as o3d

# GP imports
import torch
# from gpytorch.models import VariationalGP, ExactGP
# from gpytorch.variational import CholeskyVariationalDistribution, VariationalStrategy
# from gpytorch.means import ConstantMean
# from gpytorch.kernels import MaternKernel, ScaleKernel, GaussianSymmetrizedKLKernel, InducingPointKernel
# from gpytorch.likelihoods import GaussianLikelihood
# from gpytorch.distributions import MultivariateNormal
# from gpytorch.mlls import VariationalELBO, PredictiveLogLikelihood, ExactMarginalLogLikelihood
# from gpytorch.test.utils import least_used_cuda_device
# import gpytorch.settings
# from gp_mapping.convergence import ExpMAStoppingCriterion

"""
Script for testing FL using state dictionary from the MA scenario.
This might differe from the formats found in the ma example in the gp_mapping package.
"""

AVG_KEYS = [['model', 'mean.raw_constant'],
            ['model', 'cov.raw_outputscale'],
            ['model', 'cov.base_kernel.raw_lengthscale'],
            ['likelihood','noise_covar.raw_noise']]

def fed_avg_state_dicts(state_dicts):
    # Elements of interest
    # 'variational_strategy.inducing_points'
    # 'variational_strategy.variational_params_initialized'
    # 'variational_strategy.updated_strategy'
    # 'variational_strategy._variational_distribution.variational_mean'
    # 'variational_strategy._variational_distribution.chol_variational_covar'
    # 'mean.raw_constant'  # Used
    # 'cov.raw_outputscale'  # Used
    # 'cov.base_kernel.raw_lengthscale'  # Used
    # 'likelihood.noise_covar.raw_noise'  # Used

    # this expects a list of ordered dicts
    # copy_keys = ['variational_strategy.updated_strategy']

    # avg_keys = [['model', 'mean.raw_constant'],
    #             ['model', 'cov.raw_outputscale'],
    #             ['model', 'cov.base_kernel.raw_lengthscale'],
    #             ['likelihood','noise_covar.raw_noise']]

    # These might be of some use but are currently not inserted into the agggregated output
    # min_keys = ['cov.base_kernel.raw_lengthscale_constraint.lower_bound',
    #             'cov.raw_outputscale_constraint.lower_bound',
    #             'likelihood.noise_covar.raw_noise_constraint.lower_bound']
    
    # max_keys = ['cov.base_kernel.raw_lengthscale_constraint.upper_bound',
    #             'cov.raw_outputscale_constraint.upper_bound',
    #             'likelihood.noise_covar.raw_noise_constraint.upper_bound']
    
    # Aggregated dictionary is based on the first state dictionary
    aggregated_dict = copy.deepcopy(state_dicts[0])

    # aggregated_dict = OrderedDict()

    # for key in copy_keys:
    #     value = state_dicts[0][key]
    #     aggregated_dict[key] = value

    for key in AVG_KEYS:

        values = [state_dict[key[0]][key[1]] for state_dict in state_dicts]

        aggregated_dict[key[0]][key[1]] = sum(values)/len(values)


    return aggregated_dict

def fed_avg_state_dicts_test(current_model, input_models, testing=False):
    # Elements of interest
    # 'variational_strategy.inducing_points'
    # 'variational_strategy.variational_params_initialized'
    # 'variational_strategy.updated_strategy'
    # 'variational_strategy._variational_distribution.variational_mean'
    # 'variational_strategy._variational_distribution.chol_variational_covar'
    # 'mean.raw_constant'  # Used
    # 'cov.raw_outputscale'  # Used
    # 'cov.base_kernel.raw_lengthscale'  # Used
    # 'likelihood.noise_covar.raw_noise'  # Used

    
    aggregated_model = copy.deepcopy(current_model)

    if testing:
        return aggregated_model

    for key in AVG_KEYS:

        values = [input_model[key[0]][key[1]] for input_model in input_models.values()]
        values.append(current_model[key[0]][key[1]])

        new_value = sum(values)/len(values)

        aggregated_model[key[0]][key[1]] = new_value


    return aggregated_dict

def print_loaded_model_info(load_obj: OrderedDict, print_values=False):
    # print("Model state dictionary keys:")
    # print(model.state_dict().keys())

    # print("Model state dictionary values:")
    # print(model.state_dict().values())

    # print("Model state dictionary:")
    # print(model.state_dict())

    # print("Model state dictionary items:")
    # print(model.state_dict().items

    """
    Assumes the loaded object is produced by rbpf_svgp.py


    """
    valid_keys = ['model', 'likelihood', 'mll', 'opt']

    # Check if all keys are present
    if not all(key in load_obj.keys() for key in valid_keys):
        raise ValueError("Loaded object does not contain all necessary keys")

    model_dict = load_obj['model']
    # likelihood_dict = load_obj['likelihood']  # Not used for now
    # mll_dict = load_obj['mll']
    # opt_dict = load_obj['opt']

    # Keys for model_dict
    model_keys = ['variational_strategy.inducing_points',
                  'variational_strategy.variational_params_initialized',
                  'variational_strategy.updated_strategy',
                  'variational_strategy._variational_distribution.variational_mean',
                  'variational_strategy._variational_distribution.chol_variational_covar',
                  'mean.raw_constant',
                  'cov.raw_outputscale',
                  'cov.base_kernel.raw_lengthscale',
                  'likelihood.noise_covar.raw_noise']
    
    for test_key in model_keys:
        if test_key not in model_dict.keys():
            print(f"Key {test_key} not found in model dictionary")
            continue
        print(f"Key {test_key} found in model dictionary")
        test_value = model_dict[test_key]
        if isinstance(test_value, torch.Tensor):
            print(f"Value is a tensor of shape {test_value.shape}")

    if print_values:
        for key in AVG_KEYS:
            if key[0] not in load_obj.keys():
                continue
            if key[1] not in load_obj[key[0]].keys():
                continue
            print(f"Key: {key} Value: {load_obj[key[0]][key[1]]}")

# Defines paths to .pth files
file_0 ="/home/sam/auv_ws/src/UWExploration/utils/uw_tests/datasets/asko/svgp/hugin_0_online_svgp.pth"
file_1 ="/home/sam/auv_ws/src/UWExploration/utils/uw_tests/datasets/asko/svgp/hugin_1_online_svgp.pth"

# Load these files using torch.load()
state_dict_0 = torch.load(file_0, weights_only=False)
state_dict_1 = torch.load(file_1, weights_only=False)

# Determine the structure of these objects
# print("State dictionary 0 keys:")
# print(state_dict_0.keys())

input_model_dict = {}
input_model_dict[1] = state_dict_1

print(f"Dictionary info: state_dict_0")
print_loaded_model_info(state_dict_0, True)
print(f"Dictionary info: state_dict_1")
print_loaded_model_info(state_dict_1, True) 

print("State dictionary 1 keys:")
print(state_dict_1.keys())

aggregated_dict = fed_avg_state_dicts([state_dict_0, state_dict_1])
aggregated_dict_2 = fed_avg_state_dicts_test(state_dict_0, input_model_dict)

print(f"Dictionary info: aggregated_dict")
print_loaded_model_info(aggregated_dict, True)
print_loaded_model_info(aggregated_dict_2, True)

# test some FL methods for combining these objects
