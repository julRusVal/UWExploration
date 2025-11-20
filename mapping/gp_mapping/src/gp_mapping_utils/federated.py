from collections import OrderedDict

import numpy as np


def extract_svgp_model_hypers(model):
    """
    extract the needed parameters from a given model
    """

    mean = model.mean.raw_constant.item()
    cov = model.cov.raw_outputscale.item()

    # Sometimes the parameter is [1,2] and others times it is [2,]
    # cov_kernel_lengthscale = model.cov.base_kernel.raw_lengthscale.tolist()[0]
    if model.cov.base_kernel.raw_lengthscale.shape[0] == 1:
        cov_kernel_lengthscale = model.cov.base_kernel.raw_lengthscale.tolist()[0]
    else:
        cov_kernel_lengthscale = model.cov.base_kernel.raw_lengthscale.tolist()[:2]

    likelihood_noise = model.likelihood.noise_covar.raw_noise.item()

    hyperparameters = [mean, cov]
    hyperparameters.extend(cov_kernel_lengthscale)  # Flatten into a list
    hyperparameters.append(likelihood_noise)

    return hyperparameters

def extract_svgp_model_inducing_info(model):
    """
    Extract the inducing point from a given model.
    These inducing points can be used to train a new model.
    """

    inducing_points = model.variational_strategy.inducing_points.detach()
    inducing_means = model.variational_strategy.variational_distribution.mean.detach()

    return inducing_points, inducing_means

def federated_average(hyperparameters):
    """
    Simple federated average
    """

    if len(hyperparameters) == 0:
        return []

    hyperparameter_array = np.array(hyperparameters)

    # n_agents = hyperparameter_array.shape[0]

    hyperparameter_avg = np.mean(hyperparameter_array, axis=0)

    return hyperparameter_avg

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
    copy_keys = ['variational_strategy.updated_strategy']

    avg_keys = ['mean.raw_constant',
                'cov.raw_outputscale',
                'cov.base_kernel.raw_lengthscale',
                'likelihood.noise_covar.raw_noise']

    # These might be of some use but are currently not inserted into the agggregated output
    min_keys = ['cov.base_kernel.raw_lengthscale_constraint.lower_bound',
                'cov.raw_outputscale_constraint.lower_bound',
                'likelihood.noise_covar.raw_noise_constraint.lower_bound']
    max_keys = ['cov.base_kernel.raw_lengthscale_constraint.upper_bound',
                'cov.raw_outputscale_constraint.upper_bound',
                'likelihood.noise_covar.raw_noise_constraint.upper_bound']

    aggregated_dict = OrderedDict()

    for key in copy_keys:
        value = state_dicts[0][key]
        aggregated_dict[key] = value

    for key in avg_keys:

        values = [state_dict[key] for state_dict in state_dicts]

        aggregated_dict[key] = sum(values)/len(values)


    return aggregated_dict