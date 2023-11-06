#pragma once

#include "rbpf_slam/rbpf_par_slam.h"
#include "rbpf_multiagent/rbpf_par_slam_multiagent_extension.hpp"

class RbpfMultiagentNode
{
    public:

    RbpfMultiagentNode(int argc, char** argv);
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_mb;

    boost::shared_ptr<RbpfSlam> setup_rbpf(string base_link_custom_);

};
