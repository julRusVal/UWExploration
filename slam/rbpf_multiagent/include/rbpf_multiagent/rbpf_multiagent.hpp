// HEADER DEFINING THE RBPF_SLAM CLASS
#pragma once

// #include "rbpf_slam/rbpf_particle.h"
#include "rbpf_slam/rbpf_par_slam.h"
#include <rbpf_multiagent/rbpf_par_slam_multiagent_extension.hpp>


// Standard dependencies
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <set>
#include <thread>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <slam_msgs/MinibatchTrainingAction.h>
#include <slam_msgs/MinibatchTrainingGoal.h>
#include <slam_msgs/MinibatchTrainingResult.h>
#include <slam_msgs/SamplePosteriorAction.h>
#include <slam_msgs/ManipulatePosteriorAction.h>
#include <slam_msgs/PlotPosteriorAction.h>
#include <slam_msgs/Resample.h>

#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

//#include <auv_2_ros/FlsReading.h>
#include <auv_model/FlsReading.h>


#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <random>
#include <vector>
#include <future>


using namespace std;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

// typedef actionlib::SimpleActionClient<slam_msgs::SamplePosteriorAction> Client;

class RbpfMultiagent //public RbpfSlam
{
    public:

    // ros::Subscriber survey_area_sub_;
    // inline RbpfMultiagent(){};

    RbpfMultiagent(ros::NodeHandle &nh, ros::NodeHandle &nh_mb);

    ros::NodeHandle nh;
    ros::NodeHandle nh_mb;
    boost::shared_ptr<RbpfSlamMultiExtension> rbpf;


    boost::shared_ptr<RbpfSlamMultiExtension> setup_rbpf(string base_link_custom_);

    // void survey_area_cb(const visualization_msgs::MarkerArray& marker_array); //& sign is used to denote a reference parameter. Avoids copying full variable
    // void rbpf_update_fls_cb(const auv_2_ros::FlsReading& fls_reading);

    // ros::Subscriber sub_fls_meas_;

    // // float rbpf_period_;
    // string fls_meas_topic;
    // string survey_area_topic;
    // bool inducing_pts_sent;
    // // string namespace_;
    // ros::ServiceServer srv_server_multi_;

    // bool empty_srv_multi(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    // // void update_particles_weights(float &range, float &angle)





};
