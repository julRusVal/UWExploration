// HEADER DEFINING THE RBPF_SLAM CLASS
#pragma once

// #include "rbpf_slam/rbpf_particle.h"
#include "rbpf_slam/rbpf_par_slam.h"
#include "weight.hpp"

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

#include <auv_2_ros/FlsReading.h>



#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <random>
#include <vector>
#include <future>
#include <cmath>


using namespace std;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

// typedef actionlib::SimpleActionClient<slam_msgs::SamplePosteriorAction> Client;

class RbpfSlamMultiExtension: public RbpfSlam
{
    public:

    ros::Subscriber survey_area_sub_;
    ros::Subscriber wp_counter_sub_;
    ros::Subscriber odom_sub_neighbours_;
    ros::Publisher vis_pub_left_;
    ros::Publisher vis_pub_right_;
    ros::Publisher z_hat_pub_;
    ros::Timer timer_neighbours_rviz_;
    ros::ServiceServer srv_server_multi_;



    // inline RbpfSlamMultiExtension(){};
    // RbpfSlamMultiExtension();
    RbpfSlamMultiExtension(ros::NodeHandle &nh, ros::NodeHandle &nh_mb, string &base_link_custom_);

    void survey_area_cb(const visualization_msgs::MarkerArray& marker_array); //& sign is used to denote a reference parameter. Avoids copying full variable
    void wp_counter_cb(const std_msgs::Int32& wp_counter_msg);
    void rbpf_update_fls_cb(const auv_2_ros::FlsReading& fls_reading);
    void update_rviz_cb(const ros::TimerEvent &);
    void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg);

    bool empty_srv_multi(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    double compute_weight(const Eigen::Vector2f z, const Eigen::Vector2f z_hat)


    void setup_neighbours();
    void update_frontal_neighbour_id();

    std::vector<RbpfParticle> init_particles_of(int agent_id);
    geometry_msgs::PoseArray particles_2_pose_array(const int& id, const std::vector<RbpfParticle>& particles);
    void pub_markers(const geometry_msgs::PoseArray& array_msg, const ros::Publisher& publisher);
    void predict(nav_msgs::Odometry odom_t, float dt, std::vector<RbpfParticle>& particles,std::vector<std::thread>& pred_threads_vec);
    void update_particles_weights(const float &range, const float &angle, const int *fls_neighbour_id);
    void pub_estimated_measurement_to_rviz(const Eigen::Vector3f& start, const Eigen::Vector3f& end, const std::string frame_id);



    ros::Subscriber sub_fls_meas_;
    ros::Subscriber odom_sub_neigh_;
    // float rbpf_period_;
    string fls_meas_topic;
    string survey_area_topic;
    string z_hat_viz_top_;
    string namespace_;
    bool inducing_pts_sent;
    bool particle_sets_instantiated_ = false;
    std::vector<RbpfParticle> particles_left_;
    std::vector<RbpfParticle> particles_right_;
    int pcn_;
    int wp_counter_;
    int* frontal_neighbour_id_ = nullptr;
    int frontal_direction_;
    int* auv_id_ = nullptr;
    int* auv_id_left_ = nullptr;
    int* auv_id_right_ = nullptr;
    string vehicle_model_;
    int num_auvs_;
    double time_neigh_;
    double old_time_neigh_;
    std::vector<std::thread> pred_threads_vec_neigh_left_;
    std::vector<std::thread> pred_threads_vec_neigh_right_;
    // tf::StampedTransform oL2o_tf_; //tf from odom left to odom self
    // tf::StampedTransform oR2o_tf_; //tf from odom right to odom self
    Eigen::Matrix4f oL2o_mat_;
    Eigen::Matrix4f oR2o_mat_;

    // std::vector<float> fls_meas_;

    // nav_msgs::Odometry odom_latest_neigh_;


    
    // void update_particles_weights(float &range, float &angle)



};
