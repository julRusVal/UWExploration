#ifndef AUV_2_ROS_HPP
#define AUV_2_ROS_HPP 

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cereal/archives/binary.hpp>
#include <pcl/io/obj_io.h>

#include "data_tools/std_data.h"

#include "auv_2_ros/cxxopts.hpp"
#include "auv_2_ros/submaps.hpp"
#include <auv_model/MbesSimAction.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>



using namespace Eigen;
using namespace std;

class BathymapConstructor{

public:
    BathymapConstructor(std::string node_name, ros::NodeHandle &nh);
    ~BathymapConstructor();

    void init(const boost::filesystem::path auv_path);

    void publishMeas(int ping_num);

    void broadcastTf(const ros::TimerEvent &);

    void publishOdom(Vector3d odom_ping_i, Vector3d euler);
   
    void addMiniCar(std::string &mini_name);

    void initMiniFrames(std::vector<Vector3d> &minis_poses);

    void synchCB(const std_msgs::BoolConstPtr& synch_msg);

private:
    std::string node_name_;
    ros::NodeHandle* nh_;

    ros::Publisher ping_pub_;
    ros::Publisher sim_ping_pub_;
    ros::Publisher test_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher enable_pub_;
    ros::Subscriber pf_synch_sub_;

    actionlib::SimpleActionClient<auv_model::MbesSimAction>* ac_;

    tf::TransformListener tflistener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    tf2_ros::TransformBroadcaster br_;
    std::vector<geometry_msgs::TransformStamped> pings_tf_;

    Eigen::Isometry3d world_mbes_tf_;
    Eigen::Isometry3d world_map_tf_;
    Eigen::Isometry3d map_odom_tf_;
    Eigen::Isometry3d map_mbes_tf_;
//    Eigen::Isometry3d mini_tf_;
    Eigen::Isometry3d mbes_base_mat_;
    
    SubmapsVec maps_gt_;
    SubmapsVec traj_pings_;

    ros::Time time_now_, time_prev_;
    tf::StampedTransform tf_mbes_base_;
    // Hack to fix init of overnight 2020 beginning of mission
    // tf::StampedTransform tf_odom_map_;
    tf::Transform tf_odom_map_;
    //
    geometry_msgs::TransformStamped prev_base_link_;
    geometry_msgs::TransformStamped new_base_link_;
    geometry_msgs::TransformStamped map_odom_tfmsg_;
    geometry_msgs::TransformStamped world_map_tfmsg_;
    std::vector<geometry_msgs::TransformStamped> map_mini_tfmsgs_;

    std::string world_frame_, map_frame_, odom_frame_, base_frame_, mbes_frame_, mini_frame_, synch_name_;

    bool start_replay_, survey_finished_, change_detection_, add_mini_;
    int ping_cnt_;
    int ping_total_;
    int beams_num_;
    int first_ping_;
    int last_ping_;
};


#endif
