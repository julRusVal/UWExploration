#include <rbpf_multiagent/rbpf_multiagent.hpp>

RbpfMultiagent::RbpfMultiagent(ros::NodeHandle &nh, ros::NodeHandle &nh_mb): RbpfSlam(nh, nh_mb){

    // The mission waypoints as a path
    std::string fls_meas_topic;
    bool rbpf_sensor_FLS;
    bool rbpf_sensor_MBES;

    nh_->param<bool>(("rbpf_sensor_FLS"), rbpf_sensor_FLS, true);
    nh_->param<bool>(("rbpf_sensor_MBES"), rbpf_sensor_MBES, false);
    nh_->param<string>(("fls_meas_topic"), fls_meas_topic, "/sim/hugon_0/fls_measurement");
    nh_mb_->param<float>(("rbpf_period"), rbpf_period_, 0.3);

    if(!rbpf_sensor_MBES){
        // cout << "rbpf_sensor_MBES is off" << endl;
        timer_rbpf_.stop();
    }


    sub_fls_meas_ = nh_->subscribe(fls_meas_topic, rbpf_period_, &RbpfMultiagent::rbpf_update_fls_cb, this);


    // nh_->param<string>(("area_topic"), area_topic, "/waypoints");
    // area_sub_ = nh_->subscribe(area_topic, 1, &RbpfMultiagent::area_cb, this);
}

// TODO: connect to topic with survey area boundaries
// void RbpfMultiagent::area_cb(const nav_msgs::PathConstPtr& wp_path)
// {
//     if (wp_path->poses.size() > 0)
//     {
//         // This service will start the auv simulation or auv_2_ros nodes to start the mission
//         nh_->param<string>(("synch_topic"), synch_top_, "/pf_synch");
//         // srv_server_ = nh_->advertiseService(synch_top_, &RbpfMultiagent::empty_srv, this);
//         srv_server_ = nh_->advertiseService(synch_top_, &RbpfSlam::empty_srv, (RbpfSlam*) this);
//     }
//     else{
//         ROS_WARN("Received empty mission");
//     }

// }

void RbpfMultiagent::rbpf_update_fls_cb(const auv_2_ros::FlsReading& fls_reading)
{
    cout << "Received FLS reading" << endl;
    cout << fls_reading << endl;

    //Here implement measurement model and update weights, similar as RbpfSlam::rbpf_update

}
