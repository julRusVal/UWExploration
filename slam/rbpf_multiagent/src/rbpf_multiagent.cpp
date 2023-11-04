#include <rbpf_multiagent/rbpf_multiagent.hpp>

RbpfMultiagent::RbpfMultiagent(ros::NodeHandle &nh, ros::NodeHandle &nh_mb): RbpfSlam(nh, nh_mb){
    path_sub_.shutdown();
    // The mission waypoints as a path
    // std::string fls_meas_topic;
    bool rbpf_sensor_FLS;
    bool rbpf_sensor_MBES;

    nh_->param<bool>(("rbpf_sensor_FLS"), rbpf_sensor_FLS, true);
    nh_->param<bool>(("rbpf_sensor_MBES"), rbpf_sensor_MBES, false);
    nh_->param<string>(("fls_meas_topic"), fls_meas_topic, "/sim/hugin_0/fls_measurement");
    // nh_mb_->param<float>(("rbpf_period"), rbpf_period_, 0.3);

    if(!rbpf_sensor_MBES){
        // cout << "rbpf_sensor_MBES is off" << endl;
        timer_rbpf_.stop();
    }


    sub_fls_meas_ = nh_->subscribe(fls_meas_topic, rbpf_period_, &RbpfMultiagent::rbpf_update_fls_cb, this);

    inducing_pts_sent = false;
    nh_->param<string>(("survey_area_topic"), survey_area_topic, "/multi_agent/survey_area");
    survey_area_sub_ = nh_->subscribe(survey_area_topic, 1, &RbpfMultiagent::survey_area_cb, this);

}   

// TODO: connect to topic with survey area boundaries
void RbpfMultiagent::survey_area_cb(const visualization_msgs::MarkerArray& marker_array)
{
    if (!inducing_pts_sent) //only send inducing points once, when receiving the survey area the first time.
    {
        inducing_pts_sent = true;
        ROS_INFO("Inside survey_area_cb");
        
        if (marker_array.markers[0].points.size() > 0) //NOTE NACHO: Okay with dot and not ->?? 
        {
            // // This service will start the auv simulation or auv_2_ros nodes to start the mission
            // nh_->param<string>(("synch_topic"), synch_top_, "/pf_synch");
            // // srv_server_ = nh_->advertiseService(synch_top_, &RbpfMultiagent::empty_srv, this);
            // srv_server_ = nh_->advertiseService(synch_top_, &RbpfSlam::empty_srv, (RbpfSlam*) this);
            if (!start_training_){
                ROS_INFO("Sending inducing points");
                std::vector<Eigen::RowVector3f> i_points;
                int ma_size = marker_array.markers[0].points.size() > 0;
                sensor_msgs::PointCloud2 ip_pcloud;
                Eigen::RowVector3f ip;

                auto points = marker_array.markers[0].points;

                for (int i = 0; i < ma_size; i++)
                {
                    ip << points[i].x, points[i].y, points[i].z; //<< is element-wise assignment of the three values to the right into the row vector ip
                    i_points.push_back(ip); //add ip to the end of the vector i_points
                }

                ip_pcloud = pack_cloud(map_frame_, i_points);
                start_training_ = true; // We can start to check for loop closures
                ip_pub_.publish(ip_pcloud);
                // This service will start the auv simulation or auv_2_ros nodes to start the mission
                nh_->param<string>(("synch_topic"), synch_top_, "/pf_synch");
                srv_server_multi_ = nh_->advertiseService(synch_top_, &RbpfMultiagent::empty_srv_multi, this);
            }
        }
        else{
            ROS_WARN("Received empty mission");
        }
    }
    // else{
    //     ROS_INFO("Survey area already received");
    // }

}

bool RbpfMultiagent::empty_srv_multi(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_DEBUG("RBPF ready");
    return true;
}

void RbpfMultiagent::rbpf_update_fls_cb(const auv_2_ros::FlsReading& fls_reading)
{
    // cout << "Received FLS reading" << endl;
    // cout << fls_reading << endl;

    //Here implement measurement model and update weights, similar as RbpfSlam::rbpf_update

}
