
class RbpfSlamMultiExtension: public RbpfSlam
{
    public:

    ros::Subscriber survey_area_sub_;
    // inline RbpfSlamMultiExtension(){};

    RbpfSlamMultiExtension(ros::NodeHandle &nh, ros::NodeHandle &nh_mb, string &base_link_custom_);

    void survey_area_cb(const visualization_msgs::MarkerArray& marker_array); //& sign is used to denote a reference parameter. Avoids copying full variable
    void rbpf_update_fls_cb(const auv_2_ros::FlsReading& fls_reading);

    ros::Subscriber sub_fls_meas_;

    // float rbpf_period_;
    string fls_meas_topic;
    string survey_area_topic;
    bool inducing_pts_sent;
    ros::ServiceServer srv_server_multi_;

    bool empty_srv_multi(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    // void update_particles_weights(float &range, float &angle)



};
