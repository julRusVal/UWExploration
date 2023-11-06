#include <rbpf_multiagent/rbpf_multiagent.hpp>
#include <rbpf_multiagent/rbpf_par_slam_multiagent_extension.hpp>

RbpfMultiagent::RbpfMultiagent(ros::NodeHandle &nh, ros::NodeHandle &nh_mb){
    // path_sub_.shutdown(); //shutdown the path subscriber to allow the survey area define the first inducing points.
    // // The mission waypoints as a path
    // // std::string fls_meas_topic;
    // bool rbpf_sensor_FLS;
    // bool rbpf_sensor_MBES;

    // nh_->param<bool>(("rbpf_sensor_FLS"), rbpf_sensor_FLS, true);
    // nh_->param<bool>(("rbpf_sensor_MBES"), rbpf_sensor_MBES, false);
    // nh_->param<string>(("fls_meas_topic"), fls_meas_topic, "/sim/hugin_0/fls_measurement");
    // // nh_mb_->param<float>(("rbpf_period"), rbpf_period_, 0.3);

    // if(!rbpf_sensor_MBES){
    //     // cout << "rbpf_sensor_MBES is off" << endl;
    //     timer_rbpf_.stop();
    // }


    // sub_fls_meas_ = nh_->subscribe(fls_meas_topic, rbpf_period_, &RbpfMultiagent::rbpf_update_fls_cb, this);

    // inducing_pts_sent = false;
    // nh_->param<string>(("survey_area_topic"), survey_area_topic, "/multi_agent/survey_area");
    // survey_area_sub_ = nh_->subscribe(survey_area_topic, 1, &RbpfMultiagent::survey_area_cb, this);
    
    //PSEUDO CODE CONTINUES HERE!!
    //neighbour_left = new RbpfSlamMultiExtension (AUV left) #Holds state vector of neighbour left
    //neighbour_right = new RbpfSlamMultiExtension (AUV right) #Holds state vector of neighbour right
    //itself = new RbpfSlamMultiExtension (AUV itself) #Holds state vector of itself
    nh_->param<string>(("namespace"), namespace_, "hugin_0");
    nh_->param<int>(("num_auvs"), num_auvs_, 1);

    // take the last character of the namespace string and convert it to an integer
    int auv_id = namespace_.back() - '0'; // ASCII code for 0 is 48, 1 is 49, etc. https://sentry.io/answers/char-to-int-in-c-and-cpp/#:~:text=C%20and%20C%2B%2B%20store%20characters,the%20value%20of%20'0'%20.
    if (auv_id == 0)
    {
        boost::shared_ptr<RbpfSlam> rbpf(new RbpfSlamMultiExtension(nh, nh_mb, "hugin_0/base_link"));
    }

    
    
    // boost::shared_ptr<RbpfSlam> rbpf_multi(new RbpfSlamMultiExtension(nh, nh_mb, "hugin_0/base_link"));
    boost::shared_ptr<RbpfSlam> rbpf_multi_left(new RbpfSlamMultiExtension(nh, nh_mb, "hugin_1/base_link"));


}   

