#include <rbpf_multiagent/rbpf_multiagent.hpp>
#include <rbpf_multiagent/rbpf_par_slam_multiagent_extension.hpp>

RbpfMultiagent::RbpfMultiagent(ros::NodeHandle &nh, ros::NodeHandle &nh_mb){
    ROS_INFO("Inside RbpfMultiagent constructor");
    std::string namespace_;
    int num_auvs_;
    // boost::shared_ptr<RbpfSlamMultiExtension> setup_rbpf(std::string base_link_custom);
    // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self;
    // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_right;
    // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_left;
    nh.param<string>(("namespace"), namespace_, "hugin_0");
    nh.param<int>(("num_auvs"), num_auvs_, 1);

    // take the last character of the namespace string and convert it to an integer
    int auv_id = namespace_.back() - '0'; // ASCII code for 0 is 48, 1 is 49, etc. https://sentry.io/answers/char-to-int-in-c-and-cpp/#:~:text=C%20and%20C%2B%2B%20store%20characters,the%20value%20of%20'0'%20.
    
    // string self_base_link = "hugin_" + std::to_string(auv_id) + "/base_link";
    // rbpf_self = RbpfMultiagent::setup_rbpf(self_base_link);
    string self_base_link = "hugin_" + std::to_string(auv_id) + "/base_link";
   
    boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link)); 

    if (auv_id == 0)
    {
        ROS_INFO("Inside RbpfMultiagent constructor: auv_id == 0");
        //dynamically assign self base link as string of hugin_+str(auv_id)+/base_link
        string self_base_link = "hugin_" + std::to_string(auv_id) + "/base_link";
        string neighbour_right_base_link = "hugin_" + std::to_string(auv_id+1) + "/base_link";
        //This works: CHATGPT LOOK HERE
        // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link)); 

        // //This doesn't: CHATGPT LOOK HERE
        // rbpf_self = RbpfMultiagent::setup_rbpf(self_base_link);
        if (num_auvs_ > 1)
        {
            // rbpf_right = RbpfMultiagent::setup_rbpf(neighbour_right_base_link);
            // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_right(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_right_base_link));
        }
    }
    else if (auv_id == num_auvs_-1)
    {
        ROS_INFO("Inside RbpfMultiagent constructor: auv_id == num_auvs_-1");
        string self_base_link = "hugin_" + std::to_string(auv_id) + "/base_link";
        string neighbour_left_base_link = "hugin_" + std::to_string(auv_id-1) + "/base_link";
        // rbpf_self = RbpfMultiagent::setup_rbpf(self_base_link);
        // rbpf_left = RbpfMultiagent::setup_rbpf(neighbour_left_base_link);
        // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link));
        // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_left(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_left_base_link));
    }
    else
    {
        ROS_INFO("Inside RbpfMultiagent constructor: auv_id is neither 0 nor num_auvs_-1");
        string self_base_link = "hugin_" + std::to_string(auv_id) + "/base_link";
        string neighbour_left_base_link = "hugin_" + std::to_string(auv_id-1) + "/base_link";
        string neighbour_right_base_link = "hugin_" + std::to_string(auv_id+1) + "/base_link";
        // rbpf_self = RbpfMultiagent::setup_rbpf(self_base_link);
        // rbpf_left = RbpfMultiagent::setup_rbpf(neighbour_left_base_link);
        // rbpf_right = RbpfMultiagent::setup_rbpf(neighbour_right_base_link);
        // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link));
        // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_left(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_left_base_link));
        // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_right(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_right_base_link));
    }

    // while (true)
    // {
    //     if(!ros::ok()){
    //     // rbpf_multi.reset();
    //     rbpf_self.reset();
    //     rbpf_right.reset();
    //     rbpf_left.reset();
    // }
    // }
}
boost::shared_ptr<RbpfSlamMultiExtension> RbpfMultiagent::setup_rbpf(string base_link_custom_){
    // ros::NodeHandle nh("~");
    // ros::NodeHandle nh_mb("~");
    // ros::CallbackQueue rbpf_queue;
    // ros::CallbackQueue mb_queue;
    // nh.setCallbackQueue(&rbpf_queue);
    // nh_mb.setCallbackQueue(&mb_queue);
    // boost::shared_ptr<RbpfSlamMultiExtension> rbpf
    // boost::shared_ptr<RbpfSlam> rbpf_multi(new RbpfMultiagent(nh, nh_mb));
    boost::shared_ptr<RbpfSlamMultiExtension> rbpf(new RbpfSlamMultiExtension(nh, nh_mb, base_link_custom_)); // CONTINUE HERE: This doesn't seem to work. It doesn't create an instance of RbpfSlamMultiExtension
    return rbpf;
}
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
    // nh_->param<string>(("namespace"), namespace_, "hugin_0");
    // nh_->param<int>(("num_auvs"), num_auvs_, 1);

    // // take the last character of the namespace string and convert it to an integer
    // int auv_id = namespace_.back() - '0'; // ASCII code for 0 is 48, 1 is 49, etc. https://sentry.io/answers/char-to-int-in-c-and-cpp/#:~:text=C%20and%20C%2B%2B%20store%20characters,the%20value%20of%20'0'%20.
    // if (auv_id == 0)
    // {
    //     boost::shared_ptr<RbpfSlam> rbpf(new RbpfSlamMultiExtension(nh, nh_mb, "hugin_0/base_link"));
    // }

    
    
    // // boost::shared_ptr<RbpfSlam> rbpf_multi(new RbpfSlamMultiExtension(nh, nh_mb, "hugin_0/base_link"));
    // boost::shared_ptr<RbpfSlam> rbpf_multi_left(new RbpfSlamMultiExtension(nh, nh_mb, "hugin_1/base_link"));


  

