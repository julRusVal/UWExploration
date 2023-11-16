#include <rbpf_multiagent/rbpf_multiagent.hpp>
#include <rbpf_multiagent/rbpf_par_slam_multiagent_extension.hpp>
// #include <rbpf_multiagent/rbpf_multiagent_node.hpp>
#include <ros/callback_queue.h>


// ros::NodeHandle nh("~");
// ros::NodeHandle nh_mb("~");

int main(int argc, char** argv){
// RbpfMultiagentNode::RbpfMultiagentNode(int argc, char** argv){
    ros::init(argc, argv, "rbpf_multiagent_node");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh_mb("~");
    ros::CallbackQueue rbpf_queue;
    ros::CallbackQueue mb_queue;
    nh.setCallbackQueue(&rbpf_queue);
    nh_mb.setCallbackQueue(&mb_queue);




    // boost::shared_ptr<RbpfMultiagent> rbpf_multi(new RbpfMultiagent(nh, nh_mb));
    //TEST -----
    std::string namespace_;
    int num_auvs_;
    nh.param<string>(("namespace"), namespace_, "hugin_0");
    nh.param<int>(("num_auvs"), num_auvs_, 1);
    int auv_id = namespace_.back() - '0'; // ASCII code for 0 is 48, 1 is 49, etc. https://sentry.io/answers/char-to-int-in-c-and-cpp/#:~:text=C%20and%20C%2B%2B%20store%20characters,the%20value%20of%20'0'%20.
    string self_base_link = "hugin_" + std::to_string(auv_id) + "/base_link";
    boost::shared_ptr<RbpfSlamMultiExtension> rbpf_multi(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link));

    

    // if (auv_id == 0 && num_auvs_ > 1)
    // {
    //     ROS_INFO("Inside RbpfMultiagent constructor: auv_id == 0");
    //     //dynamically assign self base link as string of hugin_+str(auv_id)+/base_link
    //     string self_base_link = "hugin_" + std::to_string(auv_id);// + "/base_link";
    //     string neighbour_right_base_link = "hugin_" + std::to_string(auv_id+1);// + "/base_link";
    //     //This works: CHATGPT LOOK HERE
    //     // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link)); 

    //     // //This doesn't: CHATGPT LOOK HERE
    //     // rbpf_self = RbpfMultiagent::setup_rbpf(self_base_link);

    //     // rbpf_right = RbpfMultiagent::setup_rbpf(neighbour_right_base_link);
    //     boost::shared_ptr<RbpfSlamMultiExtension> rbpf_right(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_right_base_link));
        
    // }
    // else if (auv_id == num_auvs_-1 && num_auvs_ > 1)
    // {
    //     ROS_INFO("Inside RbpfMultiagent constructor: auv_id == num_auvs_-1");
    //     string self_base_link = "hugin_" + std::to_string(auv_id);// + "/base_link";
    //     string neighbour_left_base_link = "hugin_" + std::to_string(auv_id-1);// + "/base_link";
    //     // rbpf_self = RbpfMultiagent::setup_rbpf(self_base_link);
    //     // rbpf_left = RbpfMultiagent::setup_rbpf(neighbour_left_base_link);
    //     // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link));
    //     boost::shared_ptr<RbpfSlamMultiExtension> rbpf_left(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_left_base_link));
    // }
    // else
    // {
    //     ROS_INFO("Inside RbpfMultiagent constructor: auv_id is neither 0 nor num_auvs_-1");
    //     string self_base_link = "hugin_" + std::to_string(auv_id);// + "/base_link";
    //     string neighbour_left_base_link = "hugin_" + std::to_string(auv_id-1);// + "/base_link";
    //     string neighbour_right_base_link = "hugin_" + std::to_string(auv_id+1);// + "/base_link";
    //     // rbpf_self = RbpfMultiagent::setup_rbpf(self_base_link);
    //     // rbpf_left = RbpfMultiagent::setup_rbpf(neighbour_left_base_link);
    //     // rbpf_right = RbpfMultiagent::setup_rbpf(neighbour_right_base_link);
    //     // boost::shared_ptr<RbpfSlamMultiExtension> rbpf_self(new RbpfSlamMultiExtension(nh, nh_mb, self_base_link));
    //     boost::shared_ptr<RbpfSlamMultiExtension> rbpf_left(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_left_base_link));
    //     boost::shared_ptr<RbpfSlamMultiExtension> rbpf_right(new RbpfSlamMultiExtension(nh, nh_mb, neighbour_right_base_link));
    // }
    //----------------
   

    // Spinner for AUV interface callbacks
    ros::AsyncSpinner spinner_rbpf(1000, &rbpf_queue); //asyncspinner pt1 (https://roboticsbackend.com/ros-asyncspinner-example/)
    spinner_rbpf.start(); //asyncspinner pt2
    
    // Spinner for SVGPs minibatch callbacks
    int pc;
    nh.param<int>(("particle_count"), pc, 10);
    // Option 1
    ros::AsyncSpinner spinner_mb(1000, &mb_queue);
    spinner_mb.start();

    // Option 2
    // ros::MultiThreadedSpinner spinner_mb(pc);
    // spinner_mb.spin(&mb_queue);

    // Option 3
    // while (nh_mb.ok())
    // {
    //     mb_queue.callAvailable();
    // }

    ros::waitForShutdown(); //asyncspinner pt3
    if(!ros::ok()){
        rbpf_multi.reset();
        // rbpf_self.reset();
        // rbpf_right.reset();
        // rbpf_left.reset();
    }

    return 0;
}

//TODO: 
//1. Finish prediction of neighbour particles, 
// 1a. OK - MAke the neighbour particles move based on own odometry
// 1b. OK - Make the neighbour particles move in the right direction, see continue here in rbpf_par_slam_multiagent_extension.cpp
//2. init measurement model
    //2a. OK - Determinewhich neighbour is infront of the FLS
    //2b Calculate estimated mesurement (z_hat) for each neighbour particle and for each particle
    // - OK - Hugin 0 rbpf slam node seems to crash after pressing enter in the terminal. I don't think it has to do with frontal_neighbour_id_ being nullptr anymore. This is smoething that broke, when I was changing that...
    // - Right now, range_hat and angle_hat are not correct. The transformation matrix *oN2o_mat_ptr is correct, checked manually. See CONTINUE HERE in ...extension.cpp.
    //2c. Calculate particle wweights (importance factors)
    //2d. Resample particles
