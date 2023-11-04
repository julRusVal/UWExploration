#include <rbpf_multiagent/rbpf_multiagent.hpp>
#include <rbpf_multiagent/rbpf_par_slam_multiagent_extension.hpp>
#include <ros/callback_queue.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "rbpf_multiagent_node");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh_mb("~");
    ros::CallbackQueue rbpf_queue;
    ros::CallbackQueue mb_queue;
    nh.setCallbackQueue(&rbpf_queue);
    nh_mb.setCallbackQueue(&mb_queue);

    // boost::shared_ptr<RbpfSlam> rbpf_multi(new RbpfMultiagent(nh, nh_mb));
    boost::shared_ptr<RbpfSlam> rbpf_multi(new RbpfSlamMultiExtension(nh, nh_mb));


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
    }

    return 0;
}

//TODO: 
//1. OK - Init pf for all auvs in the rbpf_multiagent files 
//(2.Integrate into launch files)
//3. OK - Create callback for FLS measurements such that it can be forwarded to the update weights step
//4. Define svgp SLAM survey area intop rectangle defined by me 
//   4a. see when path_cb is actually called in single agent- Am I doing the right thing?
//   4b. OK - In RbpfMultiagent::survey_area_cb make sure that it's only called once and not every time a message is published to topic /multi_agent/survey_area
// 5. Continue fixing file structure. 
//   5a OK - Create a fiel called rbpf_par_slam_multiagent_extension which is old rbpf_multiagent. Here is the extension of the single agent rbpf_slam
//   5b Edit rbpf_multiagent to not depend on rbpf_par_slam and instead create three instances of rbpf_par_slam_multiagent_extension, one for neoghbour left, one for neighbour right and one for itself. 
//   5c In rbpf_multiagent_node now call rbpf_multiagent instead of rbpf_par_slam_multiagent_extension and this way each auv has three particle filters. One for itself and one for each neighbour.
//6. init measurement model

