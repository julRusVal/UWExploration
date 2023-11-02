#include <rbpf_multiagent/rbpf_multiagent.hpp>
#include <ros/callback_queue.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "rbpf_multiagent_node");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh_mb("~");
    ros::CallbackQueue rbpf_queue;
    ros::CallbackQueue mb_queue;
    nh.setCallbackQueue(&rbpf_queue);
    nh_mb.setCallbackQueue(&mb_queue);

    boost::shared_ptr<RbpfSlam> rbpf_multi(new RbpfMultiagent(nh, nh_mb));

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
//5. init measurement model

