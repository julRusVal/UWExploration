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
//1. Calculate particle weights (importance factors)
//   - Determine the covariance matrix (gp_var and fls_sigma), combine them to a SIGMA matrix
//   - OK caluclate the log likelihood, then return the likelihood by exp. 
//   - OK store each instance of Weight (weight for each pair)
//   - OK double check calculation of likelihood, according to me it's should be different
//2.  Resample particles
//   - OK Make the code compile
//   - OK see TODO, handle exceptions 
//   - OK Handle crashing code (see CONTINUE HERE in ...extension.cpp)
//   - OK Handle too many particles being regenerated. > pc_ or >pcn_
//   - Implement the effective (N_eff) value of the weights determining when to resample and when not to resample

// 3. Add noise in FLS measurememt
// 4. Add plots
// 5. Add external estimated measurements

// FOR OI:
// 1. OK No spinning
// 2. OK Convergence particles, find bug. It happens in the resampling of the self and neighbour particles. Ony prediction keeps the self particles dead on teh GT- 
// 3. OK Normalized pt clouds all auvs
// 4. OK FLS BEAMS only 4
// 5. semi-OK (fix is to no display survey area just before cklicking enter) Fixblue rectangle for survey area
// 6. Lower priority: Cool plots
// 7. Record video
// AFTER OI:
// - See TODO above
// - External estimated measurement, comms
// - OK Add noise after resampling