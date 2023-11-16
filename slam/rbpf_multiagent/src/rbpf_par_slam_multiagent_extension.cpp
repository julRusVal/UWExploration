#include <rbpf_multiagent/rbpf_par_slam_multiagent_extension.hpp>


RbpfSlamMultiExtension::RbpfSlamMultiExtension(ros::NodeHandle &nh, ros::NodeHandle &nh_mb, string &base_link_custom_): RbpfSlam(nh, nh_mb){
    ROS_INFO("Inside RbpfSlamMultiExtension constructor");
    path_sub_.shutdown(); //shutdown the path subscriber to allow the survey area define the first inducing points.
    // base_frame_ = base_link_custom_ + "/base_link";
    // // nh_->param<string>(("mbes_link"), mbes_frame_, "mbes_link");
    // // nh_->param<string>(("base_link"), base_frame_, "base_link");
    // // nh_->param<string>(("odom_frame"), odom_frame_, "odom");
    // mbes_frame_ = base_link_custom_ + "/mbes_link";
    // odom_frame_ = base_link_custom_ + "/odom";
    // ROS_INFO("base_frame_ = %s", base_frame_.c_str());
    // The mission waypoints as a path
    // std::string fls_meas_topic;
    bool rbpf_sensor_FLS;
    bool rbpf_sensor_MBES;

    nh_->param<bool>(("rbpf_sensor_FLS"), rbpf_sensor_FLS, true);
    nh_->param<bool>(("rbpf_sensor_MBES"), rbpf_sensor_MBES, false);
    nh_->param<string>(("fls_meas_topic"), fls_meas_topic, "/sim/hugin_0/fls_measurement");
    nh_->param<string>(("namespace"), namespace_, "hugin_0");

    nh_->param<int>(("particle_count_neighbours"), pcn_, 5);
    nh_->param<float>(("measurement_std"), meas_std_, 0.01);
    nh_->param("init_covariance", init_cov_, vector<float>());
    nh_->param("motion_covariance", motion_cov_, vector<float>());
    nh_->param<int>(("n_beams_mbes"), beams_real_, 512);
    nh_->param<string>(("map_frame"), map_frame_, "map");
    nh_->param<string>(("vehicle_model"), vehicle_model_, "hugin");
    // nh_mb_->param<float>(("rbpf_period"), rbpf_period_, 0.3);

    if(!rbpf_sensor_MBES){
        // cout << "rbpf_sensor_MBES is off" << endl;
        timer_rbpf_.stop();
    }


    sub_fls_meas_ = nh_->subscribe(fls_meas_topic, rbpf_period_, &RbpfSlamMultiExtension::rbpf_update_fls_cb, this);

    inducing_pts_sent = false;
    nh_->param<string>(("survey_area_topic"), survey_area_topic, "/multi_agent/survey_area");
    survey_area_sub_ = nh_->subscribe(survey_area_topic, 1, &RbpfSlamMultiExtension::survey_area_cb, this);
    
    //Rviz visualization
    std::string rbpf_markers_left_top;
    std::string rbpf_markers_right_top;
    nh_->param<string>(("markers_left_top"), rbpf_markers_left_top, "/markers_left");
    nh_->param<string>(("markers_right_top"), rbpf_markers_right_top, "/markers_right");
    vis_pub_left_ = nh_->advertise<visualization_msgs::MarkerArray>(rbpf_markers_left_top, 0);
    vis_pub_right_ = nh_->advertise<visualization_msgs::MarkerArray>(rbpf_markers_right_top, 0);
    // Timer for updating RVIZ
    nh_->param<float>(("rviz_period"), rviz_period_, 0.3);
    if(rviz_period_ != 0.){
        timer_neighbours_rviz_ = nh_->createTimer(ros::Duration(rviz_period_), &RbpfSlamMultiExtension::update_rviz_cb, this, false);
    }

    std::string wp_counter_topic;
    nh_->param<string>(("wp_counter_topic"), wp_counter_topic, "/wp_counter");
    wp_counter_sub_ = nh_->subscribe(wp_counter_topic, 1, &RbpfSlamMultiExtension::wp_counter_cb, this);


    //Setup neighbours
    RbpfSlamMultiExtension::setup_neighbours();

    //Neighbour prediction
    nh_->param<string>(("odometry_topic"), odom_top_, "odom");
    odom_sub_neigh_ = nh_->subscribe(odom_top_, 100, &RbpfSlamMultiExtension::odom_callback, this);
     // Start timing now
    time_neigh_ = ros::Time::now().toSec();
    old_time_neigh_ = ros::Time::now().toSec();
}   

// TODO: connect to topic with survey area boundaries
void RbpfSlamMultiExtension::survey_area_cb(const visualization_msgs::MarkerArray& marker_array)
{
    // ROS_INFO("IN MULTI EXTENSIONS!!!!");
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
                srv_server_multi_ = nh_->advertiseService(synch_top_, &RbpfSlamMultiExtension::empty_srv_multi, this);
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

void RbpfSlamMultiExtension::wp_counter_cb(const std_msgs::Int32& wp_counter_msg)
{   
    wp_counter_ = wp_counter_msg.data;
    ROS_INFO("Updating wp_counter_ to value %d", wp_counter_);
    RbpfSlamMultiExtension::update_frontal_neighbour_id();
    ROS_INFO("namespace_ = %s", namespace_.c_str());
    if (frontal_neighbour_id_)
    {
        ROS_INFO("frontal_neighbour_id_ = %d", *frontal_neighbour_id_);
    }
    else
    {
        ROS_INFO("frontal_neighbour_id_ = nullptr");
    }
    ROS_INFO("frontal_direction_ = %d", frontal_direction_);

}

void RbpfSlamMultiExtension::update_frontal_neighbour_id()
{   
    if (wp_counter_ % 2 == 0) //even wp and not the first
    {
    if (wp_counter_ != 0)
    {
        frontal_direction_ = -frontal_direction_;
    }
    if (frontal_direction_ == 1)
    {
        if (auv_id_right_)
        {
            frontal_neighbour_id_ = new int(*auv_id_right_);
        }
        else
        {
            // ROS_WARN("No frontal neighbour id available");
            frontal_neighbour_id_ = nullptr;
        }
        // frontal_neighbour_id_ = new int(*auv_id_right_);
    }
    else
    {
        if (auv_id_left_)
        {
            frontal_neighbour_id_ = new int(*auv_id_left_);
        }
        else
        {
            // ROS_WARN("No frontal neighbour id available");
            frontal_neighbour_id_ = nullptr;
        }
        // frontal_neighbour_id_ = new int(*auv_id_left_);
    }   
    }
    else if(wp_counter_ % 2 != 0)
    {
        // ROS_INFO("nullptr");
        frontal_neighbour_id_ = nullptr;
    }

}

bool RbpfSlamMultiExtension::empty_srv_multi(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_DEBUG("RBPF ready");
    return true;
}

void RbpfSlamMultiExtension::rbpf_update_fls_cb(const auv_2_ros::FlsReading& fls_reading)
{
    if (wp_counter_ % 2 == 0) //Only if facing anoher auv, not when travelling in the same direction
    {
        // fls_meas_(0) = fls_reading.range;
        // fls_meas_(1) = fls_reading.angle;
        // fls_meas_(2) = frontal_neighbour_id_;
        // RbpfSlamMultiExtension::update_particles_weights(fls_reading.range, fls_reading.angle, frontal_neighbour_id_);
        // ROS_INFO("namespace_ = %s inside rbpf_update_fls_cb", namespace_.c_str());
        if (frontal_neighbour_id_)
        {
        RbpfSlamMultiExtension::update_particles_weights(fls_reading.range.data, fls_reading.angle.data, frontal_neighbour_id_);
        }
        else
        {
            ROS_WARN("No frontal neighbour id available");
        }

    }
    // int neighbour_id = RbpfSlamMultiExtension::identify_frontal_neighbour_id();
    // cout << "Received FLS reading" << endl;
    // cout << fls_reading << endl;

    //Here implement measurement model and update weights, similar as RbpfSlam::rbpf_update

}

void RbpfSlamMultiExtension::update_particles_weights(const float &range, const float &angle, const int *fls_neighbour_id)
{
    // ROS_INFO("namespace_ = %s", namespace_.c_str());
    // ROS_INFO("Updating particle weights using FLS measurement");
    // ROS_INFO("fls_neighbour_id = %d", *fls_neighbour_id);

    const std::vector<RbpfParticle>* particles_neighbour = nullptr;
    const Eigen::Matrix4f* oN2o_mat_ptr = nullptr; //Transformation matrix odom neighbour to odom self

    // ROS_INFO("0");
    if (auv_id_left_)
    {   
        // ROS_INFO("auv_id_left_ = %d", *auv_id_left_);
        if (*fls_neighbour_id == *auv_id_left_)
        {
            // ROS_INFO("1");
            particles_neighbour = &particles_left_;
            oN2o_mat_ptr = &oL2o_mat_;
        }
    }
    if (auv_id_right_)
    {
        // ROS_INFO("auv_id_right_ = %d", *auv_id_right_);
        if (*fls_neighbour_id == *auv_id_right_)
        {
            // ROS_INFO("2");
            particles_neighbour = &particles_right_;
            oN2o_mat_ptr = &oR2o_mat_;
        }
    }
    // ROS_INFO("mid");
    if (particles_neighbour && oN2o_mat_ptr) // Check if particles_neighbour is not nullptr
    {
        // ROS_INFO("3");

        for (const RbpfParticle& particle_m : particles_) //particle m
        {
            // ROS_INFO("4");

            for (const RbpfParticle& n_particle_phi : *particles_neighbour) //neighbour particle phi
            {
                // ROS_INFO("5");
                Eigen::Vector4f n_point_Nodom;//HOMOGENOUS neighbour point in neighbour odom frame
                Eigen::Vector3f n_point; //neighbour point in self odom frame
                Eigen::Vector3f s_point; //self point in self odom frame
                const float PI = std::acos(-1.0f);
                n_point_Nodom.head(3) = n_particle_phi.p_pose_.head(3); 
                n_point_Nodom(3) = 1;
                //*oN2o_mat_ptr has been checked manually and is correct.
                n_point = ((*oN2o_mat_ptr) * n_point_Nodom).head(3); 
                s_point = particle_m.p_pose_.head(3); //CONTINUE HERE
                float heading = particle_m.p_pose_(5); // in self odom frame
                float range_hat = (n_point.head(2) - s_point.head(2)).norm();
                float phi = std::atan2(n_point(1) - s_point(1), n_point(0) - s_point(0)) - PI/2;
                float angle_hat = -heading - phi - PI/2;
                ROS_INFO("namespace_ = %s", namespace_.c_str());
                ROS_INFO("range = %f", range);
                ROS_INFO("range_hat = %f", range_hat);
                ROS_INFO("angle = %f", angle);
                ROS_INFO("angle_hat = %f", angle_hat);
                ROS_INFO("oN2o_mat_ptr contents:");

                const Eigen::Matrix4f& oN2o_mat = *oN2o_mat_ptr; // Dereference the pointer to get the matrix
                for (int i = 0; i < 4; ++i) {
                    ROS_INFO_STREAM(" | " << oN2o_mat(i, 0) << " " << oN2o_mat(i, 1) << " " << oN2o_mat(i, 2) << " " << oN2o_mat(i, 3));
                }
                // Use particle_m and particle_phi here
            }
        }
    }
    else
    {
        ROS_WARN("No neighbour particles or transformation matrix available");
    }
    

}


void RbpfSlamMultiExtension::setup_neighbours()
{
    // std::string namespace_;
    // int num_auvs_;
    // nh_->param<string>(("namespace"), namespace_, "hugin_0");
    nh_->param<int>(("num_auvs"), num_auvs_, 1);
    int auv_id = namespace_.back() - '0'; // ASCII code for 0 is 48, 1 is 49, etc. https://sentry.io/answers/char-to-int-in-c-and-cpp/#:~:text=C%20and%20C%2B%2B%20store%20characters,the%20value%20of%20'0'%20.
    auv_id_ = new int(auv_id);
    if (auv_id == 0 && num_auvs_ > 1)
    {
        // ROS_INFO("Inside RbpfMultiagent constructor: auv_id == 0");
        particles_right_ = RbpfSlamMultiExtension::init_particles_of(auv_id+1);
        auv_id_right_ = new int(auv_id+1);
        
    }
    else if (auv_id == num_auvs_-1 && num_auvs_ > 1)
    {
        // ROS_INFO("Inside RbpfMultiagent constructor: auv_id == num_auvs_-1");
        particles_left_ = RbpfSlamMultiExtension::init_particles_of(auv_id-1);
        auv_id_left_ = new int(auv_id-1);
    }
    else
    {
        // ROS_INFO("Inside RbpfMultiagent constructor: auv_id is neither 0 nor num_auvs_-1");
        particles_left_ = RbpfSlamMultiExtension::init_particles_of(auv_id-1);
        particles_right_ = RbpfSlamMultiExtension::init_particles_of(auv_id+1);
        auv_id_left_ = new int(auv_id-1);
        auv_id_right_ = new int(auv_id+1);
    }
    particle_sets_instantiated_ = true;

    //Initiate first FLS direction
    if (num_auvs_ % 2 == 0)
    {
        if (*auv_id_ % 2 == 0)
        {
            frontal_direction_ = 1;
        }
        else
        {
            frontal_direction_ = -1;
        }
    }
    else
    {
        if (*auv_id_ % 2 == 0)
        {
            frontal_direction_ = -1;
        }
        else
        {
            frontal_direction_ = 1;
        }
    }
    // Store transforms between auv odom and its two neighbours (if they exist)
    if (auv_id_left_)
    {
        try 
        {
            string odom_frame_left = vehicle_model_ + "_" + std::to_string(*auv_id_left_) + "/odom";
            tf::StampedTransform oL2o_tf; //tf from odom left to odom self

            tfListener_.waitForTransform(odom_frame_, odom_frame_left, ros::Time(0), ros::Duration(300.0));
            tfListener_.lookupTransform(odom_frame_, odom_frame_left, ros::Time(0), oL2o_tf);
            pcl_ros::transformAsMatrix(oL2o_tf, oL2o_mat_);
            
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("ERROR: Could not lookup transform from odom_left to odom");
        }
    }
    if (auv_id_right_)
    {
        try
        {
            string odom_frame_right = vehicle_model_ + "_" + std::to_string(*auv_id_right_) + "/odom";
            tf::StampedTransform oR2o_tf; //tf from odom right to odom self
            tfListener_.waitForTransform(odom_frame_, odom_frame_right, ros::Time(0), ros::Duration(300.0));
            tfListener_.lookupTransform(odom_frame_, odom_frame_right, ros::Time(0), oR2o_tf);
            pcl_ros::transformAsMatrix(oR2o_tf, oR2o_mat_);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("ERROR: Could not lookup transform from odom_right to odom");
        }
    }
}

std::vector<RbpfParticle> RbpfSlamMultiExtension::init_particles_of(int agent_id)
{
    std::vector<RbpfParticle> particles;
    // string base_frame = "hugin_" + std::to_string(agent_id) + "/base_link";
    // string mbes_frame = "hugin_" + std::to_string(agent_id) + "/mbes_link";
    // string odom_frame = "hugin_" + std::to_string(agent_id) + "/odom";

    // nh_->param<int>(("particle_count_neighbours"), pcn_, 5);
    // nh_->param<float>(("measurement_std"), meas_std_, 0.01);
    // nh_->param("init_covariance", init_cov_, vector<float>());
    // nh_->param("motion_covariance", motion_cov_, vector<float>());
    // nh_->param<int>(("n_beams_mbes"), beams_real_, 512);
    // nh_->param<string>(("map_frame"), map_frame_, "map");
    // nh_->param<string>(("vehicle_model"), vehicle_model_, "hugin");

    string base_frame = vehicle_model_ + "_" + std::to_string(agent_id) + "/base_link";
    string mbes_frame = vehicle_model_ + "_" + std::to_string(agent_id) + "/mbes_link";
    string odom_frame = vehicle_model_ + "_" + std::to_string(agent_id) + "/odom";


    tf2_ros::TransformListener tf_listener(tf_buffer_);
    try
    {
        ROS_DEBUG("Waiting for transforms");
        auto asynch_1 = std::async(std::launch::async, [this,base_frame, mbes_frame]
                                       { return tf_buffer_.lookupTransform(base_frame, mbes_frame,
                                                                           ros::Time(0), ros::Duration(60.)); });

        auto asynch_2 = std::async(std::launch::async, [this,odom_frame]
                                       { return tf_buffer_.lookupTransform(map_frame_, odom_frame,
                                                                           ros::Time(0), ros::Duration(60.)); });

        tf::StampedTransform mbes_tf;
        geometry_msgs::TransformStamped tfmsg_mbes_base = asynch_1.get();
        tf::transformMsgToTF(tfmsg_mbes_base.transform, mbes_tf);
        pcl_ros::transformAsMatrix(mbes_tf, base2mbes_mat_);

        tf::StampedTransform m2o_tf;
        geometry_msgs::TransformStamped tfmsg_map_odom = asynch_2.get();
        tf::transformMsgToTF(tfmsg_map_odom.transform, m2o_tf);
        pcl_ros::transformAsMatrix(m2o_tf, m2o_mat_);

        ROS_INFO("Transforms locked - RBPF node");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("ERROR: Could not lookup transform from base_link to mbes_link");
    }

    // Initialize the particles on top of vehicle 
    tf::StampedTransform o2b_tf;
    tfListener_.waitForTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(300.0));
    tfListener_.lookupTransform(odom_frame, base_frame, ros::Time(0), o2b_tf);
    double x, y, z, roll_o2b, pitch_o2b, yaw_o2b;
    x = o2b_tf.getOrigin().x();
    y = o2b_tf.getOrigin().y();
    z = o2b_tf.getOrigin().z();
    o2b_tf.getBasis().getRPY(roll_o2b, pitch_o2b, yaw_o2b);
    init_p_pose_(0)= x;
    init_p_pose_(1)= y;
    init_p_pose_(2)= z;
    init_p_pose_(3)= roll_o2b;
    init_p_pose_(4)= pitch_o2b;
    init_p_pose_(5)= yaw_o2b;

    // Create particles
    for (int i=0; i<pcn_; i++){
        particles.emplace_back(RbpfParticle(beams_real_, pcn_, i, base2mbes_mat_, m2o_mat_, init_p_pose_,
                                            init_cov_, meas_std_, motion_cov_));
    }
    return particles;
}

void RbpfSlamMultiExtension::update_rviz_cb(const ros::TimerEvent &)
{
    if (auv_id_left_) {
        geometry_msgs::PoseArray array_msg;
        array_msg = particles_2_pose_array(*auv_id_left_, particles_left_);
        pub_markers(array_msg, vis_pub_left_);
    }
    if (auv_id_right_) {
        geometry_msgs::PoseArray array_msg;
        array_msg = particles_2_pose_array(*auv_id_right_, particles_right_);
        pub_markers(array_msg, vis_pub_right_);
    }

}

geometry_msgs::PoseArray RbpfSlamMultiExtension::particles_2_pose_array(const int& id, const std::vector<RbpfParticle>& particles)
{
    geometry_msgs::PoseArray array_msg;
    array_msg.header.frame_id = vehicle_model_ + "_" + std::to_string(id) + "/odom";
    // array_msg.header.frame_id = vehicle_model_ + "_" + std::to_string(*auv_id_) + "/odom";

    array_msg.header.stamp = ros::Time::now();

    for (int i=0; i<pcn_; i++){
        geometry_msgs::Pose pose_i;
        pose_i.position.x = particles.at(i).p_pose_(0);
        pose_i.position.y = particles.at(i).p_pose_(1);
        pose_i.position.z = particles.at(i).p_pose_(2);

        Eigen::AngleAxisf rollAngle(particles.at(i).p_pose_(3), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(particles.at(i).p_pose_(4), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(particles.at(i).p_pose_(5), Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q = rollAngle * pitchAngle * yawAngle;
        pose_i.orientation.x = q.x();
        pose_i.orientation.y = q.y();
        pose_i.orientation.z = q.z();
        pose_i.orientation.w = q.w();

        array_msg.poses.push_back(pose_i);
    }
    // average_pose(array_msg);
    
    // if(false){
    //     pf_pub_.publish(array_msg);
    // }
    // else{
    //     pub_markers(array_msg);
    // }
    return array_msg;

}

void RbpfSlamMultiExtension::pub_markers(const geometry_msgs::PoseArray& array_msg, const ros::Publisher& publisher)
{
    visualization_msgs::MarkerArray markers;
    int i = 0;
    string frame_id = array_msg.header.frame_id;
    for (geometry_msgs::Pose pose_i : array_msg.poses)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "markers";
        marker.id = i;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose_i.position.x;
        marker.pose.position.y = pose_i.position.y;
        marker.pose.position.z = pose_i.position.z;
        marker.pose.orientation.x = pose_i.orientation.x;
        marker.pose.orientation.y = pose_i.orientation.y;
        marker.pose.orientation.z = pose_i.orientation.z;
        marker.pose.orientation.w = pose_i.orientation.w;
        marker.scale.x = 0.001;
        marker.scale.y = 0.001;
        marker.scale.z = 0.001;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.mesh_resource = "package://hugin_description/mesh/Hugin_big_meter.dae";
        markers.markers.push_back(marker);
        i++;
    }

    publisher.publish(markers);
}

void RbpfSlamMultiExtension::odom_callback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    // ROS_INFO("namespace_ = %s", namespace_.c_str());
    // if (frontal_neighbour_id_)
    // {
    //     ROS_INFO("frontal_neighbour_id_ = %d", *frontal_neighbour_id_);
    // }
    // else
    // {
    //     ROS_INFO("frontal_neighbour_id_ = nullptr");
    // }
    // ROS_INFO("frontal_direction_ = %d", frontal_direction_);
    // ROS_INFO("odom_callback");
    if (particle_sets_instantiated_)
    {
        
        time_neigh_ = odom_msg->header.stamp.toSec();

        if(mission_finished_ != true)
        {
            // Motion prediction
            if (time_neigh_ > old_time_neigh_)
            {
                // Added in the MBES CB to synch the DR steps with the pings log
                nav_msgs::Odometry odom_cp = *odom_msg; // local copy
                float dt = float(time_ - old_time_);
                // ROS_INFO("namespace_ = %s", namespace_.c_str());
                if (auv_id_left_)
                {
                    // ROS_INFO("Predicting left neighbour");
                    RbpfSlamMultiExtension::predict(odom_cp, dt,particles_left_, pred_threads_vec_neigh_left_);
                }
                if (auv_id_right_)
                {
                    // ROS_INFO("Predicting right neighbour");
                    RbpfSlamMultiExtension::predict(odom_cp, dt,particles_right_, pred_threads_vec_neigh_right_);
                }
            }
        }
        old_time_neigh_ = time_neigh_;
    }
}

void RbpfSlamMultiExtension::predict(nav_msgs::Odometry odom_t, float dt, std::vector<RbpfParticle>& particles, std::vector<std::thread>& pred_threads_vec)
{
    // ROS_INFO("Inside predict");
    // Multithreading
    // auto t1 = high_resolution_clock::now();
    // Eigen::VectorXf noise_vec(6, 1);
    // ROS_INFO("frame_id = %s", odom_t.header.frame_id.c_str());
    // ROS_INFO("auv_id_ = %d", *auv_id_);
    // Angular vel #CONTINUE HERE: The auvs don't all start looking forward. Find a way to determine how the own odometry should decide the odometry of the neightbour. Look into map frame? 

    Eigen::Vector3f vel_rot = Eigen::Vector3f(odom_t.twist.twist.angular.x,
                                              odom_t.twist.twist.angular.y,
                                              -odom_t.twist.twist.angular.z);

    // Linear vel
    Eigen::Vector3f vel_p = Eigen::Vector3f(odom_t.twist.twist.linear.x,
                                            -odom_t.twist.twist.linear.y,
                                            odom_t.twist.twist.linear.z);

    // Depth (read directly)
    float depth = odom_t.pose.pose.position.z;
    // ROS_INFO("size of particles = %d", particles.size());
    // ROS_INFO("vel_rot = %f, %f, %f", vel_rot(0), vel_rot(1), vel_rot(2));
    // ROS_INFO("vel_p = %f, %f, %f", vel_p(0), vel_p(1), vel_p(2));
    for(int i = 0; i < pcn_; i++)
    { //CONTINUE HERE: before was &particles.at(i). Builds well, but throws error when running. 
        // ROS_INFO("dt = %f", dt);
        // ROS_INFO("BEFORE pose of particle %d = %f, %f, %f", i, particles.at(i).p_pose_(0), particles.at(i).p_pose_(1), particles.at(i).p_pose_(2));
        pred_threads_vec.emplace_back(std::thread(&RbpfParticle::motion_prediction, 
                                    std::ref(particles.at(i)), std::ref(vel_rot), std::ref(vel_p),
                                    depth, dt, std::ref(rng_)));
        // ROS_INFO("AFTER pose of particle %d = %f, %f, %f", i, particles.at(i).p_pose_(0), particles.at(i).p_pose_(1), particles.at(i).p_pose_(2));

    }

    for (int i = 0; i < pcn_; i++)
    {
        // ROS_INFO("Joining thread %d", i);
        if (pred_threads_vec[i].joinable())
        {
            pred_threads_vec[i].join();
        }
        // ROS_INFO("AFTER2 pose of particle %d = %f, %f, %f", i, particles.at(i).p_pose_(0), particles.at(i).p_pose_(1), particles.at(i).p_pose_(2));

    }
    pred_threads_vec.clear();

    // // Particle to compute DR without filtering
    // dr_particle_.at(0).motion_prediction(vel_rot, vel_p, depth, dt, rng_);

}