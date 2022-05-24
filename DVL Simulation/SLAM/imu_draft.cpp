
void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    //std::cout << "IMU received linear (x, y, z) = ("
            //   << msg->linear_acceleration.x << ", "
            //   << msg->linear_acceleration.y << ", "
            //   << msg->linear_acceleration.z << ")" << std::endl;

    // IMU is received with z down, x ahead (implies y right)
    // Want to work with z up, x ahead (implies y left)
    // This is achieved by transforming:
    // x -> x
    // y -> -y
    // z -> -z
    //
    // gtsam::Vector3 linear_acceleration(
    //     msg->linear_acceleration.x,
    //     -msg->linear_acceleration.y,
    //     -msg->linear_acceleration.z);

    // gtsam::Vector3 angular_velocity(
    //     msg->angular_velocity.x,
    //     -msg->angular_velocity.y,
    //     -msg->angular_velocity.z);

    gtsam::Vector3 linear_acceleration(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);

    gtsam::Vector3 angular_velocity(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);

    linear_acceleration_accumulator += linear_acceleration;
    angular_velocity_accumulator += angular_velocity;

    gtsam::Vector3 linear_acceleration_mean = linear_acceleration_accumulator/step;
    gtsam::Vector3 angular_velocity_mean = angular_velocity_accumulator/step;

    // std::cout << std::setprecision(4) << "Running average at step " << step << " lin (x,y,z) = (" 
    // << linear_acceleration_mean[0] << ", "
    // << linear_acceleration_mean[1] << ", "
    // << linear_acceleration_mean[2] << ")"
    // << std::endl;

    graph->addImuMeasurement(linear_acceleration, angular_velocity, IMU_PERIOD);
    gtsam::Pose3 pose_dr_gtsam = graph->getEstimateDeadReckon();

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = PUBLISH_FRAME;    // TODO: Confirm that I want to call origin "world"
    transform.child_frame_id = "slam_dr";

    transform.transform.translation.x = pose_dr_gtsam.x();
    transform.transform.translation.y = pose_dr_gtsam.y();
    transform.transform.translation.z = pose_dr_gtsam.z();

    transform.transform.rotation.w = pose_dr_gtsam.rotation().toQuaternion().w();
    transform.transform.rotation.x = pose_dr_gtsam.rotation().toQuaternion().x();
    transform.transform.rotation.y = pose_dr_gtsam.rotation().toQuaternion().y();
    transform.transform.rotation.z = pose_dr_gtsam.rotation().toQuaternion().z();
    
    br.sendTransform(transform);

    // TODO: addImuMeasurement should return appropriate current state info to broadcast over ROS
    /**
     * Things that may be good to publish:
     * - Vehicle Pose3 with covariance (Dead reckoned)
     **/ 

    step += 1;
}

// Print IMU at regular intervals for debugging
void timerCallback(const ros::TimerEvent&){
    graph->getEstimateDeadReckon();
}
