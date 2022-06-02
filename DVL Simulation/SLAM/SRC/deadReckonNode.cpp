#include "../include/slam/graph.hpp"

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "gtsam/geometry/concepts.h"

#include "slam/FeatureSet.h"
#include "slam/Feature.h"
#include "slam/SlamResult.h"

// Point cloud for landmarks
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


// IMU Callback
void imuCallback(const sensor_msgs::ImuConstPtr &msg) { // Gets IMU data

msg->orientation
 
}

// DVL Callback
void dvlCallback(const dvl_uuv_msg &msg) { // Get DVL data

msg->altitude

msg->linear_velocity

}

// Timer Callback
void timerCallback(const ros::TimerEvent&){ // Timer to 

}


// Publish DeadReckon Value Using IMU and DVL Callback

//! Main
int main(int argc, char ** argv)
{
    // Set up node
    // TODO: can node names be placed in a global cfg?
    ros::init(argc, argv, "graph");
    ros::NodeHandle n;

    // Set up variables
    float ACCEL_NOISE_SIGMA, GYRO_NOISE_SIGMA, ACCEL_BIAS_RW_SIGMA, GYRO_BIAS_RW_SIGMA, LANDMARK_NOISE_ELEV, LANDMARK_NOISE_BEAR, LANDMARK_NOISE_RANGE, g;
    bool SHOULD_USE_JCBB, SHOULD_USE_DEPTH_FACTORS;

    // TODO: will need to pass parameters to graph
    n.getParam("/graph/publish_frame", PUBLISH_FRAME);

    n.getParam("/graph/imu_period", IMU_PERIOD);

    n.getParam("/graph/accel_noise_sigma", ACCEL_NOISE_SIGMA);
    n.getParam("/graph/gyro_noise_sigma", GYRO_NOISE_SIGMA);
    n.getParam("/graph/accel_bias_rw_sigma", ACCEL_BIAS_RW_SIGMA);
    n.getParam("/graph/gyro_bias_rw_sigma", GYRO_BIAS_RW_SIGMA);

    n.getParam("/graph/g", g);

    n.getParam("/graph/landmark_noise_elev", LANDMARK_NOISE_ELEV);
    n.getParam("/graph/landmark_noise_bear", LANDMARK_NOISE_BEAR);
    n.getParam("/graph/landmark_noise_range", LANDMARK_NOISE_RANGE);

    std::vector<double> translation_tmp;
    n.getParam("/graph/sonar_translation", translation_tmp);
    gtsam::Point3 SONAR_TRANSLATION(translation_tmp[0], translation_tmp[1], translation_tmp[2]);

    std::vector<double> rotation_tmp;
    n.getParam("/graph/sonar_rotation", rotation_tmp);
    gtsam::Rot3 SONAR_ROTATION(gtsam::Quaternion(rotation_tmp[0], rotation_tmp[1], rotation_tmp[2], rotation_tmp[3]).normalized());
    
    n.getParam("/graph/should_use_jcbb", SHOULD_USE_JCBB);
    n.getParam("/graph/shouls_use_depth_factors", SHOULD_USE_DEPTH_FACTORS);

    // Create a graph instance
    graph = std::make_shared<Graph>(ACCEL_NOISE_SIGMA, GYRO_NOISE_SIGMA, ACCEL_BIAS_RW_SIGMA, GYRO_BIAS_RW_SIGMA, LANDMARK_NOISE_ELEV, LANDMARK_NOISE_BEAR, LANDMARK_NOISE_RANGE, g, SONAR_TRANSLATION, SONAR_ROTATION, SHOULD_USE_JCBB, SHOULD_USE_DEPTH_FACTORS);

    // Create timer
    ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

    /** 
     * Subscribers
     **/
    ros::Subscriber imu_sub_ = n.subscribe("/slam/imu/data", 1, &imuCallback);
    ros::Subscriber feature_sub_ = n.subscribe("/slam/features", 1, &featureCallback);
    ros::Subscriber dvl_sub_ = n.subsribe("slam/dvl/data",1, &dvlCallback);

    /**
     *  Publishers
     **/
    landmark_pointcloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZHSV>>("/slam/landmarks", 1);     // Publish landmark positions in global frame for testing
    feature_pointcloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/slam/features_projected", 1);
    landmark_match_pub = n.advertise<visualization_msgs::MarkerArray>("/slam/matches", 1);

    slam_result_pub = n.advertise<slam::SlamResult>("/slam/result", 1);

    ros::spin();

}