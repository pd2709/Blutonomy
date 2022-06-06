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

// TODO: Make these launch file parameters
float IMU_PERIOD; // Update period of IMU //! This will vary for simulated vs experimental
string PUBLISH_FRAME; // Frame to publish trajectory results in

std::shared_ptr<Graph> graph;

// Accumulators and step count used for averaging linear and angular velocity (good for estimating bias)
gtsam::Vector3 linear_acceleration_accumulator;
gtsam::Vector3 angular_velocity_accumulator;
int step = 1;

// Publish landmark positions to a point cloud topic
ros::Publisher landmark_pointcloud_pub;

// Publish feature positions to a point cloud topic
ros::Publisher feature_pointcloud_pub;

// Publish matches as arrow markers
ros::Publisher landmark_match_pub;

// Publish slam result as custom message, to be bagged and opened in MATLAB
ros::Publisher slam_result_pub;

int counter = 0;

void publishSLAMResult(ros::Time time)
{

    slam::SlamResult result;
    result.header.stamp = time;

    // Pack vehicle pose
    geometry_msgs::PoseWithCovarianceStamped the_pose;
    the_pose.header.stamp = time;

    // We update for the ith frame
    int i = graph->frame_count_;

    std::stringstream ss;
    ss << "X" << i;

    the_pose.header.frame_id = ss.str();

    gtsam::Pose3 gtsam_pose = graph->result.at<gtsam::Pose3>(X(i));

    the_pose.pose.pose.position.x = gtsam_pose.x();
    the_pose.pose.pose.position.y = gtsam_pose.y();
    the_pose.pose.pose.position.z = gtsam_pose.z();

    the_pose.pose.pose.orientation.w = gtsam_pose.rotation().toQuaternion().w();
    the_pose.pose.pose.orientation.x = gtsam_pose.rotation().toQuaternion().x();
    the_pose.pose.pose.orientation.y = gtsam_pose.rotation().toQuaternion().y();
    the_pose.pose.pose.orientation.z = gtsam_pose.rotation().toQuaternion().z();

    // TODO: also add covariances to published poses

    // gtsam::Matrix cov = graph->last_marginals.marginalCovariance(X(i));
    gtsam::Matrix cov = graph->isam2_->marginalCovariance(X(graph->frame_count_));

    /** ROS message covariance is a 6x6 row-major matrix
         * In order, the parameters are:
         * (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
         * 
         *    x  y  z rx ry rz
         *  x 0  1  2  3  4  5
         *  y 6  7  8  9 10 11
         *  z 12 13 14 15 16 17 
         * rx 18 19 20 21 22 23
         * ry 24 25 26 27 28 29
         * rz 30 31 32 33 34 35
         * 
         * Gtsam ordering is rotation first
         * https://groups.google.com/g/gtsam-users/c/jpz10n7VHqU
         * 
         *     rx   ry   rz    x    y    z
         * rx 0,0  0,1  0,2  0,3  0,4  0,5
         * ry 1,0  1,1  1,2  1,3  1,4  1,5
         * rz 2,0  2,1  2,2  2,3  2,4  2,5
         *  x 3,0  3,1  3,2  3,4  3,5  3,6
         *  y 4,0  4,1  4,2  4,4  4,5  4,6
         *  z 5,0  5,1  5,2  5,3  5,5  5,6
         * 
         * 
         * Corresponing ROS indices:
         * 21 23 23 18 19 20 
         * 27 28 29 24 25 26
         * 33 34 35 30 31 32
         *  3  4  5  0  1  2
         *  9 10 11  6  7  8
         * 15 16 17 12 13 14
        
        **/

    int indices[] = {
        21, 22, 23, 18, 19, 20,
        27, 28, 29, 24, 25, 26,
        33, 34, 35, 30, 31, 32,
        3, 4, 5, 0, 1, 2,
        9, 10, 11, 6, 7, 8,
        15, 16, 17, 12, 13, 14};

    // Traverse row by row and assign values
    for (int r = 0; r < 6; r++)
    {

        for (int c = 0; c < 6; c++)
        {

            the_pose.pose.covariance[indices[6 * r + c]] = cov(r, c);
        }
    }

    result.var_X = the_pose;

    geometry_msgs::Point the_vel;
    gtsam::Vector3 gtsam_vel = graph->result.at<gtsam::Vector3>(V(i));
    the_vel.x = gtsam_vel.x();
    the_vel.y = gtsam_vel.y();
    the_vel.z = gtsam_vel.z();
    result.var_V = the_vel;

    geometry_msgs::Point the_acc_bias;
    gtsam::Vector3 gtsam_acc_bias = graph->result.at<gtsam::imuBias::ConstantBias>(B(i)).accelerometer();
    the_acc_bias.x = gtsam_acc_bias.x();
    the_acc_bias.y = gtsam_acc_bias.y();
    the_acc_bias.z = gtsam_acc_bias.z();
    result.var_B_acc = the_acc_bias;

    geometry_msgs::Point the_gyr_bias;
    gtsam::Vector3 gtsam_gyr_bias = graph->result.at<gtsam::imuBias::ConstantBias>(B(i)).gyroscope();
    the_gyr_bias.x = gtsam_gyr_bias.x();
    the_gyr_bias.y = gtsam_gyr_bias.y();
    the_gyr_bias.z = gtsam_gyr_bias.z();
    result.var_B_gyr = the_gyr_bias;

    // Pack Landmarks
    // The latest measurement is att index frame count, so we go up to and including this
    for (int i = 0; i<graph->n_landmarks; i++)
    {

        geometry_msgs::PoseWithCovarianceStamped the_pose;
        the_pose.header.stamp = time;

        std::stringstream ss;
        ss << "L" << i;
        the_pose.header.frame_id = ss.str();

        gtsam::Point3 gtsam_pose = graph->result.at<gtsam::Point3>(L(i));

        the_pose.pose.pose.position.x = gtsam_pose.x();
        the_pose.pose.pose.position.y = gtsam_pose.y();
        the_pose.pose.pose.position.z = gtsam_pose.z();

        // If the landmark value is at the linearisation point, also add it
        if (graph->isam2_->valueExists(L(i)))
        {
            gtsam::Matrix cov = graph->isam2_->marginalCovariance(L(i));
            //gtsam::Matrix cov = graph->last_marginals.marginalCovariance(L(i));
            // std::cout << "Landmark " << i << " cov: \n" << cov << std::endl;

            /** ROS message covariance is a 6x6 row-major matrix
         * In order, the parameters are:
         * (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
         * 
         *    x  y  z rx ry rz
         *  x 0  1  2  3  4  5
         *  y 6  7  8  9 10 11
         *  z 12 13 14 15 16 17 
         * rx 18 19 20 21 22 23
         * ry 24 25 26 27 28 29
         * rz 30 31 32 33 34 35
        
        **/
            the_pose.pose.covariance[0] = cov(0, 0);
            the_pose.pose.covariance[1] = cov(0, 1);
            the_pose.pose.covariance[2] = cov(0, 2);
            the_pose.pose.covariance[6] = cov(1, 0);
            the_pose.pose.covariance[7] = cov(1, 1);
            the_pose.pose.covariance[8] = cov(1, 2);
            the_pose.pose.covariance[12] = cov(2, 0);
            the_pose.pose.covariance[13] = cov(2, 1);
            the_pose.pose.covariance[14] = cov(2, 2);
        }

        result.var_L.push_back(the_pose);
    }

    // Pack landmarks
    slam_result_pub.publish(result);

}

// Publish feature measurements to a point cloud topic
void publishFeatures(ros::Time time)
{
    pcl::PointCloud<pcl::PointXYZRGB> features;
    features.header.stamp = time.toSec()*1000;
    features.header.frame_id = PUBLISH_FRAME;
    pcl::PointXYZRGB pt;

    // Matched

    for (int i=0; i<graph->associator_features_global.size(); i++)
    {

        gtsam::Point3 gtsam_point = graph->associator_features_global[i];

        pt.x = gtsam_point.x();
        pt.y = gtsam_point.y();
        pt.z = gtsam_point.z();

        int rgb;

        // True if matched, false otherwise
        if (std::count(graph->hypothesis.feature_indices.begin(), graph->hypothesis.feature_indices.end(), i))
        {
            rgb = 255 << 16 | 255 << 8 | 255; // White
        }
        else
        {
            rgb = 255 << 16 | 0 << 8 | 0; // Red
        }

        pt.rgb = rgb;

        features.points.push_back(pt);
    }

    feature_pointcloud_pub.publish(features);
}

// Publish matches made with JCBB
void publishMatches(ros::Time time)
{
    visualization_msgs::MarkerArray markers;

    for (int i = 0; i<graph->hypothesis.feature_indices.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = time;
        marker.header.frame_id = PUBLISH_FRAME;
        marker.type = 0; // Arrows
        marker.color.r = 255;
        marker.color.a = 1;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.id = i;

        geometry_msgs::Point feature;
        feature.x = graph->associator_features_global[graph->hypothesis.feature_indices[i]].x();
        feature.y = graph->associator_features_global[graph->hypothesis.feature_indices[i]].y();
        feature.z = graph->associator_features_global[graph->hypothesis.feature_indices[i]].z();

        geometry_msgs::Point landmark;
        landmark.x = graph->associator_landmarks_global[graph->hypothesis.landmark_indices[i]].x();
        landmark.y = graph->associator_landmarks_global[graph->hypothesis.landmark_indices[i]].y();
        landmark.z = graph->associator_landmarks_global[graph->hypothesis.landmark_indices[i]].z();

        marker.points.push_back(feature);
        marker.points.push_back(landmark);
        markers.markers.push_back(marker);
    }

    landmark_match_pub.publish(markers);

}

//  Publish landmarks as point cloud, which can be viewed in RViz
void publishLandmarks(ros::Time time)
{
    pcl::PointCloud<pcl::PointXYZHSV> landmarks;
    landmarks.header.stamp = time.toSec()*1000;
    landmarks.header.frame_id = PUBLISH_FRAME;
    pcl::PointXYZHSV pt;

    for (int i=0; i<graph->associator_landmarks_global.size(); i++)
    {
        gtsam::Point3 gtsam_point = graph->associator_landmarks_global[i];

        pt.x = gtsam_point.x();
        pt.y = gtsam_point.y();
        pt.z = gtsam_point.z();

        pt.h = i * 10;

        // int rgb = 255 << 16 | 10*i << 8 | 255;
        // pt.rgb = rgb;

        landmarks.points.push_back(pt);
    }

    landmark_pointcloud_pub.publish(landmarks);
}

// Publish sonar pose as a transform
void publishSonarPose(ros::Time time)
{
    gtsam::Pose3 pose_optimised_gtsam = graph->result.at<gtsam::Pose3>(S(graph->frame_count_));

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = time;
    transform.header.frame_id = PUBLISH_FRAME;    // TODO: Confirm that I want to call origin "world"
    transform.child_frame_id = "slam_optimised_sonar";

    transform.transform.translation.x = pose_optimised_gtsam.x();
    transform.transform.translation.y = pose_optimised_gtsam.y();
    transform.transform.translation.z = pose_optimised_gtsam.z();

    transform.transform.rotation.w = pose_optimised_gtsam.rotation().toQuaternion().w();
    transform.transform.rotation.x = pose_optimised_gtsam.rotation().toQuaternion().x();
    transform.transform.rotation.y = pose_optimised_gtsam.rotation().toQuaternion().y();
    transform.transform.rotation.z = pose_optimised_gtsam.rotation().toQuaternion().z();
    
    br.sendTransform(transform);

}

// Publish vehicle pose as a transform
void publishVehiclePose(ros::Time time)
{
    gtsam::Pose3 pose_optimised_gtsam = graph->result.at<gtsam::Pose3>(X(graph->frame_count_));

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = time;
    transform.header.frame_id = PUBLISH_FRAME;    // TODO: Confirm that I want to call origin "world"
    transform.child_frame_id = "slam_optimised";

    transform.transform.translation.x = pose_optimised_gtsam.x();
    transform.transform.translation.y = pose_optimised_gtsam.y();
    transform.transform.translation.z = pose_optimised_gtsam.z();

    transform.transform.rotation.w = pose_optimised_gtsam.rotation().toQuaternion().w();
    transform.transform.rotation.x = pose_optimised_gtsam.rotation().toQuaternion().x();
    transform.transform.rotation.y = pose_optimised_gtsam.rotation().toQuaternion().y();
    transform.transform.rotation.z = pose_optimised_gtsam.rotation().toQuaternion().z();
    
    br.sendTransform(transform);

}

// Callback when a new set of features from an image is received
void featureCallback(const slam::FeatureSet &msg)
{

    // Want to re-package into a vector of RangeBearingMeasurement
    std::vector<slam_geometry::RangeBearingMeasurement> features;

    // Unpack and pass to the graph function to add a landmark factor
    for (auto feature_iter = msg.features.begin(); feature_iter < msg.features.end(); feature_iter++)
    {
        slam_geometry::RangeBearingMeasurement f;
        f.bearing = feature_iter->bearing;
        f.range = feature_iter->range;
        f.elevation = feature_iter->elevation; // This will be zero, but we keep passing aroun for completeness
        features.push_back(f);
    }

    graph->addLandmarkMeasurements(features);

    // TODO: addLandmarkMeasurements should return appropriate current state info to broadcast over ROS
    /**
     * Things that may be good to publish:
     * - Vehicle Pose3 with covariance (SLAM)
     * - 
     * */

    // If graph is not empty, publish data
    if (!graph->is_graph_empty)
    {
        std::cout << "publishing results\n";
        ros::Time time_publish_result = msg.header.stamp;

        publishLandmarks(time_publish_result);
        publishVehiclePose(time_publish_result);
        publishSonarPose(time_publish_result);
        publishMatches(time_publish_result);
        publishFeatures(time_publish_result);
        publishSLAMResult(time_publish_result);
        std::cout << "Finished publishing results\n";
    }
}

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

int main(int argc, char ** argv)
{
    // Set up node
    // TODO: can node names be placed in a global cfg?
    ros::init(argc, argv, "graph");
    ros::NodeHandle n;

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

    /** 
     * Subscribe to stuff
     **/
    ros::Subscriber imu_sub_ = n.subscribe("/slam/imu/data", 1, &imuCallback);
    ros::Subscriber feature_sub_ = n.subscribe("/slam/features", 1, &featureCallback);

    // Publish landmark positions in global frame for testing
    landmark_pointcloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZHSV>>("/slam/landmarks", 1);
    feature_pointcloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/slam/features_projected", 1);
    landmark_match_pub = n.advertise<visualization_msgs::MarkerArray>("/slam/matches", 1);

    slam_result_pub = n.advertise<slam::SlamResult>("/slam/result", 1);

    ros::spin();

}