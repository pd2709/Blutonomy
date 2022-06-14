#include <ros/ros.h>

// Extractor 
#include "../include/slam/extract.hpp"

// Filters used for time sync
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Message formats
#include <sensor_msgs/Image.h>
#include "oculus_node/OculusHeader.h"
#include <mavros_msgs/VFR_HUD.h>

#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

//! Note that these are currently in the other package
#include "slam/FeatureSet.h"
#include "slam/Feature.h"

#include "../include/slam/slam_geometry.hpp"

ros::Publisher feature_pub;
ros::Publisher image_pub;

Extract* extractor;

double MIN_SCAN_RANGE;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

void imageCallback(const sensor_msgs::ImageConstPtr &msg_sonar, const sensor_msgs::ImageConstPtr &msg_depth)
{

    ROS_INFO("Received image and header");

    // Read in 32F sonar image
    cv::Mat image_sonar_tmp = cv_bridge::toCvCopy(msg_sonar)->image;

    // Convert to 8 bit image (blob detect only works with 8 bit)
    cv::Mat image_sonar;
    image_sonar_tmp.convertTo(image_sonar, CV_8UC1, 100);

    // Read in depth info image
    cv::Mat image_depth = cv_bridge::toCvCopy(msg_depth)->image;

    // Pass to the extractor
    std::vector<cv::KeyPoint> keypoints = extractor->process_image(image_sonar);
    std::vector<cv::KeyPoint> keypoints_valid; // Keypoints beyond threshold distance. Filtered after range and bearing calculated.

    // Create the features message
    slam::FeatureSet features;
    features.header.stamp = msg_sonar->header.stamp;

    // Create a per-feature message:
    int i = 0; // Counter
    for (auto kpt_iter = keypoints.begin(); kpt_iter != keypoints.end(); kpt_iter++)
    {

        slam::Feature l;

        int x_pt = kpt_iter->pt.x;
        int y_pt = kpt_iter->pt.y;
        int x_im = image_depth.size[1]; // ! Why not image? Why image_depth?
        int y_im = image_depth.size[0];

        //          \   |   /
        //           \  |  /
        // 90 brng    \ | /    -90 brng
        //  y <-------(0,0)------>

        int dx = x_pt - x_im / 2;
        int dy = y_im - y_pt;

        l.bearing = atan2(float(dy), float(dx)) - M_PI / 2;


        // pixel range to point:
        double r_px = sqrt(pow(double(dx),2) + pow(double(dy),2));
        double r_frac = r_px/double(y_im);
        double r = r_frac * 17.0; // Range from cpp file

        // Also get an x and y from depth image...
        // float x_depth = image_depth.at<cv::Vec3f>(y_pt, x_pt)[1];
        // float y_depth = image_depth.at<cv::Vec3f>(y_pt, x_pt)[2];

        // double alt_bearing = 0;

        // if (x_depth > 0 and y_depth > 0)
        // {
        //     alt_bearing = atan2(float(y_depth), float(x_depth));
        // }
       

        // std::cout << "X-depth = " << x_depth << " | Y-depth = " << y_depth << " bearing = " << alt_bearing << std::endl;

        
        //l.range = image_depth.at<cv::Vec3f>(y_pt, x_pt)[0];
        l.range = r;

        // ! ASSUMING THAT SONAR IS 15 degrees elevation with respect to global frame
        // The following yields an approximation for elevation

        // The depth of the seabed in gazebo is -1.85 m, sonar is at -0.4. This is the difference.
        float z_features = -1.4; // ! make param in launch file? Is this even correct or should it be -1.8?

        // Get the vehicle pitch
        geometry_msgs::TransformStamped transformStamped;
        double roll, pitch, yaw;
        double elevation_optimised = 1000;
        try
        {
            transformStamped = tfBuffer.lookupTransform("world", "slam_optimised_sonar",
                                                        ros::Time(0));

            gtsam::Point3 point(transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);

            gtsam::Rot3 rot = gtsam::Rot3::Quaternion(transformStamped.transform.rotation.w,
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z);

            gtsam::Pose3 sonar_pose(rot, point);

            tf::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            // std::cout << "read vehicle pitch as " << pitch << std::endl;

            elevation_optimised = slam_geometry::predictOptimalElevation(sonar_pose, l.bearing, l.range, z_features);
        }
        catch (tf2::TransformException &ex)
        {
            // std::cout << "vehicle pitch unavailable" << std::endl;
        }

        if(elevation_optimised > 100){
            l.elevation = abs(15 * DEG2RAD) - asin(abs(z_features) / l.range);
        }else{
            l.elevation = elevation_optimised;
        }

        // std::cout << "elevation is " << l.elevation << std::endl;
        

        // Only publish features with legal elevations
        if (l.range > MIN_SCAN_RANGE)
        {
            keypoints_valid.emplace_back(*kpt_iter);
            features.features.emplace_back(l);

            // Debugging print
            // std::cout << "Feature " << i << " at (r, b, e) = (" << l.range << ", " << l.bearing << ", " << l.elevation << ")" << std::endl
                     // << std::endl;

            i++;
        }
    }

    // Add keypoints to image
    cv::Mat image_keypoints;
    cv::drawKeypoints(image_sonar, keypoints_valid, image_keypoints, cv::Scalar(0,0,255));

    // Label the points
    for (size_t i = 0; i<keypoints_valid.size(); i++)
    {
        cv::putText(image_keypoints, to_string(i), cv::Point2d(keypoints_valid[i].pt.x, keypoints_valid[i].pt.y), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(255, 0, 0));
    }

    // Display the image
    // cv::imshow("sonar_display", image_keypoints);
    // cv::imwrite("sonar_display.png", image_keypoints);
    // cv::waitKey(1000);

    // Publish the image
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    img_bridge = cv_bridge::CvImage(msg_sonar->header, sensor_msgs::image_encodings::RGB8, image_keypoints);
    image_pub.publish(img_bridge.toImageMsg());

    // Publish the features
    std::cout << "Accepted " << features.features.size() << " features. Publishing." << std::endl;
    feature_pub.publish(features);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_uuvsim");
    ros::NodeHandle n;

    double AKAZE_THRESHOLD;
    n.getParam("/extract_uuvsim/akaze_threshold", AKAZE_THRESHOLD);
    extractor = new Extract(AKAZE_THRESHOLD);

    n.getParam("/extract_uuvsim/min_scan_range", MIN_SCAN_RANGE);

    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Need to subscribe to the sonar image itself and depth info
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/bluerov2/depth/image_raw_raw_sonar", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(n, "/bluerov2/depth/image_raw_depth_raw_sonar", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, image_depth_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    // We will publish an image showing keypoints
    image_pub = n.advertise<sensor_msgs::Image>("/slam/sonar_display", 1);

    // We will publish landmarks
    feature_pub = n.advertise<slam::FeatureSet>("/slam/features", 1);

    ros::spin();

    // cv::destroyWindow("sonar_display");
}