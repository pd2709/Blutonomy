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

ros::Publisher feature_pub;
ros::Publisher image_pub;

Extract* extractor;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

constexpr double DEPTH_CONVERSION = -1.282; // Convert altitude to depth. Derived from CAD 
double z_features = 1000; // Big value, until our first initialisation, after which we use it.
constexpr double ELEV_MAX = 10*DEG2RAD; // Half-FOV max elevation


void imageCallback(const sensor_msgs::ImageConstPtr &image, const oculus_node::OculusHeaderConstPtr &image_header)
{

    ROS_INFO("Received image and header");

    // Covert to a CV matrix
    cv::Mat image_cv = cv_bridge::toCvCopy(image)->image;

    // Pass to the extractor
    std::vector<cv::KeyPoint> keypoints = extractor->process_image(image_cv);

    // Add keypoints to image
    //! 8bit conversion needed?
    cv::Mat image_8bit;
    image_cv.convertTo(image_8bit, CV_8UC1, 0.01); // DrawKeyPoints only seems to work on 8bit images
    cv::Mat image_keypoints;
    cv::drawKeypoints(image_8bit, keypoints, image_keypoints, cv::Scalar(0,0,255));

    // Draw the sonar origin
    cv::drawMarker(image_keypoints, cv::Point2d(0,image_keypoints.rows/2), cv::Scalar(0, 255, 0), MARKER_CROSS, 10, 1);

    // Label the points
    for (size_t i = 0; i<keypoints.size(); i++)
    {
        cv::putText(image_keypoints, to_string(i), cv::Point2d(keypoints[i].pt.x, keypoints[i].pt.y), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(255, 0, 0));
    }

    // Display the image
    //cv::imshow("sonar_display", image_keypoints);
    //cv::imwrite("sonar_display.png", image_keypoints);
    //cv::waitKey(1000);
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    img_bridge = cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::RGB8, image_keypoints);
    image_pub.publish(img_bridge.toImageMsg());

    // Create the features message
    slam::FeatureSet features;
    features.header.stamp = image->header.stamp;

    // Get some parameters
    double r_res = image_header->rangeResolution;
    int y_max = image_header->beamCount;
    double hfov = (image_header->freqMode == 1) ? 130.0*DEG2RAD : 60.0*DEG2RAD; // Mode 1 is LF

    // Create a per-feature message:
    int i = 0; // Counter
    for (auto kpt_iter = keypoints.begin(); kpt_iter != keypoints.end(); kpt_iter++)
    {

        // Wait until we have a z measurement before continuing
        // Z-features is set to 1000 initially. Once being updated, it will become ~1 and this condition will evaluate true
        if (z_features < 10)
        {
            slam::Feature l;

            int x = kpt_iter->pt.x;
            int y = kpt_iter->pt.y;

            l.bearing = (double(y_max - 2 * y) / double(2 * y_max)) * hfov;
            l.range = x * r_res;

            // Sanity check:
            assert(abs(2 * l.bearing) <= hfov);
            assert(l.range <= image_header->range);

            // Two options for elevation calculation follow. Un-comment one of them.

            /*********************************************************************************************/

            /*
             * Elevation based on variable pitch 
             */
            // Get the vehicle pitch
            // geometry_msgs::TransformStamped transformStamped;
            // double roll, pitch, yaw;
            // double elevation_optimised = 1000;
            // try
            // {
            //     transformStamped = tfBuffer.lookupTransform("map", "slam_optimised_sonar",
            //                                                 ros::Time(0));

            //     gtsam::Point3 point(transformStamped.transform.translation.x,
            //                         transformStamped.transform.translation.y,
            //                         transformStamped.transform.translation.z);

            //     gtsam::Rot3 rot = gtsam::Rot3::Quaternion(transformStamped.transform.rotation.w,
            //                                               transformStamped.transform.rotation.x,
            //                                               transformStamped.transform.rotation.y,
            //                                               transformStamped.transform.rotation.z);

            //     gtsam::Pose3 sonar_pose(rot, point);

            //     tf::Quaternion q(
            //         transformStamped.transform.rotation.x,
            //         transformStamped.transform.rotation.y,
            //         transformStamped.transform.rotation.z,
            //         transformStamped.transform.rotation.w);

            //     tf::Matrix3x3 m(q);
            //     m.getRPY(roll, pitch, yaw);
            //     std::cout << "read vehicle pitch as " << pitch << std::endl;

            //     elevation_optimised = slam_geometry::predictOptimalElevation(sonar_pose, l.bearing, l.range, z_features);
            // }
            // catch (tf2::TransformException &ex)
            // {
            //     std::cout << "vehicle pitch unavailable" << std::endl;
            // }

            // if (elevation_optimised > 100)
            // {
            //     l.elevation = abs(15 * DEG2RAD) - asin(abs(z_features) / l.range);
            // }
            // else
            // {
            //     l.elevation = elevation_optimised;
            // }

            /*********************************************************************************************/

            /*
             * Elevation based on fixed pitch
             */
            // ! ASSUMING THAT SONAR IS 15 degrees elevation with respect to global frame
            // The following yields an approximation for elevation
            //std::cout << "Range = " << l.range << " z = " << z_features << std::endl;
            l.elevation = (15 * DEG2RAD) - asin(abs(z_features) / l.range);

            /*********************************************************************************************/

            // Debugging print
            //std::cout << "Landmark " << i << " at (r, b, e) = (" << l.range << ", " << l.bearing << ", " << l.elevation << ")" << std::endl;

            // Only publish features with legal elevations
            if (abs(l.elevation) < ELEV_MAX)
            {
                features.features.emplace_back(l);
            }
        }

        i++;
    }

    // Publish the features
    std::cout << "Publishing features" << std::endl;
    feature_pub.publish(features);

}

// Get vehicle altitude (depth) updates
void vfrCallback(const mavros_msgs::VFR_HUDConstPtr &msg)
{
    // Altitude of vehicle is a negative number, representing m below the surface
    float altitude = msg->altitude;
    z_features = DEPTH_CONVERSION - altitude;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "oculus_subscriber");
    ros::NodeHandle n;

    double AKAZE_THRESHOLD;
    n.getParam("/extract_oculus/akaze_threshold", AKAZE_THRESHOLD);
    extractor = new Extract(AKAZE_THRESHOLD);

    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Need to subscribe to the sonar image itself and depth info
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/oculus/polarscan", 1);
    message_filters::Subscriber<oculus_node::OculusHeader> image_info_sub(n, "/oculus/header", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, oculus_node::OculusHeader> sync(image_sub, image_info_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    // Subscribe to depth information
    ros::Subscriber vfr_sub_ = n.subscribe("/mavros/vfr_hud", 1, &vfrCallback); // Altitude information

    // We will publish an image showing keypoints
    image_pub = n.advertise<sensor_msgs::Image>("/slam/sonar_display", 1);

    // We will publish landmarks
    feature_pub = n.advertise<slam::FeatureSet>("/slam/features", 1);

    ros::spin();

    cv::destroyWindow("sonar_display");
}