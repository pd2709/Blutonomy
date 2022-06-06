// This node gets odometry (IMU data (orientation) and DVL data (altitude and linear velocity)) to set the pose of the BlueROV2 UUV
// 

// Kalman filter can be used with the IMU, DVL and a third sensor (like a GNSS) optimise the value
// TODO: Object Association 


//! Headers
// Headers make initialisation of objects from specialised classes shorter and easier

#include "../include/slam/graph.hpp"            // Allows for SLAM related operations (refer to graph.cpp and Graph based SLAM)

#include <ros/ros.h>                       
#include "gtsam/geometry/concepts.h"            // SLAM specialist library
// #include <tf2.h>

#include "slam/deadReckoning.h"                 // Rosmsg formats
#include "slam/DVL.h"                           // Allows for the use of DVL rosmsg format [not standard]
#include <sensor_msgs/Imu.h>                    // Allows for the use of IMU rosmsg format [from standard library sensor_msgs]

// Pointcloud library is used for landmarks
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

//! Global Variables
gtsam::Quaternion imuOrientation;
gtsam::Vector3 dvlLinearVelocity;
gtsam::Vector1 dvlAltitude;
ros::Time dvlTime;
ros::Time previousDVLTime;
bool deadReckon = 0;

void dvlCallback(const slam::DVLConstPtr& msg) { //const uuv_sensor_ros_plugins_msgs::DVLConstPtr &msg --> issues with linking 
    std::cout << "DVL CALLBACK\n"; // Sanity Check
    
    gtsam::Vector3 linear_velocity(
        msg->velocity.x,
        msg->velocity.y,
        msg->velocity.z);
    gtsam::Vector1 altitude(msg->altitude);
    ros::Time time(msg->header.stamp.nsec);

    dvlLinearVelocity = linear_velocity;
    dvlAltitude = altitude;
    previousDVLTime = dvlTime;
    dvlTime = time;
    deadReckon = 1;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) { // Gets IMU data
    std::cout << "IMU CALLBACK\n";  
    gtsam::Quaternion orientation(msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);

    imuOrientation = orientation;
}


void timerCallback(const ros::TimerEvent&){ // Timer to tine DVL and IMU callback

    std::cout << "TIMER CALLBACK\n";
    
}

slam::deadReckoning deadReckoning(gtsam::Vector3 linear_velocity, gtsam::Vector1 altitude, ros::Time time0, ros::Time time1, gtsam::Quaternion orientation){ // Provides x,y,z cartesian position and quaternion heading?? (Point and Rot)
    std::cout << "DEADRECKON CALLBACK\n";
    slam::deadReckoning drMSG;

    drMSG.ts = time1;
    drMSG.x = linear_velocity(1);
    drMSG.y = linear_velocity(2);

    // RPY
    

    // XYZ

    return drMSG;
}


//! Main
int main(int argc, char ** argv)
{
    // Set up node
    // TODO: can node names be placed in a global cfg?
    ros::init(argc, argv, "deadReckoning");
    ros::NodeHandle n;

    // Timer (5Hz) DVL frequency
    ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback); //5Hz in line with DVL frequency

    // Set up variables

    /** 
     * Subscribers (SANITY CHECKED!)
     **/
    ros::Subscriber imu_sub_ = n.subscribe("/bluerov2/imu", 1, &imuCallback); // /slam/imu/data (REMAP!!)
    ros::Subscriber dvl_sub_ = n.subscribe("/bluerov2/dvl",1, &dvlCallback); // /slam/dvl/data (REMAP!!)

    /**
     *  Publishers
     **/

    ros::Publisher deadReckPub = n.advertise<slam::deadReckoning>("deadReckoning", 1);

    // ros::spin();

    while(ros::ok())
    {   
        if (deadReckon == 1) {
            slam::deadReckoning msg = deadReckoning(dvlLinearVelocity, dvlAltitude, previousDVLTime, dvlTime, imuOrientation);
            deadReckPub.publish(msg);
            deadReckon = 0;
        }
        ros::spinOnce();
    }
}
