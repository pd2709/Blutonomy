// This node gets odometry (IMU data (orientation) and DVL data (altitude and linear velocity)) to set the pose of the BlueROV2 UUV

// Kalman filter can be used with the IMU, DVL and a third sensor (like a GNSS) optimise the value
// TODO: Object Association 


//! Headers
// Headers make initialisation of objects from specialised classes shorter and easier

#include "../include/slam/graph.hpp"            // Allows for SLAM related operations (refer to graph.cpp and Graph based SLAM)

#include <ros/ros.h>                       
#include "gtsam/geometry/concepts.h"            // SLAM specialist library
// #include <tf2.h>
#include "tf/transform_datatypes.h"
#include <tf/tf.h> 
// #include "LinearMath/btMatrix3x3.h"
#include <vector>
#include <string>

#include "slam/deadReckoning.h"                 // Rosmsg formats
#include "slam/DVL.h"                           // Allows for the use of DVL rosmsg format [not standard]
#include <sensor_msgs/Imu.h>                    // Allows for the use of IMU rosmsg format [from standard library sensor_msgs]

// Pointcloud library is used for landmarks
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

//! Global Variables
tf::Pose prevPose;
tf::Pose newPose;
tf::Quaternion orientation;
vector<double> linearVelocity;
double altitude;
ros::Time newTime;
ros::Time previousTime;
bool deadReckon = 0;

void dvlCallback(const slam::DVLConstPtr& msg) { //const uuv_sensor_ros_plugins_msgs::DVLConstPtr &msg --> issues with linking 
    std::cout << "DVL CALLBACK\n"; // Sanity Check

    linearVelocity.clear();                             // Update linear velocity
    linearVelocity.push_back(msg->velocity.x);
    linearVelocity.push_back(msg->velocity.y);
    linearVelocity.push_back(msg->velocity.z);
    auto str = std::to_string(linearVelocity.size()); 
    std::cout << "LinearVelocity size: " << str << "\n";

    altitude = msg->altitude;       
    
    ros::Time currentTime(msg->header.stamp.nsec);      // Get the time of the new message
    previousTime = newTime;                             // Update the previous time
    newTime = currentTime;                              // Update newtime
    deadReckon = 1;                                     // Toggle deadreckon node
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) { // Gets IMU data
    std::cout << "IMU CALLBACK\n";  
    orientation.setW(msg->orientation.w);
    orientation.setX(msg->orientation.x);
    orientation.setY(msg->orientation.y);
    orientation.setZ(msg->orientation.z);
}

void timerCallback(const ros::TimerEvent&){ // Timer to tine DVL and IMU callback
    std::cout << "TIMER CALLBACK\n";
}

slam::deadReckoning deadReckoning(vector<double> linear_velocity, double altitude, ros::Time time0, ros::Time time1, tf::Quaternion orientation){ // Provides x,y,z cartesian position and quaternion heading?? (Point and Rot)
    std::cout << "DEADRECKON CALLBACK\n";
    slam::deadReckoning drMSG;

    drMSG.ts = time1;                       // Sending time in rosmsg //! SUSPECT
    ros::Duration dt = time1-time0;         // Preparing time difference to calculate distance travelled (x,y,z)
    double dt_sec = dt.sec;

    drMSG.x = linear_velocity.at(0)*dt_sec;    // Sending distance travelled in //! This distance needs to be added onto a previous pose (look at previous and new pose)
    drMSG.y = linear_velocity.at(1)*dt_sec;
    drMSG.z = linear_velocity.at(2)*dt_sec;

    // RPY
    tf::Matrix3x3 m(orientation);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    drMSG.roll = roll;
    drMSG.pitch = pitch;
    drMSG.yaw = yaw;

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

    // Subscribers
    ros::Subscriber imu_sub_ = n.subscribe("/bluerov2/imu", 1, &imuCallback); // /slam/imu/data (REMAP!!)
    ros::Subscriber dvl_sub_ = n.subscribe("/bluerov2/dvl",1, &dvlCallback); // /slam/dvl/data (REMAP!!)

    // Publisher
    ros::Publisher deadReckPub = n.advertise<slam::deadReckoning>("deadReckoning", 1);

    // ros::spin();

    while(ros::ok())
    {   
        if (deadReckon == 1) {
            slam::deadReckoning msg = deadReckoning(linearVelocity, altitude, previousTime, newTime, orientation);
            deadReckPub.publish(msg);
            deadReckon = 0;
        }
        ros::spinOnce();
    }
}
