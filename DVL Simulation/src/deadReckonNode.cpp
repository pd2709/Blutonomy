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
// Updated when deadReckoning is published
tf::Pose prevPose; 
tf::Pose newPose;
ros::Time newTime;
ros::Time previousTime;
bool deadReckon = 0; // Toggle to publish deadReckoning msg

// Temporary Storage
tf::Quaternion orientation; 
vector<double> angularVelocity;
vector<double> linearVelocity;
double altitude;


void dvlCallback(const slam::DVLConstPtr& msg) { //const uuv_sensor_ros_plugins_msgs::DVLConstPtr &msg --> issues with linking 

    linearVelocity.clear();                             // Update linear velocity
    linearVelocity.push_back(msg->velocity.x);
    linearVelocity.push_back(msg->velocity.y);
    linearVelocity.push_back(msg->velocity.z); 
    altitude = msg->altitude;       
    
    ros::Time msgTime(msg->header.stamp.sec);       // Get the time of the new message
    previousTime = newTime;                         // Update the previous time
    newTime = msgTime;                              // Update newtime
    deadReckon = 1;                                 // Toggle deadreckon node

    // SANITY CHECK
    auto strA = std::to_string(linearVelocity.at(0)); 
    auto strB = std::to_string(linearVelocity.at(1)); 
    auto strC = std::to_string(linearVelocity.at(2));
    cout << "DVL CALLBACK| vx = " << strA << " , vy = " << strB << " , vz = " << strC << "\n";
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) { // Gets IMU data  
    orientation.setW(msg->orientation.w);
    orientation.setX(msg->orientation.x);
    orientation.setY(msg->orientation.y);
    orientation.setZ(msg->orientation.z);

    angularVelocity.push_back(msg->angular_velocity.x);
    angularVelocity.push_back(msg->angular_velocity.y);
    angularVelocity.push_back(msg->angular_velocity.z);
}

void timerCallback(const ros::TimerEvent&){ // Timer available for future msg synchrosition 
}

slam::deadReckoning deadReckoning(vector<double> linearVelocity, double altitude, ros::Time time0, ros::Time time1, tf::Quaternion orientation, vector<double> angularVelocity){ // Provides x,y,z cartesian position and quaternion heading?? (Point and Rot)

    //---------------TIME DIFFERENCE BETWEEN PREVIOUS AND NEW POSE-------------------------------//
    ros::Duration dt = time1-time0;         // USED TO CALCULATE DISTANCE TRAVELLED (X,Y,Z)
    double dt_sec = dt.sec;     

    //-----------------------------------Orientation---------------------------------------------//
    // Z-axis Rotation (Yaw) is used in Position Estimation (radians)

    // Get previous RPY
    tf::Quaternion prevOrientation = prevPose.getRotation();
    tf::Matrix3x3 prevM(prevOrientation);
    double prevRoll, prevPitch, prevYaw;
    prevM.getRPY(prevRoll, prevPitch, prevYaw); // Use prev Yaw (should be in radians)

    // Get new RPY
    tf::Matrix3x3 newM(orientation);
    double newRoll, newPitch, newYaw;
    newM.getRPY(newRoll, newPitch, newYaw);
    newYaw = prevYaw + angularVelocity.at(2)*dt_sec; // Rotation estimation (equation from paper) // Historically Yaw is more inaccurate that roll & pitch

    //-------------------------------Position Estimation-----------------------------------------//
    // Velocity
    double V = sqrt(pow(linearVelocity.at(0),2) + pow(linearVelocity.at(1),2));

    // Previous Position
    tf::Vector3 prevPosition = prevPose.getOrigin();

    //-----------SANITY
    auto strA = std::to_string(prevPosition.w());    
    auto strB = std::to_string(prevPosition.getX());
    auto strC = std::to_string(prevPosition.getY());
    auto strD = std::to_string(prevPosition.getZ());
    cout << "prevPose Origin [" << strA << "," << strB << "," << strC << "," << strD << "]\n";

    // Estimating New Position (X,Y)
    double px = prevPosition.getX() + V*sin(newYaw)*dt_sec;
    double py = prevPosition.getY() + V*cos(newYaw)*dt_sec;

    // --------------------------------SANITY CHECK----------------------------------------------//

    auto strX = std::to_string(px);
    auto strPrevX = std::to_string(prevPosition.getX());
    auto distXTravelled = std::to_string(px - prevPosition.getX());

    auto strY = std::to_string(py); 
    auto strPrevY = std::to_string(prevPosition.getY());
    auto distYTravelled = std::to_string(py - prevPosition.getY());

    cout << "X | New Position: " << strX << " | Prev Position: " << strPrevX << " | Distance Travelled: " << distXTravelled << "\n";
    cout << "Y | New Position: " << strY << " | Prev Position: " << strPrevY << " | Distance Travelled: " << distYTravelled << "\n";

    // -----------------------------UPDATE PREVIOUS POSE-----------------------------------------//
    
    tf::Vector3 newPosition;                // Initialise temporary variable
    tf::Quaternion newOrientation;

    newPosition.setX(px);                   // Fill out temporary variables with new pose data
    newPosition.setY(py);
    newPosition.setZ(altitude);
    newOrientation.setRPY(newRoll,newPitch,newYaw);

    newPose.setOrigin(newPosition);         // Set new pose
    newPose.setRotation(newOrientation);

    prevPose = newPose;

    //--------------------------CREATE DEAD RECKONING MESSAGE----------------------------------//

    slam::deadReckoning drMSG;

    //! Check with ground truth!!
    drMSG.ts = time1;                       // (SEC)
    drMSG.x = px;                           // (METRES) 
    drMSG.y = py;                           // (METRES) 
    drMSG.z = altitude;                     // (METRES) 
    drMSG.roll = newRoll;                   // (RADIANS)  //! Do we need to increment roll pitch yaw??
    drMSG.pitch = newPitch;                 // (RADIANS) 
    drMSG.yaw = newYaw;                     // (RADIANS) VALUE OPTIMISED (HISTORICALLY YAW IS INACCURATE FROM MEASUREMENT DEVICES)

    //------------------------------TOGGLE DEAD RECKON-----------------------------------------//
    deadReckon = 0;                         // Toggle deadreckon node TO WAIT FOR NEXT DVL MSG

    return drMSG;
}

int main(int argc, char ** argv)
{
    // Set up node
    ros::init(argc, argv, "deadReckoning");
    ros::NodeHandle n;

    // Timer (5Hz) DVL frequency
    ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback); //5Hz in line with DVL frequency

    // Subscribers
    ros::Subscriber imu_sub_ = n.subscribe("/bluerov2/imu", 1, &imuCallback); // /slam/imu/data (REMAP!!)
    ros::Subscriber dvl_sub_ = n.subscribe("/bluerov2/dvl",1, &dvlCallback); // /slam/dvl/data (REMAP!!)

    // Publisher
    ros::Publisher deadReckPub = n.advertise<slam::deadReckoning>("deadReckoning", 1);

    // Pose at the start of mission (BlueROV2 pose at start of mission)
    tf::Vector3 position;
    position.setX(0);
    position.setY(0);
    position.setZ(0);
    prevPose.setOrigin(position);

    tf::Quaternion rotation;
    rotation.setW(1);
    rotation.setX(0);
    rotation.setY(0);
    rotation.setZ(0);
    prevPose.setRotation(rotation);

    while(ros::ok())
    {   
        if (deadReckon == 1) {
            slam::deadReckoning msg = deadReckoning(linearVelocity, altitude, previousTime, newTime, orientation, angularVelocity);
            deadReckPub.publish(msg);
            deadReckon = 0;
        }
        ros::spinOnce();
    }
}
