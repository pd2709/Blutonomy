/**
 * Node Description: Gets odometry (IMU data (orientation) and DVL data (altitude and linear velocity)) to set the pose of the BlueROV2 UUV
 * 
 * @author Melody Hood
 * @author Pooja Darji
 * @note sanity check cout messaged to terminal are striked out with ////
 */

#include "../include/slam/graph.hpp"            // Allows for SLAM related operations (refer to graph.cpp and Graph based SLAM)
#include <ros/ros.h>                            
#include "gtsam/geometry/concepts.h"            // SLAM specialist library
#include "tf/transform_datatypes.h"
#include <tf/tf.h> 
#include <math.h>
#include <vector>
#include <string>
#include "slam/deadReckoning.h"                 // Rosmsg formats
#include "slam/DVL.h"                           // Allows for the use of DVL rosmsg format [not standard]
#include <sensor_msgs/Imu.h>                    // Allows for the use of IMU rosmsg format [from standard library sensor_msgs]

// Global Variables
tf::Pose prevPose;                              // Updated when deadReckoning is published
tf::Pose newPose;                               // Updated when deadReckoning is published
ros::Time newTime;                              // Updated when deadReckoning is published
ros::Time previousTime;                         // Updated when deadReckoning is published
bool deadReckon = 0;                            // Toggle to publish deadReckoning rosmsg
tf::Quaternion orientation;                     // Temporary Storage
vector<double> angularVelocity;                 // Temporary Storage
geometry_msgs::Vector3 angVel;                  // Temporary Storage
vector<double> linearVelocity;                  // Temporary Storage
double altitude;                                // Temporary Storage
double threshold = 0.1; // Threshold to determine deadreckoning methodology


/**
 * DVL callback gets the latest DVL rosmsg information
 * 
 * @param pointer to the ros message
 * @returns linear velocity of DVL, altitude of DVL, time of measurement
 * @note issues with using the pointer "const uuv_sensor_ros_plugins_msgs::DVLConstPtr &msg" 
 */

void dvlCallback(const slam::DVLConstPtr& msg) {

    linearVelocity.clear();                         // Update linear velocity
    linearVelocity.push_back(msg->velocity.x);
    linearVelocity.push_back(msg->velocity.y);
    linearVelocity.push_back(msg->velocity.z); 
    altitude = msg->altitude;       
    
    ros::Time msgTime(msg->header.stamp.sec);       // Get the time of the new message
    previousTime = newTime;                         // Update the previous time
    newTime = msgTime;                              // Update newtime
    deadReckon = 1;                                 // Toggle deadreckon node

    //// auto strA = std::to_string(linearVelocity.at(0)); 
    //// auto strB = std::to_string(linearVelocity.at(1)); 
    //// auto strC = std::to_string(linearVelocity.at(2));
    //// cout << "DVL CALLBACK| vx = " << strA << " , vy = " << strB << " , vz = " << strC << "\n";
}

/**
 *  IMU callback gets the latest IMU rosmsg information
 * 
 *  @param pointer to the ros message
 *  @returns orientation of the IMU & angular velocity of IMU
 *  @note This IMU returns the orientation not the change in orientation like some available IMUs
 */
void imuCallback(const sensor_msgs::ImuConstPtr& msg) {

    orientation.setW(msg->orientation.w);
    orientation.setX(msg->orientation.x);
    orientation.setY(msg->orientation.y);
    orientation.setZ(msg->orientation.z);

    angularVelocity.clear(); 
    angularVelocity.push_back(msg->angular_velocity.x);
    angularVelocity.push_back(msg->angular_velocity.y);
    angularVelocity.push_back(msg->angular_velocity.z);
}

/**
 *  Timer callback for future message synchronisation
 *  @param pointer to the ros message
 *  @note to be used for future rosmsg synchronisation
 */

void timerCallback(const ros::TimerEvent&){
}

/**
 *  Dead reckoning function integrates IMU and DVL data for dead reckoning report
 *  Dead reckoning report is published to /deadReckoning
 *  @param linearVelocity from DVL
 *  @param altitude from DVL
 *  @param time0 from DVL. This is the time published with the previous DVL rosmsg
 *  @param time1 from DVL. This is the time published with the new DVL rosmsg
 *  @param orientation from IMU
 *  @param angularVelocity from IMU
 *  @returns deadReckoning rosmsg
 *  @note the function is toggled on after every DVL rosmsg callback and is toggled off once the function is complete
 *  @note Z-axis rotation (yaw) is used for position estimation. Yaw is historically more inaccurate than roll & pith which is why it is estimated using angularVelocity
 *  @note displacement and rotation are in metres and radians
 */
slam::deadReckoning deadReckoning(vector<double> linearVelocity, double altitude, ros::Time time0, ros::Time time1, tf::Quaternion orientation, vector<double> angularVelocity){ // Provides x,y,z cartesian position and quaternion heading?? (Point and Rot)

    //---------------TIME DIFFERENCE BETWEEN PREVIOUS AND NEW POSE-------------------------------//
    ros::Duration dt = time1-time0;                             // To calculate distance
    double dt_sec = dt.sec;     

    //-----------------------------------Orientation---------------------------------------------//
    tf::Quaternion prevOrientation = prevPose.getRotation();    // Get previous RPY
    tf::Matrix3x3 prevM(prevOrientation);
    double prevRoll, prevPitch, prevYaw;
    prevM.getRPY(prevRoll, prevPitch, prevYaw);

    tf::Matrix3x3 newM(orientation);                            // Get new RPY
    double newRoll, newPitch, newYaw;
    newM.getRPY(newRoll, newPitch, newYaw);

    newYaw = prevYaw + angularVelocity.at(2)*dt_sec;            // Estimate yaw rotation (Equation 12 from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7913909/paper) 
    //// auto strz = std::to_string(newYaw);
    //// cout << "\n NEW YAW: " << strz << "\n\n";

    //-------------------------------POSITION ESTIMATION-----------------------------------------//

    double V = sqrt(pow(linearVelocity.at(0),2) + pow(linearVelocity.at(1),2));         // Get velocity (Equation 15 from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7913909/) //! Omitted from Method 2
    double vx = linearVelocity.at(0);                                                   // X component of linear velocity
    double vy = linearVelocity.at(1);                                                   // Y component of linear velocity

    tf::Vector3 prevPosition = prevPose.getOrigin();                                    // Getting previous position

    //// auto strA = std::to_string(prevPosition.w());    
    //// auto strB = std::to_string(prevPosition.getX());
    //// auto strC = std::to_string(prevPosition.getY());
    //// auto strD = std::to_string(prevPosition.getZ());
    //// cout << "prevPose Origin [" << strA << "," << strB << "," << strC << "," << strD << " | New yaw: " << newYaw << "\n";

    double normYaw = fmod(newYaw + M_PI, 2 * M_PI);                                     // Normalise yaw
    if (normYaw < 0){
        normYaw += 2 * M_PI;
    }
    normYaw -= M_PI;

    double px = prevPosition.getX() + vx*cos(normYaw)*dt_sec + vy*sin(normYaw)*dt_sec;  // MODEL 2 (Optimised)
    double py = prevPosition.getY() + vx*sin(normYaw)*dt_sec + vy*cos(normYaw)*dt_sec;

    //// auto strX = std::to_string(px);
    //// auto strPrevX = std::to_string(prevPosition.getX());
    //// auto distXTravelled = std::to_string(px - prevPosition.getX());

    //// auto strY = std::to_string(py); 
    //// auto strPrevY = std::to_string(prevPosition.getY());
    //// auto distYTravelled = std::to_string(py - prevPosition.getY());

    //// cout << "X | New Position: " << strX << " | Prev Position: " << strPrevX << " | Distance Travelled: " << distXTravelled << "\n";
    //// cout << "Y | New Position: " << strY << " | Prev Position: " << strPrevY << " | Distance Travelled: " << distYTravelled << "\n";

    // -----------------------------UPDATE PREVIOUS POSE-----------------------------------------//
    
    tf::Vector3 newPosition;                // Initialise temporary variable
    tf::Quaternion newOrientation;

    newPosition.setX(px);                   // Fill out temporary variables with new pose data
    newPosition.setY(py);
    newPosition.setZ(altitude);
    newOrientation.setRPY(newRoll,newPitch,newYaw);

    newPose.setOrigin(newPosition);         // Set new pose
    newPose.setRotation(newOrientation);    // newPose.setRotation(newOrientation);

    prevPose = newPose;

    //--------------------------CREATE DEAD RECKONING MESSAGE----------------------------------//

    slam::deadReckoning drMSG;
    drMSG.ts = time1;                       // (SEC)
    drMSG.x = px;                           // (METRES) 
    drMSG.y = py;                           // (METRES) 
    drMSG.z = altitude;                     // (METRES) 
    drMSG.roll = newRoll;                   // (RADIANS) RPY does not need to be incremented
    drMSG.pitch = newPitch;                 // (RADIANS) 
    drMSG.yaw = newYaw;                     // (RADIANS) OPTIMISED (historically yaw is inaccurate from measurement devices)

    //------------------------------TOGGLE DEAD RECKON-----------------------------------------//
    deadReckon = 0;                         // Toggle deadreckon node to wait for next DVL message

    return drMSG;
}

int main(int argc, char ** argv)
{
    // Set up node
    ros::init(argc, argv, "deadReckoning");
    ros::NodeHandle n;

    // Timer (5Hz) DVL frequency
    ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback); // 5Hz in line with DVL frequency //! Not used

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
