#include "ros/ros.h"
#include "std_msgs/String.h"

//include any other 

void imuCallback(const std_msgs::String::ConstPtr& msg)     //need to input correct data type
{
  ROS_INFO("I heard: [%s]", msg->data.c_str()); //update ROS INFO
}

void imuDataCallback(const std_msgs::String::ConstPtr& msg)     //need to input correct data type
{
  ROS_INFO("I heard: [%s]", msg->data.c_str()); //update ROS INFO
}

void imuDatarawCallback(const std_msgs::String::ConstPtr& msg)     //need to input correct data type
{
  ROS_INFO("I heard: [%s]", msg->data.c_str()); //update ROS INFO
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mavros_imu_subscriber");
  ros::NodeHandle n;
  ros::Subscriber subImu = n.subscribe("/mavros/imu", 10, imuCallBack);
  ros::Subscriber subImuData = n.subscribe("/mavros/imu/data", 10, imuDataCallback);
  ros::Subscriber subImuDataRaw = n.subscribe("/mavros/imu/data_raw", 10, imuDatarawCallback);
  ros::spin();
  return 0;
}


//sources used:
//https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
// https://www.stereolabs.com/docs/ros/sensor-data/
//https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_sensors_sub_tutorial/src/zed_sensors_sub_tutorial.cpp
//http://wiki.ros.org/evarobot_minimu9/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20IMU
