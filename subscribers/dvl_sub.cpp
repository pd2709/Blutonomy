//subscriber
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

//include any other 

void dvlCallBack(const sensor_msgs::Dvl::ConstPtr& msg)    
{
   ROS_INFO("DVL-> Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
           msg->angular_velocity.x,
           msg->angular_velocity.y, msg->angular_velocity.z, msg->orientation.x, msg->orientation.y, msg->orientation.z,
           msg->orientation.w);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_dvl_subscriber");
  ros::NodeHandle n;
  ros::Subscriber subDvl = n.subscribe("/slam/dvl/data", 10, dvlCallBack);

  ros::spin();
  return 0;
}


//sources used:
//https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
// https://www.stereolabs.com/docs/ros/sensor-data/
//https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_sensors_sub_tutorial/src/zed_sensors_sub_tutorial.cpp
//http://wiki.ros.org/evarobot_minimu9/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20IMU
//https://github.com/mavlink/mavros/issues/39
