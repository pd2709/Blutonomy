# Complications - UTS AUT 2022
This document entails the complications faced by Team SPR during the Blutonomy Project.

## Oculus Viewpoint Compalibility with UBUNTU OS - Blutonomy Project Laptop
This issue surrounding 'GLIBC_2.2.x not found' refers to an incompatible distro install and particular libraries being unavailable (ideally installed on 20.04 with ROS Noetic). Since there is less than 1GB free on the Ubuntu partition, as well as Matt spending a lot of time setting up the workspace for simulation (which is only compatible with ROS Melodic), and the UUV simulator repo has not been checked for cross compatibiility, the system will not be updated to 20.04 for the Oculus Viewpoint software install.

The following screenshot shows the problem:
![image](https://user-images.githubusercontent.com/88146518/165717601-8453aaf0-f19a-4fc8-aa23-f0cf22758a9d.png)

Sources:  
https://forum.juce.com/t/solved-glibc-2-29-not-found-on-ubuntu-18-04/34960/6   
https://www.reddit.com/r/JUCE/comments/bbrjsb/how_to_run_juce_project_on_linux/  

## Understanding plugins & existing BLUEROV2 code on the Blutonomy Laptop

Links that assisted in initial understanding:
https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/dvl_snippets.xacro  
https://uuvsimulator.github.io/packages/uuv_simulator/docs/api/gazebo::DVLROSPlugin/
 
## Converting between ROS Images and OpenCV Images -python 
Create a rosnode for conversion:
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
  
## Low Disk Space on Filesystem root  
![image](https://user-images.githubusercontent.com/85168871/172077211-0dff1030-2844-4868-936b-9b6fceea4891.png)  

## Issues with QGroundControl Testing 
  
During testing on the 12/4/2022, the team operated BlueROV2 in Ubuntu and had difficulties with control. We realised that we had to configure the controller settings for QGroundContol in which we copied the same layout as shown on the Pre-Dive Checklist.

https://discuss.bluerobotics.com/t/issue-with-stabilize-and-depth-hold-flight-modes/861/2

## Bluetooth Connection Issues

## WiFi Connection Issues

