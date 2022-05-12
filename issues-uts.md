
![image](https://user-images.githubusercontent.com/88146518/165717601-8453aaf0-f19a-4fc8-aa23-f0cf22758a9d.png)

This issue surrounding 'GLIBC_2.2.x not found' refers to an incompatible distro install and particular libraries being unavailable (ideally installed on 20.04 with ROS Noetic). Since there is less than 1GB free on the Ubuntu partition, as well as Matt spending a lot of time setting up the workspace for simulation (which is only compatible with ROS Melodic), and the UUV simulator repo has not been checked for cross compatibiility, the system will not be updated to 20.04 for the Oculus Viewpoint software install.

Sources:  
https://forum.juce.com/t/solved-glibc-2-29-not-found-on-ubuntu-18-04/34960/6   https://www.reddit.com/r/JUCE/comments/bbrjsb/how_to_run_juce_project_on_linux/  
https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/dvl_snippets.xacro  
https://uuvsimulator.github.io/packages/uuv_simulator/docs/api/gazebo::DVLROSPlugin/
  
DVL integration below
![image](https://user-images.githubusercontent.com/88146518/165717685-7a3937ba-7ce2-4e2f-8a3f-098670f49907.png)
source devel_isolated/setup.bash
![image](https://user-images.githubusercontent.com/88146518/165717785-32dc2d49-2ba8-46cb-9a1d-a05520fa8c03.png)
![image](https://user-images.githubusercontent.com/88146518/165717808-36b5e481-b268-4d42-8f5e-4d85c23f7baf.png)
  
  # rostopics- bluerov
  ![image](https://user-images.githubusercontent.com/85168871/168003740-e26a76d7-3e36-4a72-9d41-2aa76a5bdda4.png)

  ## converting between ROS Images and OpenCV Images -python 
  create a rosnode for conversion:
  http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
  http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
