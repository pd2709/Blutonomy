
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
`rostopic list`  
/bluerov2/camera_info  
/bluerov2/current_velocity  
/bluerov2/current_velocity_marker  
/bluerov2/depth/camera_info_real_info  
/bluerov2/depth/image_raw  
/bluerov2/depth/image_raw_depth_raw_sonar  
/bluerov2/depth/image_raw_multibeam  
/bluerov2/depth/image_raw_normals  
/bluerov2/depth/image_raw_raw_sonar  
/bluerov2/depth/image_raw_sonar  
/bluerov2/image_raw  
/bluerov2/image_raw/compressed  
/bluerov2/image_raw/compressed/parameter_descriptions  
/bluerov2/image_raw/compressed/parameter_updates  
/bluerov2/image_raw/compressedDepth  
/bluerov2/image_raw/compressedDepth/parameter_descriptions  
/bluerov2/image_raw/compressedDepth/parameter_updates  
/bluerov2/image_raw/theora  
/bluerov2/image_raw/theora/parameter_descriptions  
/bluerov2/image_raw/theora/parameter_updates  
/bluerov2/imu  
/bluerov2/imu/state  
/bluerov2/is_submerged  
/bluerov2/joint_states  
/bluerov2/points  
/bluerov2/pose_gt  
/bluerov2/pose_gt/state  
/bluerov2/pressure  
/bluerov2/pressure/state  
/bluerov2/thrusters/0/dynamic_state_efficiency  
/bluerov2/thrusters/0/input  
/bluerov2/thrusters/0/is_on  
/bluerov2/thrusters/0/thrust  
/bluerov2/thrusters/0/thrust_efficiency  
/bluerov2/thrusters/0/thrust_wrench  
/bluerov2/thrusters/1/dynamic_state_efficiency  
/bluerov2/thrusters/1/input  
/bluerov2/thrusters/1/is_on  
/bluerov2/thrusters/1/thrust  
/bluerov2/thrusters/1/thrust_efficiency  
/bluerov2/thrusters/1/thrust_wrench  
/bluerov2/thrusters/2/dynamic_state_efficiency  
/bluerov2/thrusters/2/input  
/bluerov2/thrusters/2/is_on  
/bluerov2/thrusters/2/thrust  
/bluerov2/thrusters/2/thrust_efficiency  
/bluerov2/thrusters/2/thrust_wrench  
/bluerov2/thrusters/3/dynamic_state_efficiency  
/bluerov2/thrusters/3/input  
/bluerov2/thrusters/3/is_on  
/bluerov2/thrusters/3/thrust  
/bluerov2/thrusters/3/thrust_efficiency  
/bluerov2/thrusters/3/thrust_wrench  
/bluerov2/thrusters/4/dynamic_state_efficiency  
/bluerov2/thrusters/4/input  
/bluerov2/thrusters/4/is_on  
/bluerov2/thrusters/4/thrust  
/bluerov2/thrusters/4/thrust_efficiency  
/bluerov2/thrusters/4/thrust_wrench  
/bluerov2/thrusters/5/dynamic_state_efficiency  
/bluerov2/thrusters/5/input  
/bluerov2/thrusters/5/is_on  
/bluerov2/thrusters/5/thrust  
/bluerov2/thrusters/5/thrust_efficiency  
/bluerov2/thrusters/5/thrust_wrench  
/bluerov2/using_global_current_velocity  
/clock  
/diagnostics  
/gazebo/link_states  
/gazebo/model_states  
/gazebo/parameter_descriptions  
/gazebo/parameter_updates  
/gazebo/set_link_state  
/gazebo/set_model_state  
/hydrodynamics/current_velocity  
/mavlink/from  
/mavros/battery  
/mavros/global_position/compass_hdg  
/mavros/global_position/global  
/mavros/global_position/local  
/mavros/global_position/raw/fix  
/mavros/global_position/raw/gps_vel  
/mavros/global_position/raw/satellites  
/mavros/global_position/rel_alt  
/mavros/gpsstatus/gps1/raw  
/mavros/imu/data  
/mavros/imu/data_raw  
/mavros/imu/diff_pressure  
/mavros/imu/mag  
/mavros/imu/static_pressure  
/mavros/imu/temperature_baro  
/mavros/mission/waypoints  
/mavros/mount_control/orientation  
/mavros/mount_control/status  
/mavros/nav_controller_output  
/mavros/rc/in  
/mavros/rc/out  
/mavros/state  
/mavros/time_reference  
/mavros/timesync_status  
/mavros/vfr_hud  
/oculus/header  
/oculus/polarscan <-- this one  
/rosout  
/rosout_agg  
/tf  
/tf_static  
    
`rostopic info /oculus/polarscan`  
![image](https://user-images.githubusercontent.com/85168871/168223356-721eba2a-3681-4a2f-bf9d-00ea3c4ee8c6.png)  
  
  ## converting between ROS Images and OpenCV Images -python 
  create a rosnode for conversion:
  http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
  http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
