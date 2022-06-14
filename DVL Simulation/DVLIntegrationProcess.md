# DVL Integration

## Initial Research
- https://uuvsimulator.github.io/packages/uuv_simulator/docs/api/gazebo::DVLROSPlugin/
- https://github-wiki-see.page/m/Field-Robotics-Lab/dave/wiki/whn_dvl_examples
- https://answers.gazebosim.org//question/13966/examples-of-registering-a-custom-sensor-not-plugin/#17653
- https://answers.gazebosim.org//question/20324/unable-to-create-sensor-of-type/
- https://github.com/Field-Robotics-Lab/dave/wiki/dvl_description
- https://github.com/Field-Robotics-Lab/dave/wiki/Notes


### DVL A50 - WaterLink
https://store.waterlinked.com/product/dvl-a50/  
Transducer setup : 4-beam convex Janus array   
  
### Exploring Matt's Simulation
![image](https://user-images.githubusercontent.com/85168871/166186423-68b4d3c3-4b3b-4755-ba9e-9c35738e18ef.png)  
Note: DVLROSPlugin() exists in this workspace 

### How To Open an Image  
![image](https://user-images.githubusercontent.com/85168871/166187026-c0fae98d-e88c-4204-b45c-3b7e7d2df766.png)  
Note: `eog` command can be used to view an image in terminal

### Exploring URDF
Our tutor Richardo reccomended us to start with looking at the 'URDF'. Matt's documentation reccomends us to add a sonar plugin to the BlueROV2 model by adding particular text to the sensors.xacro file of the BlueROV2 package.  

Disclaimer: Team SPR does not have access to this repository and will not be given access due to the complexity and time delay of gaining access in the Thales system.  
  
Here is the location of the sensors.xacro file:  
![image](https://user-images.githubusercontent.com/85168871/166187496-9ca050ac-b40c-4bb8-ad58-ab42b51353e8.png)  

Sources:
Build a Custom Robot in ROS | URDF | ROS Tutorial for Beginners: https://www.youtube.com/watch?v=JFpg7vG8NXE  
How do we describe a robot? With URDF! | Getting Ready to build Robots with ROS #7 https://www.youtube.com/watch?v=CwdbsvcpOHM  
   
What is included in the urdf: colours, material and links (think of Denavit-Hartenberg Parameters)  

## Xacro  
![image](https://user-images.githubusercontent.com/85168871/166198932-bc03f6e3-8ef1-4c24-b512-9290727cc20b.png)  
  

  
### Current Error
	[ERROR]: Mesh scale was specified, but could not be parsed: Parser found 0 elements but 3 expected while parsing vector [  ]
	[ERROR]: Could not parse visual element for Link [bluerov2/dvl_link]
	[ERROR]: Failed to build tree: parent link [bluerov2/baselink] of joint [bluerov2/dvl_joint] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [bluerov2/dvl_joint] from your urdf file, or add "<link name="bluerov2/baselink" />" to your urdf file.
    
## Further Notes

Wiki on xacro:
https://wiki.ros.org/xacro 

subscriber and publisher for DVL info (python code):
https://github.com/waterlinked/dvl-a50-ros-driver/tree/master/scripts 

DVL sensor msgs:
http://docs.ros.org/en/hydro/api/underwater_sensor_msgs/html/msg/DVL.html 

documentation on a submarine simulator with gazebo (example has good instructions):
https://github.com/paagutie/submarine/blob/master/Marum_simulator.pdf

Add the following lines to 'sensors.xacro' to add dvl sensor:

	 <xacro:default_dvl namespace="${namespace}" parent_link="${namespace}/base_link">
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	  </xacro:default_dvl>

How to - Gazebo tutorials for sensors:
https://classic.gazebosim.org/tutorials?cat=sensors 

## Integration Progress Snapshots
Terminal showing `catkin_make_isolated` has run successfully as well as the launch file.
![Terminal](https://user-images.githubusercontent.com/88146518/169041620-3b5f74cd-0a7d-4434-ab20-de668446fb02.png)

BlueRov in Gazebo visually showing the DVL sensor in Red (on top of the bluerov)
![GazeboDVL](https://user-images.githubusercontent.com/88146518/169041673-3311d174-8b7c-4ffc-a547-517036faff44.png)

RVIZ with BLUEROV and TF added (showing the DVL topics on left)
![rvizDVL](https://user-images.githubusercontent.com/88146518/169041704-8b472998-aa01-47c4-a19d-8bc2fc3ab615.png)

rostopic list (highlighting DVL topics)
![rostopics](https://user-images.githubusercontent.com/88146518/169042576-11041dce-70f0-42dd-8c6f-a26d156a8ec3.png)

rostopic info
- `/bluerov2/dvl`
![image](https://user-images.githubusercontent.com/88146518/169042738-cd1f1b97-be38-4cc9-a8fe-a68a2366c49b.png)

- `/blurov2/dvl/state`
![image](https://user-images.githubusercontent.com/88146518/169042950-c3bfed79-2fa5-42a3-85b6-535cdd16b882.png)

- `/bluerov2/dvl_sonar0` 
![image](https://user-images.githubusercontent.com/88146518/169043246-f7047921-b73b-4afd-b59e-3aa6a8f70440.png)

- `/bluerov2/dvl_sonar1`
![image](https://user-images.githubusercontent.com/88146518/169043279-18794c48-4e13-4eb8-a3fd-4a0eb5c577e6.png)

- `/bluerov2/dvl_sonar2`
![image](https://user-images.githubusercontent.com/88146518/169043326-89b50f9f-1011-4e51-80fa-21fe4656f34a.png)

- `/bluerov2/dvl_sonar3`
![image](https://user-images.githubusercontent.com/88146518/169043375-b62f7254-28d2-4c76-93de-d0fb01b41990.png)

- `/bluerov2/dvl_twist`
![image](https://user-images.githubusercontent.com/88146518/169043400-7570d3f8-4cca-45e7-ba26-d51e54116d7e.png)


### Rostopic List (18/05/2022)
	/bluerov2/automatic_on
	/bluerov2/camera_info
	/bluerov2/cmd_vel
	/bluerov2/current_velocity
	/bluerov2/current_velocity_marker
	/bluerov2/depth/camera_info_real_info
	/bluerov2/depth/image_raw
	/bluerov2/depth/image_raw_depth_raw_sonar
	/bluerov2/depth/image_raw_multibeam
	/bluerov2/depth/image_raw_normals
	/bluerov2/depth/image_raw_raw_sonar
	/bluerov2/depth/image_raw_sonar
	/bluerov2/dp_controller/error
	/bluerov2/dp_controller/input_trajectory
	/bluerov2/dp_controller/reference
	/bluerov2/dp_controller/trajectory
	/bluerov2/dp_controller/waypoints
	/bluerov2/dvl
	/bluerov2/dvl/state
	/bluerov2/dvl_sonar0
	/bluerov2/dvl_sonar1
	/bluerov2/dvl_sonar2
	/bluerov2/dvl_sonar3
	/bluerov2/dvl_twist
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
	/bluerov2/interpolator_visual_markers
	/bluerov2/is_submerged
	/bluerov2/joint_states
	/bluerov2/points
	/bluerov2/pose_gt
	/bluerov2/pose_gt/state
	/bluerov2/pressure
	/bluerov2/pressure/state
	/bluerov2/station_keeping_on
	/bluerov2/thruster_manager/input
	/bluerov2/thruster_manager/input_stamped
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
	/bluerov2/time_to_target
	/bluerov2/trajectory_tracking_on
	/bluerov2/using_global_current_velocity
	/clock
	/gazebo/link_states
	/gazebo/model_states
	/gazebo/parameter_descriptions
	/gazebo/parameter_updates
	/gazebo/set_link_state
	/gazebo/set_model_state
	/ground_truth_to_tf_bluerov2/euler
	/ground_truth_to_tf_bluerov2/pose
	/hydrodynamics/current_velocity
	/rosout
	stopic /rosout_agg
	/tf
	/tf_static

### Rostopic echo /bluerov2/dvl  
    header:  
      seq: 746   
      stamp:   
        secs: 107   
        nsecs: 628000000   
      frame_id: "bluerov2/dvl_link"   
    velocity:   
      x: -2.05290318627e-08   
      y: 0.0122516323095  
      z: 7.05966919942e-06  
    velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
    altitude: 61.629863739  
    beams:  
    
    range: 81.0  
    range_covariance: 0.0  
    velocity: 0.0  
    velocity_covariance: 0.0  
    pose:   
      header:   
        seq: 0  
        stamp:   
          secs: 0  
          nsecs: 204000000  
        frame_id: "bluerov2/dvl_sonar0_link"  
      pose:   
        position:   
          x: 0.0  
          y: 0.0  
          z: 0.0  
        orientation:   
          x: 0.0  
          y: -1.0  
          z: 0.0  
          w: 0.966389978135  
     
    range: 81.0  
    range_covariance: 0.0  
    velocity: 0.0  
    velocity_covariance: 0.0  
    pose:   
      header:   
        seq: 0  
        stamp:   
          secs: 0  
          nsecs: 204000000  
        frame_id: "bluerov2/dvl_sonar1_link"  
      pose:   
        position:   
          x: 0.0  
          y: 0.0  
          z: 0.0  
        orientation:   
          x: 0.0  
          y: 0.0  
          z: 1.0  
          w: 0.966389978135  
       
    range: 3.51945090294  
    range_covariance: 0.0  
    velocity: 0.0  
    velocity_covariance: 0.0  
    pose:   
      header:   
        seq: 0  
        stamp:   
          secs: 0  
          nsecs: 204000000  
        frame_id: "bluerov2/dvl_sonar2_link"  
      pose:   
        position:   
          x: 0.0   
          y: 0.0  
          z: 0.0  
        orientation:   
          x: 0.0  
          y: 1.0  
          z: 0.0  
          w: 0.966389978135  
       
    range: 81.0  
    range_covariance: 0.0  
    velocity: 0.0  
    velocity_covariance: 0.0  
    pose:   
      header:   
        seq: 0  
        stamp:   
          secs: 0  
          nsecs: 204000000  
        frame_id: "bluerov2/dvl_sonar3_link"  
      pose:   
        position:   
          x: 0.0  
          y: 0.0  
          z: 0.0  
        orientation:   
          x: 0.0  
          y: 0.0  
          z: -1.0  
          w: 0.966389978135`  
---  
# Rostopic echo bluerov2/imu
	header:   
	  seq: 3012  
	  stamp:   
	    secs: 60  
	    nsecs: 262000000  
	  frame_id: "bluerov2/imu1_link"  
	orientation:   
	  x: 2.03530660864e-05  
	  y: -0.00686565444963  
	  z: 8.66754028487e-06  
	  w: 0.999976430872  
	orientation_covariance: [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]  
	angular_velocity:   
	  x: -1.45231253002e-08  
	  y: -0.00225401373389  
	  z: -1.32590579578e-05  
	angular_velocity_covariance: [1.1519236000000001e-07, 0.0, 0.0, 0.0, 1.1519236000000001e-07, 0.0, 0.0, 0.0, 1.1519236000000001e-07]  
	linear_acceleration:   
	  x: 0.134477133585  
	  y: -7.32809458324e-06  
	  z: 9.80998584481  
	linear_acceleration_covariance: [1.6e-05, 0.0, 0.0, 0.0, 1.6e-05, 0.0, 0.0, 0.0, 1.6e-05]


## Code Research  
Quaternion to RPY: https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac  


## DVL Dead reckoning Expected Report
![image](https://user-images.githubusercontent.com/88146518/172031094-61678c77-81ac-40f7-b7a5-feac2470ee52.png)  
![image](https://user-images.githubusercontent.com/88146518/172031123-a5f4937d-1666-4c2c-8294-5f5ae50bd32e.png)  
https://waterlinked.github.io/dvl/dvl-protocol/#dead-reckoning-report  
Figure of merit: https://www.sciencedirect.com/topics/chemistry/figure-of-merit  
### Quaternion to RPY  
https://math.stackexchange.com/questions/147028/are-euler-angles-the-same-as-pitch-roll-and-yaw  
https://stackoverflow.com/questions/36501262/quaternion-to-rotation-matrix-incorrect-values-using-eigen-library  
https://answers.ros.org/question/58742/quaternion-to-roll-pitch-yaw/  
https://answers.ros.org/question/58863/incorrect-rollpitch-yaw-values-using-getrpy/  

### Timing
https://answers.ros.org/question/229445/how-can-i-get-time-difference-of-two-messages-published-by-same-publisher-on-same-topic/   


### Blutonomy Creating dead reckoning msg
Successful make:

![image](https://user-images.githubusercontent.com/88146518/172032367-ec6ce46a-a7be-4b19-a87e-40f4562e7fbb.png)

deadReckoning.msg file:

![image](https://user-images.githubusercontent.com/88146518/172032384-102b6656-c8f9-44de-8cac-ecce87104dbf.png)

![image](https://user-images.githubusercontent.com/88146518/172049726-9c46fe81-8eb1-4283-b18d-a3b7076a3ea2.png)

rostopic echo /deadReckoning:

	header: 
	  seq: 771
	  stamp: 
	    secs: 0
	    nsecs:         0
	  frame_id: ''
	ts: 
	  secs: 628000000
	  nsecs:         0
	x: -2.95618058823
	y: 1764235.05257
	z: 1016.59236472
	std: 0.0
	roll: -0.000600288845046
	pitch: -0.0494595149374
	yaw: 8.71770664959e-05
	type: ''
	status: False
	format: ''
	---
			

# SLAM

 `cd ~/matt_thesis/catkin_ws`
 `source devel_isolated/setup.bash`
 
 ### rostopic echo /slam/result

       - 
         header: 
           seq: 0
           stamp: 
             secs: 58
             nsecs: 222000000
           frame_id: "L19"
         pose: 
           pose: 
             position: 
               x: 14.0382997787
               y: 0.105515084965
               z: -1.35599182296
             orientation: 
               x: 0.0
               y: 0.0
               z: 0.0
               w: 0.0
           covariance: [0.002305698082329353, -0.0007677795560456066, 0.0010750016963873218, 0.0, 0.0, 0.0, -0.0007677795560456064, 0.056302038407862534, 5.134805061173342e-06, 0.0, 0.0, 0.0, 0.0010750016963873218, 5.134805061173269e-06, 0.011561871304630702, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       - 
         header: 
           seq: 0
           stamp: 
             secs: 58
             nsecs: 222000000
           frame_id: "L20"
         pose: 
           pose: 
             position: 
               x: 14.0446038384
               y: -3.82828993916
               z: -1.34783573859
             orientation: 
               x: 0.0
               y: 0.0
               z: 0.0
               w: 0.0
           covariance: [0.007052884718071339, 0.014926803346030564, 0.0012957136018633853, 0.0, 0.0, 0.0, 0.014926803346030568, 0.05802773029243345, -0.0003205802179232825, 0.0, 0.0, 0.0, 0.0012957136018633853, -0.0003205802179232825, 0.015135254814819218, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       - 
         header: 
           seq: 0
           stamp: 
             secs: 58
             nsecs: 222000000
           frame_id: "L21"
         pose: 
           pose: 
             position: 
               x: 14.019665929
               y: 4.04250199978
               z: -1.34378859846
             orientation: 
               x: 0.0
               y: 0.0
               z: 0.0
               w: 0.0
           covariance: [0.00930121207676022, -0.018973998857165417, 0.0018311515109201342, 0.0, 0.0, 0.0, -0.01897399885716542, 0.06411410382598012, 0.0005777165192690222, 0.0, 0.0, 0.0, 0.0018311515109201342, 0.0005777165192690223, 0.022039704461262775, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
     var_V: 
       x: 0.284938276365
       y: 0.0255306619807
       z: -0.00803038288944
     var_B_acc: 
       x: -0.091645883217
       y: 0.00959798827928
       z: 0.00393975414902
     var_B_gyr: 
       x: 2.8668825083e-05
       y: -0.00100418144986
       z: -1.31551933291e-05
     ---
### rostopic echo /slam/features
 - polar coordinate of each detected object at that particular time 

    header:  
      seq: 619
      stamp: 
        secs: 61
        nsecs: 422000000
      frame_id: ''
    features: 
      -  
        header:  
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        range: 5.95500659943
        elevation: 0.0296018365771
        bearing: 0.342063695192
        id: 0
      -  
        header:  
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        range: 7.83522987366
        elevation: 0.0746018365026
        bearing: 0.263397604227
        id: 0
      -   
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        range: 7.80300474167
        elevation: 0.0746018365026
        bearing: -0.247620657086
        id: 0
      -   
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        range: 5.90076971054
        elevation: 0.0296018365771
        bearing: -0.337694495916
        id: 0

## Correlation
Note: There is a header for each detected feature at a particular time step  

### RVIZ
![image](https://user-images.githubusercontent.com/85168871/171074917-7511ef50-72f3-4c7d-b241-632e84e59489.png)
  
### rostopic echo /slam/features   
    header: 
      seq: 24
      stamp: 
        secs: 57
        nsecs: 622000000
      frame_id: ''
    features: 
      -   
        header: << OBJECT 1 >>
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        range: 8.23590373993
        elevation: 0.124601833522
        bearing: 0.2449785918
        id: 0
      -   
        header:  << OBJECT 2 >>
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        range: 8.17437553406
        elevation: 0.124601833522
        bearing: -0.236151650548
        id: 0
      - 
        header: << OBJECT 3 >>
          seq: 0
          stamp: 
            secs: 0
            nsecs:         0
          frame_id: ''
        range: 6.30333900452
        elevation: 0.0846018344164
        bearing: -0.315354079008
        id: 0
    ---
### Rostopic echo mavros/imu/data
![image](https://user-images.githubusercontent.com/85168871/172029735-158b9689-2d15-4b7c-bb17-2cb95977a47d.png)  
  
    header: 
      seq: 13657
      stamp: 
        secs: 1649718027
        nsecs: 495378389
      frame_id: "base_link"
    orientation: 
      x: 0.00415225432238
      y: -0.0243123515396
      z: -0.669372864911
      w: -0.742517094798
    orientation_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    angular_velocity: 
      x: -0.00022425246425
      y: 0.0008114639204
      z: 0.000987689942122
    angular_velocity_covariance: [1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07]
    linear_acceleration: 
      x: -0.3334261
      y: 0.0588399
      z: 9.85568325
    linear_acceleration_covariance: [8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08]
    ---

 
## Extended Kalman Filter  

### Lighter reading
Generic Kalman Filter: https://openimu.readthedocs.io/en/latest/algorithms/KalmanFilter.html  

### Deeper reading
https://robotics.stackexchange.com/questions/964/extended-kalman-filter-using-odometry-motion-model  
https://mdpi-res.com/d_attachment/sensors/sensors-11-09182/article_deploy/sensors-11-09182.pdf?version=1403316184  

## AHRS   

### Readings  
AHRS Code cpp: https://github.com/emlid/Navio/blob/master/C%2B%2B/Examples/AHRS/AHRS.cpp    
AHRS Code hpp: https://github.com/emlid/Navio/blob/master/C%2B%2B/Examples/AHRS/AHRS.hpp  

## Example SLAM Code  
https://github.com/WaldumA/sonar_slam/blob/c638b6ecf00101b16d9ff62a97267ca29e6ef083/src/sonar_slam.cpp   
https://github.com/MohamedMehery/Localization-package-AUV/blob/84fba624f7ad61e58475d19d512ba1aeeec0d119/navigation/underwater_odom/src/underwater_odom_node.cpp   
https://github.com/luansilveira/dolphin_slam/blob/e58680a48fdd9f23b98b733f0a654e8a945cf5de/src/dolphin_slam/robot_state.cpp   

## Ground Truth 

/ground_truth_to_tf_bluerov2/pose

Collect in rosbag.
Assume roll and pitch = 0 and calculate Yaw.
