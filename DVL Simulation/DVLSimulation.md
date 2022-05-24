# DVL Integration
- https://uuvsimulator.github.io/packages/uuv_simulator/docs/api/gazebo::DVLROSPlugin/
- https://github-wiki-see.page/m/Field-Robotics-Lab/dave/wiki/whn_dvl_examples
- https://answers.gazebosim.org//question/13966/examples-of-registering-a-custom-sensor-not-plugin/#17653
- https://answers.gazebosim.org//question/20324/unable-to-create-sensor-of-type/

## A50 - WaterLink
https://store.waterlinked.com/product/dvl-a50/  
Transducer setup 	: 4-beam convex Janus array   
  
# Exploring Matt's Simulation
  
![image](https://user-images.githubusercontent.com/85168871/166186423-68b4d3c3-4b3b-4755-ba9e-9c35738e18ef.png)  
Note: DVLROSPlugin() exists in this workspace 

## How To Open an Image  
![image](https://user-images.githubusercontent.com/85168871/166187026-c0fae98d-e88c-4204-b45c-3b7e7d2df766.png)  
Note: `eog` command can be used to view an image in terminal

## Exploring URDF
Our tutor Richardo reccomended us to start with looking at the 'URDF'. Matt's documentation reccomends us to add a sonar plugin to the BlueROV2 model by adding particular text to the sensors.xacro file of the BlueROV2 package.  
(Insert image of Matt's repo)  
  
Disclaimer: Team SPR does not have access to this repository and will not be given access due to the complexity and time delay of gaining access in the Thales system.  
  
Here is the location of the sensors.xacro file:  
![image](https://user-images.githubusercontent.com/85168871/166187496-9ca050ac-b40c-4bb8-ad58-ab42b51353e8.png)  
Sources:
Build a Custom Robot in ROS | URDF | ROS Tutorial for Beginners: https://www.youtube.com/watch?v=JFpg7vG8NXE  
How do we describe a robot? With URDF! | Getting Ready to build Robots with ROS #7 https://www.youtube.com/watch?v=CwdbsvcpOHM  
   
What is included in the urdf: colours, material and links (think of Denavit-Hartenberg Parameters)  

## Xacro  
![image](https://user-images.githubusercontent.com/85168871/166198932-bc03f6e3-8ef1-4c24-b512-9290727cc20b.png)  
  
### sensors.xacro
  <!-- Mount DVL -->
  <!--fov="1.22173"-->
  <!--width="512"-->
  <!--height="400"-->
  
	  <xacro:forward_looking_sonar
	      namespace="${namespace}"
	      suffix=""
	      parent_link="${namespace}/base_link"
	      topic="fls_sonar"
	      mass="0.00001"
	      update_rate="5"
	      samples="512"
	      fov="0.8726"
	      width="512"
	      height="115" >
	      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001" />
	      <origin xyz="0 0 -0.4" rpy="0 0.261799 0" />
	  </xacro:forward_looking_sonar>

### Splugins sonar_snippet.xacro

	  <xacro:macro name="forward_looking_sonar" params="namespace suffix parent_link topic mass update_rate samples fov width height *inertia *origin">
	    <!-- Sensor link -->
	    <link name="${namespace}/forward_sonar${suffix}_link">
	      <inertial>
		<xacro:insert_block name="inertia" />
		<mass value="${mass}" />
		<origin xyz="0 0 0" rpy="0 0 0" />
	      </inertial>
	      <visual>
		<geometry>
		  <mesh filename="file://$(find uuv_sensor_ros_plugins)/meshes/oe14-372.dae" scale="1 1 1"/>
		</geometry>
	      </visual>
	      <xacro:no_collision/>
	    </link>

	    <joint name="${namespace}_forward_sonar${suffix}_joint" type="revolute">
	      <xacro:insert_block name="origin" />
	      <parent link="${parent_link}" />
	      <child link="${namespace}/forward_sonar${suffix}_link" />
	      <limit upper="0" lower="0" effort="0" velocity="0" />
	      <axis xyz="1 0 0"/>
	    </joint>


	    <gazebo reference="${namespace}/forward_sonar${suffix}_link">
	      <sensor name="${namespace}/image_sonar" type="depth">
		<camera>
			  <horizontal_fov>${fov}</horizontal_fov>
		  <image>
			    <width>${width}</width>
			    <height>${height}</height>
		    <format>R8G8B8</format>
		  </image>
		  <clip>
		    <near>0.1</near>
		    <far>17</far>
		  </clip>
		  <save enabled="true">
		    <path>/tmp/camera</path>
		  </save>
		</camera>
		    <plugin filename="libimage_sonar_ros_plugin.so" name="forward_sonar${suffix}_controller">
			  <topicName>${topic}</topicName>
		  <frameName>forward_sonar${suffix}_optical_frame</frameName>
		</plugin>
		<always_on>true</always_on>
		    <update_rate>${update_rate}</update_rate>
	      </sensor>
	    </gazebo>

		<joint name="${namespace}/forward_sonar${suffix}_joint" type="fixed">
	      <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
	      <parent link="${namespace}/forward_sonar${suffix}_link"/>
	      <child link="${namespace}/forward_sonar${suffix}_optical_frame"/>
	    </joint>

		<link name="${namespace}/forward_sonar${suffix}_optical_frame"/>
	  </xacro:macro>
  
### Current Error
	[ERROR]: Mesh scale was specified, but could not be parsed: Parser found 0 elements but 3 expected while parsing vector [  ]
	[ERROR]: Could not parse visual element for Link [bluerov2/dvl_link]
	[ERROR]: Failed to build tree: parent link [bluerov2/baselink] of joint [bluerov2/dvl_joint] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [bluerov2/dvl_joint] from your urdf file, or add "<link name="bluerov2/baselink" />" to your urdf file.

### DVL Xacro Macros

NOT WORKING!!! (GOOD IF WE WANT TO SCALE THE DVL -> BUT WHAT IS SCALED? THE MESH?)

	  <xacro:dvl_plugin_macro 
	      namespace="${namespace}"
	      parent_link="${namespace}/base_link" 
	      topic="dvl_sonar"
	      scale=""
	      update_rate="5"
	      reference_frame=""
	      noise_sigma=""
	      noise_amplitude="" >
	      <origin xyz="0 0 0" rpy="0 0 0" />
	  </xacro:dvl_plugin_macro>
  
  WORKING!!! (ALSO DEPRECATED)
  
	  <xacro:default_dvl namespace="${namespace}" parent_link="${namespace}/base_link">
	      <origin xyz="0 0 0" rpy="0 0 0" />
	  </xacro:default_dvl >
    
# Further Notes

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

# Integration Progress Snapshots
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


# Rostopic List (18/05/2022)
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

# Rostopic echo /bluerov2/dvl  
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



### Next
- have a look at scaling (does it scale the visual model?) and adding parameters
- what does deprecated really mean?
- Terminal output of dvl data from simulation

# missing topic from rostopic list command
![image](https://user-images.githubusercontent.com/88146518/170007969-52e3881d-c20f-423d-80e8-ce9a763ce72b.png)
![image](https://user-images.githubusercontent.com/88146518/170007989-89a5d798-b8ef-4f90-b933-631abb9933d0.png)
![image](https://user-images.githubusercontent.com/88146518/170008010-e55f8d1a-2cfb-48f8-b728-bd5b77ab8ad9.png)
