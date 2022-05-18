# DVL Integration

https://github-wiki-see.page/m/Field-Robotics-Lab/dave/wiki/whn_dvl_examples
https://answers.gazebosim.org//question/13966/examples-of-registering-a-custom-sensor-not-plugin/#17653
https://answers.gazebosim.org//question/20324/unable-to-create-sensor-of-type/

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
  
  
