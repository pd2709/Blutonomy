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
