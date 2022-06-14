# Blutonomy

This directory contains the current code for SLAM implementation of BlueROV2.

The launch file we have been using is **simulation.launch**. This launch file then launches **extract_uuvsim.launch** and **graph_uuvsim.launch**.   
  
To launch the simulation we use the cmd ‘roslaunch slam simulation.launch’.  

![image](https://user-images.githubusercontent.com/85168871/172343329-bf2ef2b0-d5c0-496b-bd72-81c9dad4773f.png)  


## SRC FILES from intern who worked on SLAM
Note that the file descriptions can be inaccurate as we have not written the code.

  **associate_jcbb.cpp**, **associate_nearest.cpp** - attempt at data association of landmarks.

  **extract.cpp**, **extract_oculus_node.cpp**, **extract_uuvsim_node.cpp**(This one is used in siumulation) - Uses AKAZE to filter out keypoints for the aim of extracting landmark information from Multibeam Sonar.

  **graph.cpp** - Is a graph class.

  **graph_node.cpp** - uses GTSAM and runs the SLAM using IMU data.

  **slam_geometry.cpp** - geometery calculations to conduct SLAM.

## SRC FILES from UTS Blutonomy group AUT2022

**deadReckonNode.cpp** - This is our newly created file aimed to perform Dead Reckoning using both DVL and IMU Data.
