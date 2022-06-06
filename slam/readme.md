# Blutonomy

This directory contains the current code for SLAM implementation of BlueROV2.

The launch file we have been using is **simulation.launch**.

Following files are inside the source folder:

## FILES from intern who worked on SLAM
Note that the file descriptions can be inaccurate as we have not written the code.

  **associate_jcbb.cpp**, **associate_nearest.cpp** - attempt at data association of landmarks.

  **extract.cpp**, **extract_oculus_node.cpp** - Uses AKAZE to filter out keypoints for the aim of extracting landmark information from Multibeam Sonar.

  **extract_uuvsim_node.cpp**

  **graph.cpp**

  **graph_node.cpp**

  **slam_geometry.cpp**

## FILES from UTS Blutonomy group AUT2022


**deadReckonNode.cpp** - This is our newly created file aimed to perform Dead Reckoning using both DVL and IMU Data.
