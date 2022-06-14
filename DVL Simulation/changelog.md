# Changelog

  
## Edits of existing files
The following section shows the changes in the matt_thesis 'bluerov2' and 'slam' packages.    
### graph_oculus.launch
Address: ~/matt_thesis/catkin_ws/src/slam_thesis/slam/launch/  
- line 7:    COMMENT Currently bluerov2/dvl but can be altered for remapping below  
- line 8:    COMMENT Put DVL on correct topic  
- line 9:    COMMENT remap from="/bluerov2/dvl" to="[INSERT MAVROS DVL TOPIC /mavros/dvl/data]"   
  
### graph_uuvsim.launch  
Address: ~/matt_thesis/catkin_ws/src/slam_thesis/slam/launch/  
- line 7:    COMMENT Currently bluerov2/dvl but can be altered for remapping below  
- line 8:    COMMENT Put DVL on correct topic  
- line 9:    COMMENT remap from="/bluerov2/dvl" to="/bluerov2/dvl"  
  
### CMakeLists.txt
Address: ~/matt_thesis/catkin_ws/src/slam_thesis/slam  

- line 68 "deadReckoning.msg"   
- line 69 "DVL.msg"   
- line 70 "DVL.msg"   
- line 164 "add_executable(deadReckoning src/deadReckonNode.cpp)"   
- line 203 target_link_libraries(deadReckoning
- line 204   ${catkin_LIBRARIES}
- line 205   ${OpenCV_LIBRARIES}
- line 206   ${PCL_LIBRARIES}
- line 207   gtsam
- line 208   gtsam_unstable
- line 209 )
  
## New Files    
The following section shows the changes in the matt_thesis 'bluerov2'   
  
### msg folder   
Address: ~/matt_thesis/catkin_ws/src/slam_thesis/slam  
- deadReckoning.msg   
- DVL.msg   
- DVLBeam.msg   
  
### src folder  
Address: ~/matt_thesis/catkin_ws/src/slam_thesis/slam  
- deadReckonNode.cpp  
