# Commands for Simmulation

## Recording simulated data:
On the blutonomy Laptop:

    cd ~/matt_thesis/catkin_ws
    source devel_isolated/setup.bash
    
*note - source setup.bash in every new terminal opened for roslaunch

1) Start simulation world:
    
       roslaunch matt_thesis_world flat_world.launch
       
2) Connect an XBox360 joystick to control the vehicle
3) Launch BlueROV2 model:
      
       roslaunch bluerov2_gazebo start_pid_controller_demo.launch teleop_on:="true"
       
4) Record data using rosbag:
      
       rosbag record -a -x "(.*)/compressed(.*)"
       
## Playing Back Simulated data
5) Start SLAM

        cd ~/matt_thesis/catkin_ws
        source devel_isolated/setup.bash
        roslaunch slam simulation.launch
        
   *note - the above roslaunch slam will start an RViz visualisation
6) Playback recorded data

        cd {data_folder}
        rosbag play {file-name}.bag -r 0.1
        
# Commands for experimental data
Navigate to `MattD_SLAM_code/` folder to see commands.
        
