## PLEASE ALSO HAVE THE BLUEROV2 DIVE CHECKLIST FOR OCULUS AND PING360 TO FOLLOW CORRECTLY:

# Connect with project laptop

 
        ssh pi@192.168.2.2 (pwd: companion)
        nohup ./bridge_script.sh
        ./remove_eth0_route.sh
//

        ssh thales@192.168.2.4 (pwd: Blutonomy@789)
        Python leakSensorMavlink.py
  
 # PING360 - Windows
        
        sudo reboot
        nohup ./bridge_script.sh
        ./remove_eth0_route.sh
//

        cd bluerobotics/ping-python/tools
        pingproxy.py --device /dev/ttyUSB1       
//
        
        cd ~/bluerobotics
        ./ping360.sh
        
 # OCULUS - Windows
    
        ssh thales@192.168.2.4 (pwd: Blutonomy@789)
        nohup ./bridge_script.sh
        ./remove_eth0_route.sh
        
 # OCULUS - Ubuntu
      
        ssh thales@192.168.2.4 (pwd: Blutonomy@789)
        nohup ./bridge_script.sh
        ./remove_eth0_route.sh        
// 

        python ~/.local/bin/mavproxy.py --master=udp:192.168.2.1:14550 --out=udp:192.168.2.1:14560 --out=udp:192.168.2.1:14570 --streamrate=-1
//

        ./QgroundControl.AppImage
//

        roslaunch mavros apm.launch fcu_url:=udp://192.168.2.1:14560@
//

        rosservice call /mavros/set_stream_rate 0 10 1  (All streams @ 10 Hz default)
        rosservice call /mavros/set_stream_rate 1 10 1  (Raw sensor to 10 Hz (imu/data_raw))
        rosservice call /mavros/set_stream_rate 10 10 1 (Position to 10 Hz (imu/data))
//

        /home/thales/matt_thesis/catkin_ws/src/slam_thesis/oculus_node/include/oculusInterface.hpp 
//

        cd ~/matt_thesis/catkin_ws
        catkin_make_isolated
        source devel_isolated/setup.bash
//

        cd /media/thales/<name of data logging drive>
//

        cd ~/matt_thesis/catkin_ws
        source devel_isolated/setup.bash
        rqt
//

        /mavros/imu/data
        /mavros/imu/data_raw 
//

        rosrun oculus_node oculus_node
//

        rosbag record -a
//
        
        rosbag record -a --duration=30 -O tc1_1.bag

# PING360 - UBUNTU    
Same as Section 3.2.1 – using python script to save CSV


# Poset-dive Data Check

        rosbag play <bag name>
//

        /mavros/imu/data 
        /mavros/imu/data_raw 
        /oculus/header 
        /oculus/polarscan 
        
