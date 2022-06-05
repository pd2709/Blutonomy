 `cd ~/matt_thesis/catkin_ws`
 `source devel_isolated/setup.bash`
 
 
 
 # rostopic echo /slam/result

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
# rostopic echo /slam/features
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
# Rostopic echo mavros/imu/data
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
# Research on plugins
  
  Trying to make the uuv simulator plugin useable  
  - This can be checked with the cmd `rospack plugins uuv_sensor_ros_plugins_msgs` 
  - ![image](https://user-images.githubusercontent.com/85168871/172045778-2d206132-8a7d-42b7-bd14-8d311f7569eb.png)  
 
 # Research code structure  
 How to subscribe to multiple topics and publish to another topic  
 - https://robotics.stackexchange.com/questions/20089/not-trivial-ros-how-to-subscribe-to-multiple-topics-and-publish-to-another-top 
  
# More research
## AHRS
https://gtsam.org/doxygen/4.0.0/a01268_source.html


