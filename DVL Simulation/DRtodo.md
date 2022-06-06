# TO DO 6/06
  - Message published
    - Make sure it matches the DVL DR protocol in waterlinked.
    - Double check the variables in msg file.
      - Change the quarternion to rotation matrix
      - rotation matrix -> roll, pitch, yaw
      - need to get x,y,z distance by determining timestamp inbetween DVL msgs. d=v*t
      - stretch: std measure (not sure how to get it)

 ## DR Questions for Shoudong & Yingyu
    1) How should we use roll, pitch, yaw of the rover? (should we make a homogenous matrix?)
    2) When is it appropriate to use gtsam for variables?
    3)

## Extended Kalman Filter Research
https://mdpi-res.com/d_attachment/sensors/sensors-11-09182/article_deploy/sensors-11-09182.pdf?version=1403316184
