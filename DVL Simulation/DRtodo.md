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


        ---
        header: 
          seq: 1426
          stamp: 
            secs: 71
            nsecs: 352000000
          frame_id: "world"
        pose: 
          position: 
            x: 1.40396725092
            y: -0.224058904297
            z: -0.109012311163
          orientation: 
            x: 1.54590558896e-06
            y: -0.0149156005825
            z: -7.32005914418e-05
            w: 0.999888753561
        ---

        header: 
          seq: 1192
          stamp: 
            secs: 59
            nsecs: 652000000
          frame_id: "world"
        vector: 
          x: -7.23739779312e-08
          y: -0.0123068317767
          z: 0.0
