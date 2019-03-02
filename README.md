# Robot of Things (RoT)
## Description
Code and documentation for my personal experimental robot.

The robot is based on a remote control car base and mostely uses parts I already had (things).

![Robot of Things (RoT)](/documentation/RoT.png)

**Note:**
This is not a complete solution. You will also need at least the following packages and many features either won't work or have issues:

Package Name | Github source | Reference
------------ | ------------- | ---------
AWS RoboMaker TTS | https://github.com/aws-robotics/tts-ros1 | AWS RoboMaker (https://aws.amazon.com/robomaker/)
Ackermann message | https://github.com/ros-drivers/ackermann_msgs | ROS Ackermann Group (http://wiki.ros.org/Ackermann%20Group)

## Overview
This robot is a personal projct to test robotics, deap learning and vision processing systems. I've shared the code here to help others in their learning of robotics and so others can give me feedback.

The robot is based on ROS (http://ros.org) the Robot Operating System. It uses many other people code either as packages or as modified source. I strive to credit the original authors where possible but sometimes miss one. If you notice any code not attributed to it's authore please let me know.

While making this robot I have used many online sites fo information. In no particuular order I have extensively used the following:

**ROS** - http://www.ros.org/
**pyimagesearch** - https://www.pyimagesearch.com
**O'Reilly Safafi Books Online (subscription)** - https://my.safaribooksonline.com/
**OpenCV** - https://opencv.org/
**Robot Accademy** - https://robotacademy.net.au/
**The Construct** - http://www.theconstructsim.com/
**PJRC.com (Teensy)** - https://pjrc.com
**Adafruit** - https://www.adafruit.com/
**Sparkfun Electronics** - https://www.sparkfun.com/

## Controller and Sensors Overview
A short cli of RoT running:
https://youtu.be/b_HMtGfrs7E
![Controller Overview](/documentation/RoT-overview.png)

### Messages and Transforms
#### Frames
![Transform Tree](/documentation/frames.png)

#### Robot Model (URDF)
![URDF Tree](/documentation/urdf.png)

### Rviz
![Rviz with the Teleop terminal](/documentation/RoT-rviz.png)
A short clip of RoT running in the Rviz application on Youtube.
https://youtu.be/lAinIkkNIuQ

## TODO
- [ ] Complete the readme (this document)
- [x] Check the controller transformations
- [x] Fix the URDF mapping to RoT messages
- [ ] Implement LIDAR Scanning on the Teensy controller
- [ ] Implement the depth and stereo image messages fro the Raspberry Pi stereo comeras
- [ ] Implement the ackermann steering planner (teb_local_planner)
- [ ] Integrate with AWS RoboMaker
- [ ] Integrate with Alexa
