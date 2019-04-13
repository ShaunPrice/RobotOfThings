# Robot of Things (RoT)
## Description
Code and documentation for my personal experimental robot.

The robot is based on a remote control car base and mostely uses parts I already had (things).

![Robot of Things (RoT)](/documentation/RoT.jpg)

**Note:**
This is not a complete solution. You will also need at least the following packages and many features either won't work or have issues:

Package Name | Github source | Reference
------------ | ------------- | ---------
AWS RoboMaker TTS | https://github.com/aws-robotics/tts-ros1 | AWS RoboMaker (https://aws.amazon.com/robomaker/)
Ackermann message | https://github.com/ros-drivers/ackermann_msgs | ROS Ackermann Group (http://wiki.ros.org/Ackermann%20Group)

## Overview
This robot is a personal projct to test robotics, deap learning and vision processing systems. I've shared the code here to help others in their learning of robotics and so others can give me feedback.

The robot is based on ROS (http://ros.org) the Robot Operating System. It uses many other people code either as packages or as modified source. I strive to credit the original authors where possible but sometimes miss one. If you notice any code not attributed to it's authore please let me know.

While making this robot I have used many online sites for information. In no particuular order I have extensively used the following:

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
PDF: [RoT-overview.pdf](/documentation/pdf/RoT-overview.pdf)

### Messages and Transforms

#### ROS Message Graph View
![ROS ](/documentation/rosgraph.png)
ROS Command: rqt_graph

#### Frames
![Frames Tree](/documentation/frames.png)
PDF: [frames.pdf](/documentation/pdf/frames.pdf)

ROS Command: view_frames

#### Transform Tree
![Transform Tree](/documentation/tf_tree.png)
ROS Command: rqt_tf_tree

#### Robot Model (URDF)
![URDF Tree PDF](/documentation/urdf.png)
ODF: [urdf.pdf](/documentation/pdf/urdf.pdf)

ROS Command: urdf_to_graphiz ~/catkin_ws/src/rot/urdf/rot.urdf

### Rviz
![Rviz with the Teleop terminal](/documentation/RoT-rviz.png)
A short clip of RoT running in the Rviz application on Youtube.
https://youtu.be/lAinIkkNIuQ
Another with the stereo images and disparity (depth) map image.
https://youtu.be/ZktRqChj550

### RoTVision
The RoTVision board processes the stereo images from the cameras. It consists of a Raspberry Pi Compute Module CM3b+ (16Gb) with stereo cameras. I used a USB hub with an ethernet connection to connect it to the Odroid XU4.  

The board is loaded with Raspbian Stretch and I've compiled ROS Melodic (headless) and OpenCV 4.0.1. ROS includes the sensor_msgs and cv_bridge modules to communicate with ROS.

ROS on the board supplies the components required and it connects back to eth master. My .bashrc scripts contain the following to allow this:

    export ROS_HOST=rot
    export ROS_MASTER_URI=http://rot:11311

The IP address for the main controller (rot) is in the hosts file.

You'll also need to source the ROS setup script

source /opt/ros/melodic/setup.bash

### Connecting the network between the boards (RoT and RoTVision)

The two boards need to talk to each other and the RoT Vision board needs to talk to the internet for updates etc through the RoT board or if its connected to my home network automatically switch to that for internet access.

I've implemented the robot network by having the RoTVision board default to the home network using DHCP. If it can't find that it drops back to a static IP that's connected to the RoT controller board (the normal mode of operation).

#### The Router ####
The RoT controller (Odroid XU4) is the network router for the robot. I've implemented the ethernet connections as follows:
    
    Eth0 is the external network gateway (USB Ethernet connection)
    Eth1 is the internal network (Onboard Network onnection)


Add the host names to the Odroids host file:

    sudo nano /etc/hosts

Add:

    127.0.1.1	rot
    10.1.0.2	rotvision

Configure forwarding for RoT Vision (rotvision):

**Note:**
The following is adapted from https://askubuntu.com/questions/1050816/ubuntu-18-04-as-a-router

Enable ufw and ufw logging:

    sudo ufw enable
    sudo ufw logging on

Flush any existing rules (do NOT do this if you are already using ufw or IP tables for firewalling). Delete and flush. Default table is "filter". Others like "nat" must be explicitly stated.

    iptables --flush            # Flush all the rules in filter and nat tables    
    iptables --table nat --flush    
    iptables --delete-chain     # Delete all chains that are not in default filter and nat table    
    iptables --table nat --delete-chain    

Packet forwarding needs to be enabled in ufw. Two configuration files will need to be adjusted, in **/etc/default/ufw** change the DEFAULT_FORWARD_POLICY to “ACCEPT”:

    DEFAULT_FORWARD_POLICY="ACCEPT"

Edit **/etc/ufw/sysctl.conf** and uncomment:

    net/ipv4/ip_forward=1
    net/ipv4/conf/all/forwarding=1 
    net/ipv6/conf/default/forwarding=1 # if using IPv6

Add rules to the **/etc/ufw/before.rules** file. The default rules only configure the filter table, and to enable masquerading the nat table will need to be configured.

Add the following to the top of the file just after the header comments:

    # nat Table rules
    *nat
    :POSTROUTING ACCEPT [0:0]
    # Forward traffic from eth1 through eth0.
    -A POSTROUTING -s 10.1.0.0/24 -o eth0 -j MASQUERADE
    # don't delete the 'COMMIT' line or these nat table rules won't be processed
    COMMIT

For each Table a corresponding COMMIT statement is required. In these examples only the nat and filter tables are shown, but you can also add rules for the raw and mangle tables.

Add the following forwarding rules to the iptables:

    sudo iptables -A FORWARD -i eth0 -o eth1 -m state --state RELATED,ESTABLISHED -j ACCEPT
    sudo iptables -A FORWARD -i eth1 -o eth0 -j ACCEPT
    
To store the iptable rules persistently install the following package:
	
    sudo apt-get install iptables-persistent

The setup will ask you to save the Version 4 and 6 IP rules in the following default files which will be used later:

    /etc/iptables/rules.v4
    /etc/iptables/rules.v6

You can save rules added later with:

    sudo sh -c "iptables-save > /etc/iptables/rules.v4"
    sudo sh -c "iptables-save > /etc/iptables/rules.v6"
    
Add the following to the end of the /etc/network/interfaces file to restore the rules at boot:

    pre-up iptables-restore < /etc/iptables/rules.v4
    pre-up iptables-restore < /etc/iptables/rules.v6
    
Don't forget to add any other firewall rules you require. I use ssh and xrdp so add the following:

    sudo ufw allow 22/tcp
    sudo ufw allow 3389/tcp

Disable and re-enable ufw to apply the changes:

    sudo ufw disable && sudo ufw enable

IP Masquerading should now be enabled. You can also add any additional FORWARD rules to the /etc/ufw/before.rules. It is recommended that these additional rules be added to the ufw-before-forward chain.

####RoTVision IP Configuration####
On the RoTVision board (Raspberry Pi Compute) with Raspbian Stretch installed Eth0 is the network interface to route through and we'll make it's IP Address 10.1.0.2. The Odroid is 10.1.0.1.

Open the configuration file:
    
    sudo nano /etc/dhcpcd.conf

Add the following to allow the device to use DHCP or fallback to a static IP.

	# It is possible to fall back to a static IP if DHCP fails:
	# define static profile
	profile static_eth0
	static ip_address=10.1.0.2/24
	static routers=10.1.0.1
	static domain_name_servers=10.1.0.1
	
	# fallback to static profile on eth0
	interface eth0
	fallback static_eth0
	
Add the host names to the Odroids host file:

    sudo nano /etc/hosts

Add:

    127.0.1.1	rotvision
    10.1.0.1	rot

**Note**
You may need to manually add a static default gateway route.

    sudo route add default gw 10.1.0.1

####RoTVision Stereo Camera Configuration####
The stereo cameras on the Raspberry Pi Compute Module has a special configuration to get the camera to work together detailed on the Raspberry Pi site:

    https://www.raspberrypi.org/documentation/hardware/computemodule/cmio-camera.md

## Current Issues
1. The point cloud implementation works but is not scaled to real world units.

## TODO
- [ ] Complete the readme (this document)
- [x] Check the controller transformations
- [x] Fix the URDF mapping to RoT messages
- [x] Implement LIDAR Scanning on the Teensy controller
- [x] Implement the depth and stereo image messages fro the Raspberry Pi stereo comeras
- [ ] Integrate with Alexa voice services
