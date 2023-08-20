# MR Teleoperation (User Study version)

MyCobot 320 Raspberry Old version

[Project homepage](https://robodd.github.io/site/research/)

# On the MyCobot 320 Raspberry Pi (Ubuntu 18.04 + ROS Melodic + pymycobot)

- Install ROS

- Setup `mycobot_ros` on the robot, ensure everything works fine. [mycobot_ros GitHub Repo](https://github.com/elephantrobotics/mycobot_ros.git)

- Setup ROS-TCP-Endpoint. [ROS-TCP-Endpoint GitHub Repo](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

- Setup Realsense Driver and ROS

- Configure ROS Master IP (if needed)

# On the Unity (Windows)

- Configure this PC as Server, and connect robot and data recorder PC to server, setup ROS master and slave IP of each computer
- Power on Quest Pro, connect quest link cable, and enter quest link world
- Open teleoperation software by double click
- Modify ROS IP accordingly and then connect


# On the Data Record Node (Ubuntu 20.04 + ROS Noetic)

- Configure ROS Master IP (if needed)
- run data recorder in Python

record_newest.py is changed to record_mycobot320.py

```bash
(base) newuser@rex-Legion-5-15ACH6H:~/Desktop$ python3 record_mycobot320.py --participant ShanZhang --experiments VR --data-path ~/Desktop/ziniu_data
```

# Useful Commandline

```
source ~/unity_tcp_ws/devel/setup.bash

roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=192.168.137.10 tcp_port:=10000
```


**this ip may change, check and restart pi every time**

```
roslaunch realsense2_camera rs_camera.launch

roslaunch mycobot_communication communication_topic.launch

rosrun new_mycobot_320_pi mycobot_320_unity.py _port:=/dev/ttyAMA0 _baud:=115200
```

```
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=10.42.0.1 tcp_port:=10000

rosrun new_mycobot_320_pi mycobot_320_slider.py _port:=/dev/ttyAMA0 _baud:=115200

roslaunch new_mycobot_320_pi mycobot_320_teleop_keyboard.launch port:=/dev/ttyAMA0 baud:=115200
```



# 2023-06-23 Network Latency test

--- 192.168.137.56 ping statistics ---
30 packets transmitted, 30 received, 0% packet loss, time 29690ms
rtt min/avg/max/mdev = 0.094/0.168/0.213/0.044 ms
