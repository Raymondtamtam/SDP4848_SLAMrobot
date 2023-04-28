# SDP4848_SLAMrobot

Hardware on robot: Raspberry Pi 4 

Environment using : Ubuntu 20.04 
ROS version : Noetic



Command to start the robot: 
roslaunch mycar_start start.launch 


1. Keyboard control SLAM
roslaunch mycar_start start.launch 
roslaunch nav gmapping.launch 
- new terminal:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py



a. save the map
roslaunch nav map_save.launch 
b. load the map
roslaunch nav map_server.launch



2. Navigation SLAM with map
roslaunch nav gmapping.launch
roslaunch nav map_save.launch 
roslaunch nav map_server.launch
roslaunch nav test_amcl.launch 



3. Navigation SLAM with no map
roslaunch mycar_start start.launch 
roslaunch nav auto_slam.launch 




