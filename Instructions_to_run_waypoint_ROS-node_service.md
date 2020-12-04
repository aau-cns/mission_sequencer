# Instructions to run waypoint ROS-node service  
The drone can navigate to any point pre-clarified by the client(s), plus the client can specify the mode(Absolute, relative, or landing) and the orientation of the drone.
### Steps for launching the node:  
1- user@user_name:~$ roscore  
2- user@user_name:~$ source /opt/ros/melodic/setup.bash  
3- user@user_name:~$ roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"  
4- user@user_name:~/catkin_ws2$ catkin build amaze_waypoint_following  
5- user@user_name:~/catkin_ws2$ source devel/setup.bash  
6- user@user_name:~/catkin_ws2$ rosrun amaze_waypoint_following waypoint_service
7- user@user_name:~/catkin_ws2$ rosrun amaze_waypoint_following waypoint_service_client 1 1 1 2 90
 
By launching the last command the drone should take mode=1 (navigation based on absolute cordinate), navigate to x=1, y=1, z=2 coordinates, with 90 degrees orientation (around the vertical axis) and waits there till receiving the new command.
mode=1 (navigation based on absolute cordinate)
mode=2 (navigation based on relative cordinate)
mode=0 (land)   
If you want to make a simulation into gazebo use the following command:  
user@user_name:~/px4/Firmware$ make px4_sitl_default gazebo  
  
### Errors might be faced and solutions:  
  
- INFO [dataman] Unknown restart, data manager file './dataman' size is 11798680 bytes  
INFO [simulator] Waiting for simulator to accept connection on TCP port 4560  
**treat it with:**  
sudo apt upgrade libignition-math2  
----------------------------------------  
- FAILED: external/Stamp/sitl_gazebo/sitl_gazebo-build  
**treat it with:**  
make clean  
----------------------------------------  
- RLException: roscore cannot run as another roscore/master is already running. Please kill other roscore/master processes before relaunching. The ROS_MASTER_URI is http://user_name:11311/ The traceback for the exception was written to the log file  
**treat it with:**  
killall -9 roscore  
killall -9 rosmaster  
----------------------------------------  
- If you're in such a situation in which the terminal command is not ending with ctrl + c  
**treat it with:**  
ctrl + z  
----------------------------------------  
- [rospack] Error: package 'amaze_waypoint_following' not found  
**treat it with:**  
User_name:~/catkin_ws2$ source /opt/ros/melodic/setup.bash  
User_name:~/catkin_ws2$ source devel/setup.bash  
----------------------------------------  
- ERROR: cannot launch node of type [mavros/mavros_node]: Cannot locate node of type  
**treat it with:**  
source /opt/ros/melodic/setup.bash  
ls /opt/ros/melodic/lib/mavros // you should see mav components
