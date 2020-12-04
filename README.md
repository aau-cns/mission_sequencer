
# Instructions to run waypoints code 1.1  
Through this code the drone is moving through set points clarified below:  
x=0, y=0, z=2  
x=0, y=1, z=2    
Then doing RTL and landing.  

### Steps for launching the node:  
1- user@user_name:~$ roscore  
2- user@user_name:~$ source /opt/ros/melodic/setup.bash  
3- user@user_name:~$ roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"  
4- user@user_name:~/catkin_ws2$ catkin build amaze_waypoint_following  
5- user@user_name:~/catkin_ws2$ source devel/setup.bash  
6- user@user_name:~/catkin_ws2$ rosrun amaze_waypoint_following amaze_waypoint_following  

Now node is already started to run.  
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



## References

Arming and switching to offboard mode: https://dev.px4.io/master/en/ros/mavros_offboard.html