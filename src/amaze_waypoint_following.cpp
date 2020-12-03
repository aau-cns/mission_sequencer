/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h> //velocity control
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

// position Subscriber

geometry_msgs::PoseStamped current_pose;
bool pose_ok(false);
void current_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  current_pose = *msg;
  pose_ok = true;
}

double flight_altitude = 1;
int point_time = 15;

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh; // class: ros::NodeHandle, object: nh

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::Subscriber current_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "mavros/local_position/pose", 10, current_cb);
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client =
      nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("connecting to FCT...");
  }

  while (pose_ok == false) {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("Wait for valid init pose...");
  }

  geometry_msgs::PoseStamped init_pose(current_pose);
  ROS_INFO_STREAM("Got init pose [x,y,z]: " << init_pose.pose.position.x << " "
                                            << init_pose.pose.position.x << " "
                                            << init_pose.pose.position.x);

  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = 0 + init_pose.pose.position.x;
  pose.pose.position.y = 0 + init_pose.pose.position.y;
  pose.pose.position.z = flight_altitude + init_pose.pose.position.z;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = 0;
  land_cmd.request.longitude = 0;
  land_cmd.request.altitude = 0;

  ros::Time last_request = ros::Time::now();

  // change to offboard mode and arm
  while (ros::ok() && !current_state.armed) {
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      ROS_INFO(current_state.mode.c_str());
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  // go to the first waypoint

  pose.pose.position.x = 0 + init_pose.pose.position.x;
  pose.pose.position.y = 0 + init_pose.pose.position.y;
  pose.pose.position.z = flight_altitude + init_pose.pose.position.z;

  ROS_INFO("going to the first way point");
  for (int i = 0; ros::ok() && i < point_time * 20; ++i) {
    local_pos_pub.publish(pose);
    if (abs(pose.pose.position.x - current_pose.pose.position.x) < 0.2 &&
        abs(pose.pose.position.y - current_pose.pose.position.y) < 0.2 &&
        abs(pose.pose.position.z - current_pose.pose.position.z) < 0.2) {
      ROS_INFO("first way point got acheaved!");
    }

    ros::spinOnce();
    rate.sleep();
  }

  // go to the second waypoint

  pose.pose.position.x = 0 + init_pose.pose.position.x;
  pose.pose.position.y = 1 + init_pose.pose.position.y;
  pose.pose.position.z = flight_altitude + init_pose.pose.position.z;

  // send setpoints for point_time seconds
  ROS_INFO("going to second way point");
  for (int i = 0; ros::ok() && i < point_time * 20; ++i) {
    local_pos_pub.publish(pose);
    if (abs(pose.pose.position.x - current_pose.pose.position.x) < 0.2 &&
        abs(pose.pose.position.y - current_pose.pose.position.y) < 0.2 &&
        abs(pose.pose.position.z - current_pose.pose.position.z) < 0.2) {
      ROS_INFO("second way point got acheaved!");
    }
    ros::spinOnce();
    rate.sleep();
  }

  // go back to the first waypoint
  pose.pose.position.x = 0 + init_pose.pose.position.x;
  pose.pose.position.y = 0 + init_pose.pose.position.y;
  pose.pose.position.z = flight_altitude + init_pose.pose.position.z;

  ROS_INFO("going back to the first point!");
  // send setpoints for point_time seconds
  for (int i = 0; ros::ok() && i < point_time * 20; ++i) {
    if (abs(pose.pose.position.x - current_pose.pose.position.x) < 0.2 &&
        abs(pose.pose.position.y - current_pose.pose.position.y) < 0.2 &&
        abs(pose.pose.position.z - current_pose.pose.position.z) < 0.2) {
      ROS_INFO("first way point got acheaved!");
    }
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("tring to land");
  while (!(land_client.call(land_cmd) && land_cmd.response.success)) {
    ROS_INFO("tring to land");
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
