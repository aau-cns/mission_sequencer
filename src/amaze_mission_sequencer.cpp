// Copyright (C) 2020 Control of Networked Systems, Universit?t Klagenfurt, Austria
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

///
/// \brief PX4 offboard waypoint service nide
/// Tested in Gazebo SITL and real-world
///

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>  //velocity control
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>

#include "parse_waypoints.hpp"

// Vehicle state subscriber
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

// position Subscriber
geometry_msgs::PoseStamped current_pose;
bool pose_ok(false);
void current_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
  pose_ok = true;
}

// landing state
static mavros_msgs::ExtendedState current_ext_state_;
static bool ext_state_msg_ok_ = false;
void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  current_ext_state_ = *msg;
  ext_state_msg_ok_ = true;
}

// tolerance of achieving waypoint = 20cm
static double threshold_wp_reached_ = 0.1;

// degrees to radians transformation constant
static double deg_rad_ = M_PI / 180.0;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;  // class: ros::NodeHandle, object: nh

  // create publishers and subscribers
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber current_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, current_cb);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::Publisher local_pos_vel = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Subscriber ext_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, ext_state_cb);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // Read parameters from launchfile
  std::string filename;
  int mode;
  std::vector<std::string> header;
  std::vector<std::string> header_default = {"x", "y", "z", "yaw", "holdtime"};

  if(!nh.getParam("filename", filename)) {
    std::cout << std::endl;
    ROS_ERROR("Waypoint filename missing!");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("mode", mode)) {
    std::cout << std::endl;
    ROS_ERROR("Mission mode missing!");
    std::exit(EXIT_FAILURE);
  }
  ROS_INFO("Mode: %d", mode);

  if(!nh.getParam("wp_threshold", threshold_wp_reached_)) {
    std::cout << std::endl;
    ROS_WARN("No Waypoint threshold defined, Taking 0.1 by default");
  }
  ROS_INFO("Selected waypoint threshold: %f", threshold_wp_reached_);


  nh.param<std::vector<std::string>>("header", header, header_default);

  //  // DEBUG:
  //  if (!header.empty()) {
  //    for (const auto &it : header) {
  //      std::cout << "[DEBUG] Print header: " << it << std::endl;
  //    }
  //  } else {
  //    std::cout << "[DEBUG] Error reading header param";
  //  }

  // Define waypoint parser
  std::shared_ptr<ParseWaypoint> WaypointParser =  std::make_shared<ParseWaypoint>(filename, header);

  // Parse waypoint file
  WaypointParser->readParseCsv();

  // Get the data
  std::vector<ParseWaypoint::Waypoint> waypoints;
  waypoints = WaypointParser->getData();

//  // DEBUG
//  for (const auto &it : waypoints) {
//    std::cout << "[DEBUG] Print waypoint: " << it.x << ", " << it.y << ", " << it.z << ", " << it.yaw << ", " << it.holdtime << std::endl;
//  }

  // Wait for FCU connection
  std::cout << std::endl << "connecting to FCT..." << std::endl << std::endl;
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // Wait for a valid pose
  std::cout << "Wait for valid init pose..." << std::endl << std::endl;
  while (pose_ok == false)
  {
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Got a valid init pose" << std::endl;
  std::cout << "Init pose p(x,y,z): " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << std::endl;
  std::cout << "Init pose q(x,y,z,w): " << current_pose.pose.orientation.x << ", " << current_pose.pose.orientation.y << ", " << current_pose.pose.orientation.z << ", " << current_pose.pose.orientation.w << std::endl << std::endl;

  // Initial pose adn quaternion
  geometry_msgs::PoseStamped init_pose(current_pose);
  tf2::Quaternion init_quat;

  // DEBUG
  // ROS_INFO_STREAM("Got init pose [x,y,z]: " << init_pose.pose.position.x << " " << init_pose.pose.position.x << " " << init_pose.pose.position.x);

  // Pose and quaternion
  geometry_msgs::PoseStamped pose;
  tf2::Quaternion quat;

  // Set pose depending on mode
  switch(mode) {

    case 1 :
      pose.pose.position.x = waypoints.at(0).x;
      pose.pose.position.y = waypoints.at(0).y;
      pose.pose.position.z = waypoints.at(0).z;
      break;

    case 2 :
      pose.pose.position.x = waypoints.at(0).x + init_pose.pose.position.x;
      pose.pose.position.y = waypoints.at(0).y + init_pose.pose.position.y;
      pose.pose.position.z = waypoints.at(0).z + init_pose.pose.position.z;
      init_quat[0] = init_pose.pose.orientation.x;
      init_quat[1] = init_pose.pose.orientation.y;
      init_quat[2] = init_pose.pose.orientation.z;
      init_quat[3] = init_pose.pose.orientation.w;
      init_quat.normalize();
      break;
  }

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
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

  //Arm request time and Offboard request time time
  ros::Time Arm_request_time = ros::Time::now();
  ros::Time Offboard_request_time = ros::Time::now();

  // change to offboard mode and arm
  while (ros::ok() && !current_state.armed)
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - Offboard_request_time > ros::Duration(2.5)))
    {
      std::cout << "Actual mode: " << current_state.mode.c_str() << std::endl;
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        std::cout << "Offboard enabled" << std::endl;
      }
      Offboard_request_time = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - Arm_request_time > ros::Duration(2.5)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          std::cout << "Vehicle armed" << std::endl << std::endl;
        }
        Arm_request_time = ros::Time::now();
      }
    }
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  // Check everything is ok before setting waypoints
  std::cout << "Vehicle armed and offboard mode enabled... Press any key to continue within 10 seconds" << std::endl;
  std::cin.ignore();

  // Waypoint counter
  int waypoint_cnt = 1;

  // Loop through waypoints
  for (const auto &it : waypoints) {

    // Bolean to check if waypoint is reached
    bool reached = false;

    // Print infos
    std::cout << "Setting waypoint [x: " << it.x << ", y: " << it.y << ", z: " << it.z << ", yaw: " << it.yaw << "]" << std::endl;
    std::cout << "Setting holdtime: " << it.holdtime << " s" << std::endl << std::endl;

    // Convert yaw to radians
    double yaw_rad = it.yaw * deg_rad_;
    quat.setRotation(tf2::Vector3(0, 0, 1), yaw_rad);
    quat.normalize();

    // Set pose depending on mode
    switch(mode) {

      case 1 :
        pose.pose.position.x = it.x;
        pose.pose.position.y = it.y;
        pose.pose.position.z = it.z;
        break;

      case 2 :
        pose.pose.position.x = it.x + init_pose.pose.position.x;
        pose.pose.position.y = it.y + init_pose.pose.position.y;
        pose.pose.position.z = it.z + init_pose.pose.position.z;
        quat = init_quat * quat;
        break;
    }

    pose.pose.orientation.x = quat[0];
    pose.pose.orientation.y = quat[1];
    pose.pose.orientation.z = quat[2];
    pose.pose.orientation.w = quat[3];

    // Time when the waypoint got reached
    ros::Time Waypoint_reached_time = ros::Time::now();

    // Fly to the waypoint and holds it for the required holdtime
    while (ros::ok())
    {

      // Publish
      local_pos_pub.publish(pose);

      // Check thresholds
      if (abs(pose.pose.position.x - current_pose.pose.position.x) < threshold_wp_reached_ &&
          abs(pose.pose.position.y - current_pose.pose.position.y) < threshold_wp_reached_ &&
          abs(pose.pose.position.z - current_pose.pose.position.z) < threshold_wp_reached_) {

        // Set the time when the waypoint got reached and the reached flag
        if (!reached) {

          // print info
          std::cout << "Reached waypoint [x: " << it.x << ", y: " << it.y << ", z: " << it.z << ", yaw: " << it.yaw << "]" << std::endl;
          std::cout << "Hold waypoint for: " << it.holdtime << " s" << std::endl;

          // Set time and flag
          Waypoint_reached_time = ros::Time::now();
          reached = true;
        }

        // Break the loop after holding the waypoint
        if ((ros::Time::now() - Waypoint_reached_time) > ros::Duration(it.holdtime)) {
          std::cout << "Waypoint Helded for: " << it.holdtime << " s" << std::endl << std::endl;
          break;
        }
      }

      ros::spinOnce();
      rate.sleep();
    }

    // Print waypoints still to fly
    std::cout << "Number of waypoint that have still to be reached: " << waypoints.size() - waypoint_cnt << std::endl << std::endl;

    // Increase waypoint counter
    ++waypoint_cnt;
  }

  // land
  while (ros::ok())
  {
    if (land_client.call(land_cmd))
    {
      // only continue if the vehicle landed
      if (land_cmd.response.success)
      {
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok() && !(current_ext_state_.landed_state == current_ext_state_.LANDED_STATE_ON_GROUND))
  {
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "Vehicle landed" << std::endl;

  arm_cmd.request.value = false;

  std::cout << "Disarming..." << std::endl;

  while (ros::ok())
  {
    if (arming_client.call(arm_cmd))
    {
      if (arm_cmd.response.success)
      {
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Disarmed - Engines got turned off" << std::endl;

  return 0;
}
