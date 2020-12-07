/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 * cpp
 */

#include <amaze_waypoint_following/wp_service.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>  //velocity control
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static mavros_msgs::State current_state;
static mavros_msgs::ExtendedState current_ext_state;
static geometry_msgs::PoseStamped current_pose;
static geometry_msgs::PoseStamped last_request_waypoint;

// Subscriber
static ros::Subscriber current_sub;
static ros::Subscriber state_sub;
static ros::Subscriber ext_state_sub;
// Publisher
static ros::Publisher local_pos_pub;
static ros::Publisher local_pos_vel;
// Service client handle
static ros::ServiceClient arming_client;
static ros::ServiceClient land_client;
static ros::ServiceClient set_mode_client;
// Waypoint service server
static ros::ServiceServer wp_service;

static bool standby = true;
static bool pose_msg_ok = false;
static bool state_msg_ok = false;
static bool ext_state_msg_ok = false;

// Vehicle state subscriber
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);

// Pose Subscriber
void current_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

// Waypoint service
bool wp_srv_callback(amaze_waypoint_following::wp_service::Request& req,
                     amaze_waypoint_following::wp_service::Response& res);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // Create subscriber
  current_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, current_cb);
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ext_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, ext_state_cb);

  // Create publisher
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  local_pos_vel = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

  // Create service client handle
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // Create waypoint service server
  wp_service = nh.advertiseService("wp_service", wp_srv_callback);

  ROS_INFO("Server is ready!");

  while (ros::ok())
  {
    if (standby == false)
    {
      local_pos_pub.publish(last_request_waypoint);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  state_msg_ok = true;
}

void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  current_ext_state = *msg;
  ext_state_msg_ok = true;
}

void current_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
  pose_msg_ok = true;
}

bool wp_srv_callback(amaze_waypoint_following::wp_service::Request& req,
                     amaze_waypoint_following::wp_service::Response& res)
{
  // Check if critical messages have been received
  if (pose_msg_ok == false)
  {
    ROS_WARN("Did not receive a pose msg from the vehicle yet");
    res.wp_reached = false;
    return false;
  }
  if (state_msg_ok == false)
  {
    ROS_WARN("Did not receive a state msg from the vehicle yet");
    res.wp_reached = false;
    return false;
  }
  if (ext_state_msg_ok == false)
  {
    ROS_WARN("Did not receive an ext_state msg from the vehicle yet");
    res.wp_reached = false;
    return false;
  }

  standby = false;
  ros::Rate rate(20.0);  // the setpoint publishing rate MUST be faster than 2Hz

  int mode = req.mode;

  // Map Position
  last_request_waypoint.pose.position.x = double(req.x);
  last_request_waypoint.pose.position.y = double(req.y);
  last_request_waypoint.pose.position.z = double(req.z);

  // Map Orientation
  double yaw = double(req.yaw);
  double radians = yaw / (180.0 / 3.141592653589793238463);
  tf2::Quaternion q_wp;
  q_wp.setRotation(tf2::Vector3(0, 0, 1), radians);  // 45 degrees rotation
  q_wp.normalize();
  last_request_waypoint.pose.orientation.w = q_wp[3];  // real part of quaternion
  last_request_waypoint.pose.orientation.x = q_wp[0];
  last_request_waypoint.pose.orientation.y = q_wp[1];
  last_request_waypoint.pose.orientation.z = q_wp[2];

  ROS_INFO_STREAM("Request: mode= " << req.mode << " x= " << req.x << " y= " << req.y << " z= " << req.z
                                    << " yaw= " << req.yaw);
  ROS_INFO_STREAM("Current Position [" << current_pose.pose.position.x << " " << current_pose.pose.position.y << " "
                                       << current_pose.pose.position.z << "]");

  // geometry_msgs::TwistStamped vel;
  // geometry_msgs::PoseStamped init_pose(current_pose);

  // Check that we are not in the air and initialize the PX4
  if (current_ext_state.landed_state == current_ext_state.LANDED_STATE_ON_GROUND)
  {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time t_last_request = ros::Time::now();

    // change to offboard mode and arm the vehicle
    while (ros::ok() && !current_state.armed)
    {
      // Switch to offboard mode
      if (current_state.mode != "OFFBOARD" && (ros::Time::now() - t_last_request) > ros::Duration(2.5))
      {
        ROS_INFO(current_state.mode.c_str());
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
        t_last_request = ros::Time::now();
      }
      else
      {
        // Arm the vehicle
        if (!current_state.armed && (ros::Time::now() - t_last_request) > ros::Duration(2.5))
        {
          if (arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("Vehicle armed");
          }
          t_last_request = ros::Time::now();
        }
      }

      // send a few setpoints before starting
      for (int i = 100; ros::ok() && i > 0; --i)
      {
        local_pos_pub.publish(last_request_waypoint);
        ros::spinOnce();
        rate.sleep();
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

  // Perform requested waypoint service
  switch (mode)
  {
    case 0:  // Land
    {
      mavros_msgs::CommandTOL land_cmd;
      land_cmd.request.yaw = 0;
      land_cmd.request.latitude = 0;
      land_cmd.request.longitude = 0;
      land_cmd.request.altitude = 0;

      ROS_INFO("Landing...");

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

      while (ros::ok() && !(current_ext_state.landed_state == current_ext_state.LANDED_STATE_ON_GROUND))
      {
        ros::spinOnce();
        rate.sleep();
      }

      ROS_INFO("Vehicle landed");

      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = false;

      ROS_INFO("Disarming...");

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

      ROS_INFO("Disarmed - Engines got turned off");
      break;
    }
    case 1:  // Absolut waypoint
    {
      while (ros::ok())
      {
        local_pos_pub.publish(last_request_waypoint);

        if ((last_request_waypoint.pose.position.x - current_pose.pose.position.x) <= 0.2 &&
            (last_request_waypoint.pose.position.y - current_pose.pose.position.y) <= 0.2 &&
            (last_request_waypoint.pose.position.z - current_pose.pose.position.z) <= 0.2)
        {
          ROS_INFO("The reached waypoint");
          break;
        }

        ros::spinOnce();
        rate.sleep();
      }

      break;
    }
    //    case 2:  // relative waypoint
    //    {
    //      // TBD
    //      break;
    //    }
    default:
    {
      ROS_WARN_STREAM("Unknown mode given from waypoint client");
      break;
    }
  }

  res.wp_reached = true;
  return true;
}
