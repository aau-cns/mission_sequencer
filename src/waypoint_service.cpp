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

#include <amaze_waypoint_following/wp_service.h>
#include <geometry_msgs/PoseStamped.h>
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

static mavros_msgs::State current_state_;
static mavros_msgs::ExtendedState current_ext_state_;
static geometry_msgs::PoseStamped current_pose_;
static geometry_msgs::PoseStamped last_request_waypoint_;

// Subscriber
static ros::Subscriber current_sub_;
static ros::Subscriber state_sub_;
static ros::Subscriber ext_state_sub_;
// Publisher
static ros::Publisher local_pos_pub_;
// Service client handle
static ros::ServiceClient arming_client_;
static ros::ServiceClient land_client_;
static ros::ServiceClient set_mode_client_;
// Waypoint service server
static ros::ServiceServer wp_service_;

///
/// \brief standby: True if the vehicle is not in flight and the node does not need to publish waypoints
///
static bool standby_ = true;
///
/// \brief pose_msg_ok True if a pose message from the PX4 has been received
///
static bool pose_msg_ok_ = false;
///
/// \brief state_msg_ok True if a state message from the PX4 has been received
///
static bool state_msg_ok_ = false;
///
/// \brief ext_state_msg_ok True if an extended state message from the PX4 has been received
///
static bool ext_state_msg_ok_ = false;

static double threshold_wp_reached_ = 0.2;

// degrees to radians transformation constant
double deg_rad_ = 1 / (180.0 / 3.141592653589793238463);
///
/// \brief state_cb PX4 state message subscriber callback
/// \param msg
///
void state_cb(const mavros_msgs::State::ConstPtr& msg);

///
/// \brief ext_state_cb
/// \param msg
///
void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);

///
/// \brief current_cb PX4 current pose message subscriber callback
/// \param msg
///
void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

///
/// \brief wp_srv_callback Advertised service for requesting waypoints
/// \param req
/// \param res
/// \return
///
bool wp_srv_callback(amaze_waypoint_following::wp_service::Request& req,
                     amaze_waypoint_following::wp_service::Response& res);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ///
  /// \brief rate publishing rate for waypoints
  /// The setpoint publishing rate MUST be faster than 2Hz
  ///
  ros::Rate rate(20.0);

  // Create subscriber
  current_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, current_pose_cb);
  state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ext_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, ext_state_cb);

  // Create publisher
  local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  // Create service client handle
  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  land_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // Create waypoint service server
  wp_service_ = nh.advertiseService("wp_service", wp_srv_callback);
  ROS_INFO("Server is ready!");

  while (ros::ok())
  {
    if (standby_ == false)
    {
      local_pos_pub_.publish(last_request_waypoint_);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_ = *msg;
  state_msg_ok_ = true;
}

void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  current_ext_state_ = *msg;
  ext_state_msg_ok_ = true;
}

void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_ = *msg;
  pose_msg_ok_ = true;
}

bool wp_srv_callback(amaze_waypoint_following::wp_service::Request& req,
                     amaze_waypoint_following::wp_service::Response& res)
{
  // Check if critical messages have been received
  if (pose_msg_ok_ == false)
  {
    ROS_WARN("Did not receive a pose msg from the vehicle yet");
    res.wp_reached = false;
    return false;
  }
  if (state_msg_ok_ == false)
  {
    ROS_WARN("Did not receive a state msg from the vehicle yet");
    res.wp_reached = false;
    return false;
  }
  if (ext_state_msg_ok_ == false)
  {
    ROS_WARN("Did not receive an ext_state msg from the vehicle yet");
    res.wp_reached = false;
    return false;
  }

  standby_ = false;
  ///
  /// \brief rate publishing rate for waypoints
  /// The setpoint publishing rate MUST be faster than 2Hz
  ///
  ros::Rate rate(20.0);

  const int mode = req.mode;
  geometry_msgs::PoseStamped init_pose_(current_pose_);  // registered for relative waypoint navigation case

  switch (mode)
  {
    // Map Position
    case 1:
    {
      last_request_waypoint_.pose.position.x = double(req.x);
      last_request_waypoint_.pose.position.y = double(req.y);
      last_request_waypoint_.pose.position.z = double(req.z);

      // Map Orientation
      const double yaw = double(req.yaw);
      const double req_yaw_rad = yaw * deg_rad_;
      tf2::Quaternion q_wp;
      q_wp.setRotation(tf2::Vector3(0, 0, 1), req_yaw_rad);  // 45 degrees rotation
      q_wp.normalize();
      last_request_waypoint_.pose.orientation.w = q_wp[3];  // real part of quaternion
      last_request_waypoint_.pose.orientation.x = q_wp[0];
      last_request_waypoint_.pose.orientation.y = q_wp[1];
      last_request_waypoint_.pose.orientation.z = q_wp[2];
      break;
    }
    case 2:
    {
      last_request_waypoint_.pose.position.x = double(req.x) + init_pose_.pose.position.x;
      last_request_waypoint_.pose.position.y = double(req.y) + init_pose_.pose.position.y;
      last_request_waypoint_.pose.position.z = double(req.z) + init_pose_.pose.position.z;

      // Map Orientation
      tf2::Quaternion init_quat;
      init_quat[3] = init_pose_.pose.orientation.w;
      init_quat[0] = init_pose_.pose.orientation.x;
      init_quat[1] = init_pose_.pose.orientation.y;
      init_quat[2] = init_pose_.pose.orientation.z;
      init_quat.normalize();

      // yaw (z-axis initial rotation)
      // since there's no pitch and roll only yaw -> init_quat[0] = init_quat[1] =0
      double siny_cosp = 2 * (init_quat[3] * init_quat[2]);
      double cosy_cosp = 1 - 2 * (init_quat[2] * init_quat[2]);
      double initial_yaw = atan2(siny_cosp, cosy_cosp);

      double initial_yaw_degrees = initial_yaw * deg_rad_;
      ROS_INFO_STREAM("inital angle:" << initial_yaw_degrees);
      double yaw = double(req.yaw);
      ROS_INFO_STREAM("requested yaw:" << yaw);
      double req_yaw_rad = initial_yaw + yaw * deg_rad_;
      ROS_INFO_STREAM("turning angle:" << req_yaw_rad / deg_rad_);

      tf2::Quaternion q_wp;
      q_wp.setRotation(tf2::Vector3(0, 0, 1), req_yaw_rad);  // radians rotation
      q_wp.normalize();
      last_request_waypoint_.pose.orientation.w = q_wp[3];  // real part of quaternion
      last_request_waypoint_.pose.orientation.x = q_wp[0];
      last_request_waypoint_.pose.orientation.y = q_wp[1];
      last_request_waypoint_.pose.orientation.z = q_wp[2];
      break;
    }
    // Land
    case 0:
    {
      if (current_ext_state_.landed_state == current_ext_state_.LANDED_STATE_ON_GROUND)
      {
        ROS_WARN("Requested landing but vehicle is already on the ground");
        break;
      }

      mavros_msgs::CommandTOL land_cmd;
      land_cmd.request.yaw = 0;
      land_cmd.request.latitude = 0;
      land_cmd.request.longitude = 0;
      land_cmd.request.altitude = 0;

      ROS_INFO("Landing...");

      while (ros::ok())
      {
        if (land_client_.call(land_cmd))
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

      ROS_INFO("Vehicle landed");

      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = false;

      ROS_INFO("Disarming...");

      while (ros::ok())
      {
        if (arming_client_.call(arm_cmd))
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
      res.wp_reached = true;
      return true;
    }
    default:
    {
      ROS_WARN_STREAM("Unknown mode given from waypoint client");
      break;
    }

  }  // end of switch
  ROS_INFO_STREAM("Request: mode= " << int(req.mode) << "Position (xyz): [" << req.x << " " << req.y << " " << req.z
                                    << "] "
                                    << " yaw [deg]: " << req.yaw);
  // current position with respect to the absolute frame
  ROS_INFO_STREAM("Current Position [" << current_pose_.pose.position.x << " " << current_pose_.pose.position.y << " "
                                       << current_pose_.pose.position.z << "]");

  // ROS_INFO_STREAM("Current orientation [" << init_pose.pose.orientation.x << " " << init_pose.pose.orientation.y << "
  // "
  //                                       << init_pose.pose.orientation.z << "]");
  // ROS_INFO_STREAM("requested orientation [" << last_request_waypoint.pose.orientation.x << " " <<
  // last_request_waypoint.pose.orientation.y << " "
  //                                       << last_request_waypoint.pose.orientation.z << "]");

  // Check that the vehicle is not in the air and thus requires switching to offboard and arming before takeoff
  if (current_ext_state_.landed_state == current_ext_state_.LANDED_STATE_ON_GROUND)
  {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time t_last_request = ros::Time::now();

    // Change to offboard mode and arm the vehicle, only if the vehicle is not armed and the requested mode is not
    // landing
    while (ros::ok() && !current_state_.armed && mode != 0)
    {
      // Switch to offboard mode
      if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - t_last_request) > ros::Duration(2.5))
      {
        ROS_INFO(current_state_.mode.c_str());
        if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
        t_last_request = ros::Time::now();
      }
      else
      {
        // Arm the vehicle
        if (!current_state_.armed && (ros::Time::now() - t_last_request) > ros::Duration(2.5))
        {
          if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("Vehicle armed");
          }
          t_last_request = ros::Time::now();
        }
      }

      // Send a few setpoints before starting
      for (int i = 100; ros::ok() && i > 0; --i)
      {
        local_pos_pub_.publish(last_request_waypoint_);
        ros::spinOnce();
        rate.sleep();
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

  // Perform requested waypoint service

  while (ros::ok())
  {
    local_pos_pub_.publish(last_request_waypoint_);

    if (abs(last_request_waypoint_.pose.position.x - current_pose_.pose.position.x) <= threshold_wp_reached_ &&
        abs(last_request_waypoint_.pose.position.y - current_pose_.pose.position.y) <= threshold_wp_reached_ &&
        abs(last_request_waypoint_.pose.position.z - current_pose_.pose.position.z) <= threshold_wp_reached_)
    {
      ROS_INFO("The reached waypoint");

      break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  res.wp_reached = true;
  return true;
}
