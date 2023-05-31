// Copyright (C) 2023 Christian Brommer, Martin Scheiber, Christoph Boehm,
// and others, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <christian.brommer@ieee.org>,
// <martin.scheiber@ieee.org>, and <christoph.boehm@aau.at>

#ifndef MISSION_SEQUENCER_HPP
#define MISSION_SEQUENCER_HPP

#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

// Include Subscriber Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <mission_sequencer/MissionRequest.h>
#include <nav_msgs/Odometry.h>

// Include Publisher Messages
#include <mission_sequencer/MissionResponse.h>
#include <mission_sequencer/MissionWaypoint.h>
#include <mission_sequencer/MissionWaypointArray.h>
#include <mission_sequencer/MissionWaypointStamped.h>

// Include Services
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mission_sequencer/GetStartPose.h>

// Waypoint list
#include "types/sequencer_options.hpp"
#include "types/sequencer_types.hpp"
#include "types/sequencer_waypoint.hpp"

//#define RAD_TO_DEG (180.0 / M_PI)
//#define DEG_TO_RAD (M_PI / 180.0)

namespace mission_sequencer
{
class MissionSequencer
{
private:
  ///
  /// \brief The MavrosCommands struct implements a hardcoded structure to easily send ARM, DISARM, LAND and MODE
  /// commands to mavros
  ///
  struct MavrosCommands
  {
    mavros_msgs::SetMode offboard_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::CommandLong disarm_cmd_;
    mavros_msgs::CommandTOL land_cmd_;

    ros::Time time_arm_request;
    ros::Time time_disarm_request;
    ros::Time time_offboard_request;

    ///
    /// \brief MavrosCommands default constructor setting the default values for all commands.
    ///
    /// \see https://mavlink.io/en/messages/common.html#COMMAND_LONG
    /// \see https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
    ///
    /// \author ALF
    ///
    MavrosCommands()
    {
      // setup disarm command
      disarm_cmd_.request.broadcast = false;
      disarm_cmd_.request.command = 400;
      disarm_cmd_.request.confirmation = 0;
      disarm_cmd_.request.param1 = 0.0;
      disarm_cmd_.request.param2 = 21196.0;
      disarm_cmd_.request.param3 = 0.0;
      disarm_cmd_.request.param4 = 0.0;
      disarm_cmd_.request.param5 = 0.0;
      disarm_cmd_.request.param6 = 0.0;
      disarm_cmd_.request.param7 = 0.0;

      // setup arm command
      arm_cmd_.request.value = true;

      // setup land command
      land_cmd_.request.yaw = 0;
      land_cmd_.request.latitude = 0;
      land_cmd_.request.longitude = 0;
      land_cmd_.request.altitude = 0;

      // setup offboard mode
      offboard_mode_.request.custom_mode = "OFFBOARD";

      // set times to current ros time
      ros::Time request_time = ros::Time::now();
      time_arm_request = request_time;
      time_disarm_request = request_time;
      time_offboard_request = request_time;
    }
  };

  // ROS VARIABLES
private:
  // ROS Node handles
  ros::NodeHandle nh_;   //!< ROS node handle
  ros::NodeHandle pnh_;  //!< ROS private node handle

  // ROS Publishers
  ros::Publisher pub_pose_setpoint_;  //!< ROS publisher for current setpoint
  ros::Publisher pub_ms_response_;  //!< ROS publisher for mission sequencer request's response. This is similar to the
                                    //!< action feedback given once the request has been fullfilled.
  ros::Publisher pub_waypoint_list_;     //!< ROS publisher for current waypoint list
  ros::Publisher pub_waypoint_reached_;  //!< ROS publisher for last waypoint reached

  // ROS Subscribers
  ros::Subscriber sub_vehicle_state_;           //!< ROS subscriber for mavros vehicle state
  ros::Subscriber sub_extended_vehicle_state_;  //!< ROS subscriber for extended mavros vehicle state
  ros::Subscriber sub_vehicle_pose_;            //!< ROS subscriber for current vehicle pose
  ros::Subscriber sub_vehicle_odom_;            //!< ROS subscriber for current vehicle odometry
  ros::Subscriber sub_ms_request_;  //!< ROS subscirber for mission sequencer request (ARM, TAKEOFF, MISSION, LAND, etc)
  ros::Subscriber sub_waypoint_file_name_;  //!< ROS subscriber for waypoint file name
  ros::Subscriber sub_waypoint_list_;       //!< ROS subscriber for waypoint list

  // ROS Service Clients
  ros::ServiceClient srv_mavros_arm_;       //!< ROS service client to the 'arm' mavros interface
  ros::ServiceClient srv_mavros_disarm_;    //!< ROS service client to the 'disarm' mavros interface
  ros::ServiceClient srv_mavros_land_;      //!< ROS service client to the 'land' mavros interface
  ros::ServiceClient srv_mavros_set_mode_;  //!< ROS service client to the 'set mode' mavros interface

  // ROS Service Servers
  ros::ServiceServer srv_get_start_pose_;
  ros::ServiceServer srv_get_waypoint_list_;

  // ROS METHODS
private:
  ///
  /// \brief cbVehicleState ROS topic callback for the mavros vehicle state
  /// \param msg mavros vehicle state
  ///
  void cbVehicleState(const mavros_msgs::State::ConstPtr& msg);

  ///
  /// \brief cbExtendedVehicleState ROS topic callback for the extended mavros vehicle state
  /// \param msg mavros extended vehicle state
  ///
  void cbExtendedVehicleState(const mavros_msgs::ExtendedState::ConstPtr& msg);

  ///
  /// \brief cbPose ROS topic callback for the current vehicle pose
  /// \param msg current vehicle pose in the 'global' navigation frame
  ///
  /// This callback sets the current vehicle pose used for checking waypoint reached in the mission phase.
  /// The first time this function is called the stating_vehicle_pose_ is set to the received pose.
  ///
  void cbPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  ///
  /// \brief cbOdom ROS topic callback for the current vehicle odometry
  /// \param msg curren vehicle odometry in the 'global' navigation frame
  ///
  /// This callback sets the current vehicle pose and velocity used for checking waypoint reached in the mission phase.
  /// The first time this function is called the starting_vehicle_pose_ is set to the received pose.
  ///
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);

  void cbMSRequest(const mission_sequencer::MissionRequest::ConstPtr& msg);
  void cbWaypointFilename(const std_msgs::String::ConstPtr& msg);

  ///
  /// \brief cbWaypointList ROS topic callback for a list of waypoitns
  /// \param msg array of waypoints in the 'global' navigation frame
  ///
  /// This either
  /// 1. replaces,
  /// 2. appends, or
  /// 3. inserts (NOT YET IMPLEMENTED)
  /// the given waypoints to the current list of waypoints.
  ///
  /// \todo TODO(scm): implement the insert functionality
  ///
  void cbWaypointList(const mission_sequencer::MissionWaypointArrayConstPtr& msg);

  bool srvGetStartPose(GetStartPose::Request& req, mission_sequencer::GetStartPose::Response& res);

  ///
  /// \brief publishResponse publishes the reponse to a mission request onto the ROS network
  /// \param id mission ID in use
  /// \param request requested state
  /// \param response reply if transition to new state is approved
  /// \param completed reply if state task has been completed
  ///
  /// The possible outcome of the request and thus published response are displayed in the table below
  /// <table>
  /// <caption id="multi_row">Possible Response situations</caption>
  /// <tr><th>publish time              <th>response  <th>completed <th>description
  /// <tr><td>immediatly after request  <td>0         <td> 0        <td>request denied
  /// <tr><td>immediatly after request  <td>1         <td> 0        <td>request accepted, not executed yet
  /// <tr><td>immediatly after request  <td>1         <td> 1        <td>request accepted and executed
  /// <tr><td>any time after request    <td>0         <td> 1        <td>previous request has been executed
  /// </table>
  ///
  void publishResponse(const uint8_t& id, const uint8_t& request, const bool& response, const bool& completed) const;

  // EXECUTORS
private:
  void performIdle();

  ///
  /// \brief performPrearming reads the waypoints from CSV files and waits for ARMING
  ///
  void performPrearming();
  void performArming();

  ///
  /// \brief performTakeoff performs the takeoff based on the selected type
  ///
  void performTakeoff();

  ///
  /// \brief performMission
  ///
  void performMission();
  void performHover();
  void performLand();
  void performHold();
  void performDisarming();
  void performAbort();

  void updatePose(const geometry_msgs::PoseStamped& pose);

private:
  bool b_pose_is_valid_{ false };            //!< flag to determine if a valid pose has been received
  bool b_odom_is_valid_{ false };            //!< flag to determine if a valid pose has been received
  bool b_state_is_valid_{ false };           //!< flag to determine if a valid mavros state has been received
  bool b_extstate_is_valid_{ false };        //!< flag to determine if a valid extended mavros state has been received
  bool b_executed_landing_{ false };         //!< flag to determine if a landing command has been executed
  bool b_is_landed_{ true };                 //!< flag to determine if the vehicle has landed
  bool b_do_auto_state_change_{ false };     //!< \deprecated flag to determine if state changes should happen
                                             //!< automatically or per request
  bool b_do_automatically_land_{ false };    //!< \deprecated flag to determine if vehicle should automatically land
  bool b_do_automatically_disarm_{ false };  //!< \deprecated flag to deterime if vehicle should be automatically
                                             //!< disarmed
  bool b_wp_are_relativ_{ false };  //!< \deprecated flag to determine if waypoints are relative to starting position
  bool b_wp_is_reached_{ false };   //!< flag to determine if waypoint has been reached
  bool b_do_verbose_{ false };      //!< flag to determine if debug output should be verbosed
                                    //!< \deprecated will be removed in next version and replaced by the ROS debug flag

  // state machine
private:
  SequencerState current_sequencer_state_;
  SequencerState previous_sequencer_state_;
  MissionSequencerOptions sequencer_params_;
  double time_last_valid_request_;  //!< time stamp of last received valid request, used for timeouts

  // navigation variables
private:
  geometry_msgs::PoseStamped starting_vehicle_pose_;   //!< determines the start pose of the vehicle
  geometry_msgs::PoseStamped current_vehicle_pose_;    //!< determines the current pose of the vehicle
  geometry_msgs::PoseStamped setpoint_vehicle_pose_;   //!< determines the setpoint (goal) pose of the vehicle
  geometry_msgs::PoseStamped setpoint_takeoff_pose_;   //!< determines the setpoint (goal) pose for takeoff
  geometry_msgs::TwistStamped current_vehicle_twist_;  //!< determines the current velocity of the vehicle
  bool current_vel_reached_[3] = { false, false, false };

  mavros_msgs::State current_vehicle_state_;              //!< determines the current vehicle mavros state
  mavros_msgs::ExtendedState current_vehicle_ext_state_;  //!< determines the current vehcile extended mavros state

  uint8_t current_mission_ID_;  //!< determines the ID of the current mission (used to safeguard that requests are
                                //!< acutally made for the correct mission)
  std::vector<Waypoint> waypoint_list_;  //!< list of waypoints currently in use
  ros::Time time_last_wp_reached_;       //!< time since last waypoint was reached

  // communication variables
private:
  MavrosCommands mavros_cmds_;

  // REST - WIP
private:
  static constexpr double dbg_throttle_rate_ = 0.01;  // 3.0;
  std::string waypoint_fn_ = "";

  /// vector of filenames read from parameter server
  std::vector<std::string> filenames_;

  geometry_msgs::PoseStamped waypointToPoseStamped(const Waypoint& waypoint);

  // obtaining a vector of file names depending a ROS parameter with the current mission id
  // or via a waypoint_fn_ (higher priority).
  bool reloadFilenames();
  bool setWaypointFilename(const std::string waypoint_fn);
  bool parseFilename();

  // METHODS TO CHECK TRANSITIONS, IDS, STATES, ETC.
private:
  bool checkWaypoint(const geometry_msgs::PoseStamped& current_waypoint);
  bool checkVelocity(const geometry_msgs::TwistStamped& set_velocity);
  bool checkStateChange(const SequencerState& new_state) const;

  ///
  /// @brief checkRequestTime checks if the last request time was within the request timeout
  /// @return true|false if the (current time - last request time) < timeout
  ///
  inline bool checkRequestTime() const
  {
    return ros::Time::now().toSec() - time_last_valid_request_ < sequencer_params_.request_timeout_s_;
  }

  ///
  /// \brief checkMissionID checks if the mission ID is suitable with the current mission ID
  /// \param mission_id
  /// \param request_id
  /// \return
  ///
  /// The mission_id is deemed suitable if it is exactly the same as the current_mission_ID_ or if the
  /// current_sequencer_state is either IDLE or PREARM given that the request_id corresponds to a READ request.
  ///
  bool checkMissionID(const uint8_t& mission_id, const uint8_t& request_id);

  ///
  /// \brief executeLanding executes any landing by sending the command to the mavros interface
  /// \return true, if the command has been successfully sent to mavros
  ///
  bool executeLanding();

public:
  MissionSequencer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MissionSequencer();

  void logic(void);
  void publishPoseSetpoint(void);
};  // class MissionSequencer

}  // namespace mission_sequencer

#endif  // MISSION_SEQUENCER_HPP
