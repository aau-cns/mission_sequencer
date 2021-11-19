// Copyright (C) 2021 Martin Scheiber, Christoph Boehm,
// and others, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <martin.scheiber@ieee.org>,
// and <christoph.boehm@aau.at>

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
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <mission_sequencer/MissionRequest.h>

// Include Publisher Messages
#include <mission_sequencer/MissionResponse.h>
#include <mission_sequencer/MissionWaypointArray.h>

// Include Services
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

// Waypoint list
#include "parse_waypoints.hpp"
#include "types/sequencer_options.hpp"
#include "types/sequencer_types.hpp"

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

namespace mission_sequencer
{
static const char* StateStr[] = { "IDLE", "ARM", "MISSION", "HOLD", "LAND", "DISARM" };

class MissionSequencer
{
  // ROS VARIABLES
private:
  // ROS Node handles
  ros::NodeHandle nh_;   //!< ROS node handle
  ros::NodeHandle pnh_;  //!< ROS private node handle

  // ROS Publishers
  ros::Publisher pub_pose_setpoint_;  //!< ROS publisher for current setpoint
  ros::Publisher pub_ms_response_;  //!< ROS publisher for mission sequencer request's response. This is similar to the
                                    //!< action feedback given once the request has been fullfilled.

  // ROS Subscribers
  ros::Subscriber sub_vehicle_state_;           //!< ROS subscriber for mavros vehicle state
  ros::Subscriber sub_extended_vehicle_state_;  //!< ROS subscriber for extended mavros vehicle state
  ros::Subscriber sub_vehicle_pose_;            //!< ROS subscriber for current vehicle pose
  ros::Subscriber sub_ms_request_;  //!< ROS subscirber for mission sequencer request (ARM, TAKEOFF, MISSION, LAND, etc)
  ros::Subscriber sub_waypoint_file_name_;  //!< ROS subscriber for waypoint file name

  // ROS Service Clients
  ros::ServiceClient srv_mavros_arm_;       //!< ROS service client to the 'arm' mavros interface
  ros::ServiceClient srv_mavros_disarm_;    //!< ROS service client to the 'disarm' mavros interface
  ros::ServiceClient srv_mavros_land_;      //!< ROS service client to the 'land' mavros interface
  ros::ServiceClient srv_mavros_set_mode_;  //!< ROS service client to the 'set mode' mavros interface

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

  void cbMSRequest(const mission_sequencer::MissionRequest::ConstPtr& msg);
  void cbWaypointFilename(const std_msgs::String::ConstPtr& msg);

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

private:
  bool b_pose_is_valid_{ false };          //!< flag to determine if a valid pose has been received
  bool b_state_is_valid_{ false };         //!< flag to determine if a valid mavros state has been received
  bool b_extstate_is_valid_{ false };      //!< flag to determine if a valid extended mavros state has been received
  bool b_is_landed_{ true };               //!< flag to determine if the vehicle has landed
  bool b_do_auto_state_change_{ false };   //!< flag to determine if state changes should happen automatically or per
                                           //!< request
  bool b_do_automatically_land_{ false };  //!< flag to determine if vehicle should automatically land
  bool b_wp_are_relativ_{ false };         //!< flag to determine if waypoints are relative to starting position
  bool b_wp_is_reached_{ false };          //!< flag to determine if waypoint has been reached
  bool b_do_verbose_{ false };             //!< flag to determine if debug output should be verbosed
                                //!< \deprecated will be removed in next version and replaced by the ROS debug flag

  // state machine
private:
  SequencerState current_sequencer_state_;
  SequencerState previous_sequencer_state_;
  MissionSequencerOptions sequencer_params_;

  // navigation variables
private:
  geometry_msgs::PoseStamped starting_vehicle_pose_;  //!< determines the start pose of the vehicle
  geometry_msgs::PoseStamped current_vehicle_pose_;   //!< determines the current pose of the vehicle
  geometry_msgs::PoseStamped setpoint_vehicle_pose_;  //!< determines the setpoint (goal) pose of the vehicle
  geometry_msgs::PoseStamped setpoint_takeoff_pose_;  //!< determines the setpoint (goal) pose for takeoff

  mavros_msgs::State current_vehicle_state_;              //!< determines the current vehicle mavros state
  mavros_msgs::ExtendedState current_vehicle_ext_state_;  //!< determines the current vehcile extended mavros state

  uint8_t current_mission_ID_;
  // REST - WIP
private:
  int requestNumber_;

  std::vector<ParseWaypoint::Waypoint> waypointList_;
  ros::Time time_last_wp_reached_;

  mavros_msgs::SetMode offboardMode_;
  mavros_msgs::CommandBool armCmd_;
  mavros_msgs::CommandLong disarmCmd_;
  mavros_msgs::CommandTOL landCmd_;
  ros::Time armRequestTime_;
  ros::Time disarmRequestTime_;
  ros::Time offboardRequestTime_;

  double thresholdPosition_;
  double thresholdYaw_;

  static const size_t dbg_throttle_rate_ = 10;
  std::string waypoint_fn_ = "";

  /// vector of filenames read from parameter server
  std::vector<std::string> filenames_;

  geometry_msgs::PoseStamped waypointToPoseStamped(const ParseWaypoint::Waypoint& waypoint);

  bool getFilenames();
  bool setFilename(std::string const waypoint_fn);

  bool checkWaypoint(const geometry_msgs::PoseStamped& current_waypoint);
  bool checkStateChange(const SequencerState new_state) const;

public:
  MissionSequencer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MissionSequencer();

  void logic(void);
  void publishPoseSetpoint(void);
};  // class MissionSequencer

}  // namespace mission_sequencer

#endif  // MISSION_SEQUENCER_HPP
