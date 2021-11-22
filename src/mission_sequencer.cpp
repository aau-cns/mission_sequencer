// Copyright (C) 2021 Christian Brommer, Martin Scheiber, Christoph Boehm,
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

#include "mission_sequencer.hpp"

#include "utils/message_conversion.hpp"

namespace mission_sequencer
{
double warp_to_pi(double const angle_rad)
{
  bool is_neg = (angle_rad < 0);
  double differenceYaw = std::fmod(abs(angle_rad), 2 * M_PI);
  if (differenceYaw > M_PI)
  {
    differenceYaw = std::abs(differenceYaw - 2 * M_PI);
  }
  if (is_neg)
  {
    return -differenceYaw;
  }
  return differenceYaw;
}

MissionSequencer::MissionSequencer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), current_sequencer_state_{ SequencerState::IDLE }
{
  current_vehicle_state_ = mavros_msgs::State();
  current_vehicle_ext_state_ = mavros_msgs::ExtendedState();
  current_vehicle_pose_ = geometry_msgs::PoseStamped();

  starting_vehicle_pose_ = geometry_msgs::PoseStamped();

  setpoint_vehicle_pose_ = geometry_msgs::PoseStamped();

  current_mission_ID_ = 0;
  requestNumber_ = 0;

  waypointList_ = std::vector<ParseWaypoint::Waypoint>(0);
  time_last_wp_reached_ = ros::Time::now();

  filenames_ = std::vector<std::string>(0);

  offboardMode_.request.custom_mode = "OFFBOARD";
  armCmd_.request.value = true;
  landCmd_.request.yaw = 0;
  landCmd_.request.latitude = 0;
  landCmd_.request.longitude = 0;
  landCmd_.request.altitude = 0;
  armRequestTime_ = ros::Time::now();
  offboardRequestTime_ = ros::Time::now();

  b_wp_are_relativ_ = true;

  b_is_landed_ = false;

  // ros::NodeHandle private_nh("~");
  // Load Parameters (privately)
  if (!pnh_.getParam("threshold_position", thresholdPosition_))
  {
    ROS_WARN("Could not retrieve threshold for position, setting to 0.3m");
    thresholdPosition_ = 0.3;
  }
  if (!pnh_.getParam("threshold_yaw", thresholdYaw_))
  {
    ROS_WARN("Could not retrieve threshold for yaw, setting to 0.1 rad");
    thresholdYaw_ = 0.1;
  }
  std::string waypoint_fn;
  pnh_.param<std::string>("waypoint_filename", waypoint_fn, "");
  pnh_.param<bool>("automatic_landing", b_do_automatically_land_, false);
  pnh_.param<bool>("verbose", b_do_verbose_, false);
  pnh_.param<bool>("relative_waypoints", b_wp_are_relativ_, true);

  // Subscribers
  sub_vehicle_state_ = nh.subscribe("/mavros/state", 10, &MissionSequencer::cbVehicleState, this);
  sub_extended_vehicle_state_ =
      nh.subscribe("/mavros/extended_state", 10, &MissionSequencer::cbExtendedVehicleState, this);
  sub_vehicle_pose_ = nh.subscribe("/mavros/local_position/pose", 10, &MissionSequencer::cbPose, this);
  sub_ms_request_ = nh.subscribe("/autonomy/request", 10, &MissionSequencer::cbMSRequest, this);

  // Subscribers (relative to node's namespace)
  sub_vehicle_state_ = nh_.subscribe("mavros/state", 10, &MissionSequencer::cbVehicleState, this);
  sub_extended_vehicle_state_ =
      nh_.subscribe("mavros/extended_state", 10, &MissionSequencer::cbExtendedVehicleState, this);
  sub_vehicle_pose_ = nh_.subscribe("mavros/local_position/pose", 10, &MissionSequencer::cbPose, this);
  sub_ms_request_ = nh_.subscribe("autonomy/request", 10, &MissionSequencer::cbMSRequest, this);
  sub_waypoint_file_name_ = pnh_.subscribe("waypoint_filename", 10, &MissionSequencer::cbWaypointFilename, this);
  sub_waypoint_list_ = pnh_.subscribe("waypoint_list", 10, &MissionSequencer::cbWaypointList, this);

  // Publishers (relative to node's namespace)
  pub_pose_setpoint_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  pub_ms_response_ = nh_.advertise<mission_sequencer::MissionResponse>("autonomy/response", 10);

  // Services (relative to node's namespace)
  std::string service_mavros_cmd_arming;
  pnh_.param<std::string>("mavros_cmd_arming_o", service_mavros_cmd_arming, "mavros/cmd/arming");
  std::string service_mavros_cmd_command;
  pnh_.param<std::string>("mavros_cmd_command_o", service_mavros_cmd_command, "mavros/cmd/command");
  std::string service_mavros_cmd_land;
  pnh_.param<std::string>("mavros_cmd_land_o", service_mavros_cmd_land, "mavros/cmd/land");
  std::string service_mavros_set_mode;
  pnh_.param<std::string>("mavros_set_mode_o", service_mavros_set_mode, "mavros/set_mode");

  srv_mavros_arm_ = nh_.serviceClient<mavros_msgs::CommandBool>(service_mavros_cmd_arming);
  srv_mavros_disarm_ = nh_.serviceClient<mavros_msgs::CommandLong>(service_mavros_cmd_command);
  srv_mavros_land_ = nh_.serviceClient<mavros_msgs::CommandTOL>(service_mavros_cmd_land);
  srv_mavros_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>(service_mavros_set_mode);

  setFilename(waypoint_fn);
};

MissionSequencer::~MissionSequencer(){

};

void MissionSequencer::cbVehicleState(const mavros_msgs::State::ConstPtr& msg)
{
  b_state_is_valid_ = true;
  current_vehicle_state_ = *msg;
};

void MissionSequencer::cbExtendedVehicleState(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  b_extstate_is_valid_ = true;
  current_vehicle_ext_state_ = *msg;
};

void MissionSequencer::cbPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // check if starting position has been determined, i.e. the current pose is valid and MS is in IDLE

  if (!b_pose_is_valid_ || (current_sequencer_state_ == SequencerState::IDLE))
  {
    /// \todo make this to Eigen variable
    starting_vehicle_pose_ = *msg;

    /// \footnote this had been developed by RJ, needs to be clarified why this is needed
    /// \todo clarify conversion from ENU to NED

    // Initial YAW as it is used for LANDING
    // The vehilce pose is in ENU:
    tf2::Quaternion q_ENU_BODY(starting_vehicle_pose_.pose.orientation.x, starting_vehicle_pose_.pose.orientation.y,
                               starting_vehicle_pose_.pose.orientation.z, starting_vehicle_pose_.pose.orientation.w);

    tf2::Quaternion q_NED_2_ENU(0, 0, 0.7071068, 0.7071068);  // [x,y,z,w]
    // q_NED_2_ENU.setRotation(tf2::Vector3(0.7071068, 0, 0.7071068), M_PI);
    // q_NED_2_ENU.normalize();
    tf2::Quaternion q_y;
    q_y.setRotation(tf2::Vector3(0, 1, 0), M_PI);

    tf2::Quaternion q_NED_BODY = q_y * q_NED_2_ENU * q_ENU_BODY;
    double startingYaw, startingPitch, startingRoll;
    tf2::Matrix3x3(q_NED_BODY).getEulerYPR(startingYaw, startingPitch, startingRoll);
    startingYaw = warp_to_pi(startingYaw);
    if (b_do_verbose_)
    {
      ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* Initial (PX4/NED) yaw= "
                                                       << startingYaw * RAD_TO_DEG << "[deg], (OptiTrack/ENU) pos x="
                                                       << starting_vehicle_pose_.pose.position.x
                                                       << " pos y=" << starting_vehicle_pose_.pose.position.y
                                                       << " pos z=" << starting_vehicle_pose_.pose.position.z);
    }

    landCmd_.request.yaw = startingYaw * RAD_TO_DEG;
    landCmd_.request.altitude = starting_vehicle_pose_.pose.position.z;

    // set the current goal to the current pose
    setpoint_vehicle_pose_ = starting_vehicle_pose_;
    b_pose_is_valid_ = true;
  }

  // update the current vehicle pose
  current_vehicle_pose_ = *msg;
};

bool MissionSequencer::getFilenames()
{
  // Define filepaths
  XmlRpc::XmlRpcValue filepaths;

  // get filepaths
  if (!waypoint_fn_.empty())
  {
    filenames_.clear();
    filenames_.emplace_back(waypoint_fn_);
    return true;
  }
  else
  {
    if (!nh_.getParam("autonomy/missions/mission_" + std::to_string(current_mission_ID_) + "/filepaths", filepaths))
    {
      // [TODO] Manage error
      ROS_WARN_STREAM("AmazeMissionSequencer::getFilenames(): failure! Cound not get file paths for mission:"
                      << std::to_string(current_mission_ID_));
      return false;
    }
    // Check type to be array
    if (filepaths.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      // Loop through filepaths
      for (int j = 0; j < filepaths.size(); ++j)
      {
        // Check type to be string
        if (filepaths[j].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          // assign filename
          filenames_.emplace_back(std::string(filepaths[j]));
        }
      }
      return true;
    }
  }

  return false;
};

bool MissionSequencer::setFilename(std::string const waypoint_fn)
{
  if (!waypoint_fn.empty())
  {
    ROS_INFO_STREAM("*  Received single waypoint filename: " << waypoint_fn);
    std::ifstream file(waypoint_fn);

    if (!file)
    {
      ROS_WARN_STREAM("ERROR: failed opening file: " << waypoint_fn);
    }
    else
    {
      waypoint_fn_ = waypoint_fn;
    }
  }
};

void MissionSequencer::cbMSRequest(const mission_sequencer::MissionRequest::ConstPtr& msg)
{
  bool b_wrong_input = false;

  // check mission id
  if (!checkMissionID(msg->id, msg->request))
  {
    // WRONG ID
    // check if the request is to read mission files
    ROS_INFO_STREAM("WRONG MISSION ID FOR CURRENT STATE: " << current_sequencer_state_ << "; Local mission ID:"
                                                           << current_mission_ID_ << " msg ID: " << int(msg->id));
    // Respond if input was wrong
    publishResponse(current_mission_ID_, msg->request, false, false);
    return;
  }

  // main request state machine
  switch (msg->request)
  {
      // arming, no takeoff
    case mission_sequencer::MissionRequest::ARM: {
      ROS_DEBUG_STREAM("* amaze_mission_sequencer::request::ARM...");
      if (checkStateChange(SequencerState::ARM) && b_pose_is_valid_ && b_state_is_valid_ && b_extstate_is_valid_)
      {
        //        // Take first entry of filename list
        //        std::string filename = filenames_[0];

        //        std::vector<std::string> header_default = { "x", "y", "z", "yaw", "holdtime" };
        //        std::shared_ptr<ParseWaypoint> WaypointParser = std::make_shared<ParseWaypoint>(filename,
        //        header_default);

        //        // Parse waypoint file
        //        WaypointParser->readParseCsv();

        //        // Get the data
        //        waypointList_ = WaypointParser->getData();

        // Set initial pose
        starting_vehicle_pose_ = current_vehicle_pose_;
        setpoint_vehicle_pose_ = starting_vehicle_pose_;

        //        if (waypointList_.size() == 0)
        //        {
        //          // Error if waypoint list is empty
        //          publishResponse(missionID_, int(msg->request), false, false);
        //          ROS_INFO_STREAM("MISSION FILE EMPTY");
        //          return;
        //        }

        // Preparation for arming
        armCmd_.request.value = true;
        armRequestTime_ = ros::Time::now();
        offboardRequestTime_ = ros::Time::now();
        current_sequencer_state_ = SequencerState::ARM;

        // Respond to request
        publishResponse(current_mission_ID_, int(msg->request), true, false);
      }
      else
      {
        if (b_do_verbose_)
        {
          ROS_WARN_STREAM("* amaze_mission_sequencer::request::ARM - failed! Not in PREARM!");
          ROS_WARN_STREAM("*   Valids: Pose=" << b_pose_is_valid_ << ", State=" << b_state_is_valid_
                                              << ", extState=" << b_extstate_is_valid_);
        }
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::TAKEOFF: {
      ROS_DEBUG_STREAM("* amaze_mission_sequencer::request::TAKEOFF...");
      // check if change is approved
      /// \todo TODO(scm): if enum conversion from string exists (or int) then this can be put outside the switch
      if (checkStateChange(SequencerState::TAKEOFF))
      {
        // state change to TAKEOFF is approved
        if (sequencer_params_.takeoff_type_ == TakeoffType::POSITION)
        {
          // set starting position if they are relative
          if (sequencer_params_.b_wp_are_relative_)
            starting_vehicle_pose_ = current_vehicle_pose_;

          // set takeoff position
          setpoint_takeoff_pose_ = starting_vehicle_pose_;
          setpoint_takeoff_pose_.pose.position.z += sequencer_params_.takeoff_z_;

          // set waypoint reached to false and transition to new state
          b_wp_is_reached_ = false;
          current_sequencer_state_ = SequencerState::TAKEOFF;
        }
        else
        {
          ROS_ERROR_STREAM("=> TakeoffType not implemented");
        }
      }
      break;
    }

    case mission_sequencer::MissionRequest::HOLD: {
      ROS_DEBUG_STREAM("* amaze_mission_sequencer::request::HOLD...");
      // check if change is approved
      if (checkStateChange(SequencerState::HOLD))
      {
        // save previous state
        previous_sequencer_state_ = current_sequencer_state_;
        ROS_DEBUG_STREAM("- prehold state: " << previous_sequencer_state_);

        // set holding position
        setpoint_vehicle_pose_ = current_vehicle_pose_;
        ROS_INFO_STREAM("Hold Position: "
                        << "  x = " << setpoint_vehicle_pose_.pose.position.x
                        << ", y = " << setpoint_vehicle_pose_.pose.position.y
                        << ", z = " << setpoint_vehicle_pose_.pose.position.z);

        // transition to new state
        current_sequencer_state_ = SequencerState::HOLD;

        // respond to request --> completed immediatly
        publishResponse(current_mission_ID_, msg->request, true, true);
      }
      else
      {
        ROS_WARN_STREAM("* amaze_mission_sequencer::request::HOLD - failed! Not in MISSION!");
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::RESUME: {
      ROS_INFO_STREAM("* amaze_mission_sequencer::request::RESUME...");
      // check if change is approved
      if (checkStateChange(previous_sequencer_state_))
      {
        ROS_INFO_STREAM("Resuming Mission");

        // transition to new state
        current_sequencer_state_ = previous_sequencer_state_;
        previous_sequencer_state_ = SequencerState::HOLD;

        // respond to request --> completed immediatly
        publishResponse(current_mission_ID_, msg->request, true, true);
      }
      else
      {
        if (b_do_verbose_)
        {
          ROS_WARN_STREAM("* amaze_mission_sequencer::request::RESUME - failed! Not in HOLD!");
        }
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::LAND: {
      ROS_DEBUG_STREAM("* amaze_mission_sequencer::request::LAND...");
      if (checkStateChange(SequencerState::LAND))
      {
        // execute landing
        b_executed_landing_ = executeLanding();

        // transition to new state
        current_sequencer_state_ = SequencerState::LAND;

        // respond to request --> completed upon landing
        publishResponse(current_mission_ID_, msg->request, true, false);
      }
      else
      {
        // wrong input - probably due to being in dis/armed or idle state
        ROS_WARN_STREAM("* amaze_mission_sequencer::request::LAND - failed! Not in above ground state.");
        publishResponse(current_mission_ID_, msg->request, false, false);
      }
      break;
    }

    case mission_sequencer::MissionRequest::HOVER: {
      ROS_DEBUG_STREAM("* amaze_mission_sequencer::request::HOVER...");
      if (checkStateChange(SequencerState::HOVER))
      {
        // set the hover pose to the current pose
        setpoint_vehicle_pose_ = current_vehicle_pose_;
        ROS_INFO_STREAM("Hover Position: "
                        << "  x = " << setpoint_vehicle_pose_.pose.position.x
                        << ", y = " << setpoint_vehicle_pose_.pose.position.y
                        << ", z = " << setpoint_vehicle_pose_.pose.position.z);

        // transition to new state
        current_sequencer_state_ = SequencerState::HOVER;

        // respond to request --> completed immediatly
        publishResponse(current_mission_ID_, msg->request, true, true);
      }
      break;
    }

    case mission_sequencer::MissionRequest::ABORT: {
      ROS_FATAL_STREAM("* amaze_mission_sequencer::request::ABORT...");
      ROS_INFO("Abort Mission - Landing");
      /// \todo TODO(scm): ABORT should disable usage of mission sequencer and put it into a locked state, that could
      /// only be reset by e.g. setting a dynamic parameter
      //      if (srv_mavros_land_.call(landCmd_))
      //      {
      //        if (landCmd_.response.success)
      //        {
      //          ROS_INFO("Landing");
      //          current_sequencer_state_ = LAND;
      //        }
      //      }
      //      current_sequencer_state_ = LAND;

      // respond to request --> completed immediatly
      publishResponse(current_mission_ID_, msg->request, true, true);
      break;
    }

    case mission_sequencer::MissionRequest::DISARM: {
      ROS_INFO_STREAM("* amaze_mission_sequencer::request::DISARM...");
      if (checkStateChange(SequencerState::DISARM))
      {
        // Preparation for arming
        disarmRequestTime_ = ros::Time::now();

        // transition to new state
        current_sequencer_state_ = SequencerState::DISARM;

        // Respond to request --> completed upon disarming
        publishResponse(current_mission_ID_, msg->request, true, false);
      }
      {
        // wrong input - probably due to being in dis/armed or idle state
        ROS_WARN_STREAM("* amaze_mission_sequencer::request::DISARM - failed! Not in safe armed state.");
        publishResponse(current_mission_ID_, msg->request, false, false);
      }
      break;
    }

    case mission_sequencer::MissionRequest::READ: {
      ROS_DEBUG_STREAM("* amaze_mission_sequencer::request::READ...");
      // transition to IDLE regardless of current state
      /// \todo TODO(scm): perform this also in PREARM state and create transition function

      if (current_sequencer_state_ == SequencerState::IDLE)
      {
        try
        {
          // Get filepaths
          if (!getFilenames())
          {
            // Respond that mission could not be loaded
            publishResponse(current_mission_ID_, int(msg->request), false, false);
            ROS_INFO_STREAM("CAN NOT READ MISSION(S)");
            return;
          }

          // Respond that mission has been loaded
          publishResponse(current_mission_ID_, int(msg->request), true, false);

          current_sequencer_state_ = SequencerState::PREARM;
        }
        catch (const std::exception& e)
        {
          // Respond that mission could not be loaded
          publishResponse(current_mission_ID_, int(msg->request), false, false);
          ROS_INFO_STREAM("CAN NOT READ MISSION(S) - Exception");
          std::cerr << e.what() << '\n';
        }
      }
      else
      {
        if (b_do_verbose_)
        {
          ROS_WARN_STREAM("* amaze_mission_sequencer::request::READ - failed! Not in IDLE nor!");
        }
        b_wrong_input = true;
      }
      break;
    }

    default:
      ROS_ERROR("REQUEST NOT DEFINED");
      b_wrong_input = true;
      break;
  }

  // respond if false input was provided
  if (b_wrong_input)
  {
    ROS_INFO_STREAM("WRONG REQUEST FOR CURRENT STATE: " << current_sequencer_state_);
    // Respond if input was wrong
    publishResponse(current_mission_ID_, msg->request, false, false);
  }
};

void MissionSequencer::cbWaypointFilename(const std_msgs::String::ConstPtr& msg)
{
  std::string fn = msg->data.c_str();
  bool res = setFilename(fn);
  if (b_do_verbose_)
  {
    ROS_INFO_STREAM("Received new waypoint_filename: " << fn << "; accepted:" << res);
  }
}

void MissionSequencer::cbWaypointList(const mission_sequencer::MissionWaypointArrayConstPtr& msg)
{
  std::vector<ParseWaypoint::Waypoint> new_waypoints = MSMsgConv::WaypointArray2WaypointList(msg->waypoints);
  if (new_waypoints.empty())
  {
    ROS_ERROR_STREAM("=> cbWaypointList: Could not add new waypoints as list is empty.");
    return;
  }

  // perform action as requested
  if (msg->action == mission_sequencer::MissionWaypointArray::CLEAR)
  {
    // clear waypoint list, then append
    waypointList_.clear();
    waypointList_ = std::move(new_waypoints);

    // reset WP reached
    b_wp_is_reached_ = false;
  }
  else
  {
    // otherwise the wp are added to list (depending on append or insert
    std::vector<ParseWaypoint::Waypoint>::iterator it_cur_wp = waypointList_.end();
    if (msg->action == mission_sequencer::MissionWaypointArray::INSERT)
    {
      ROS_WARN_STREAM("=> cbWaypointList: INSERT is not yet implemented, APPENDING waypoints");
      // todo modify iterator accordingly here
    }

    // "insert" vector
    waypointList_.insert(it_cur_wp, new_waypoints.begin(), new_waypoints.end());
  }

  // debug output
  ROS_DEBUG_STREAM("=> cbWaypointList:\n"
                   << "\tAdded " << new_waypoints.size() << " WPs\n"
                   << "\tHave  " << waypointList_.size() << " WPs");
}

void MissionSequencer::publishResponse(const uint8_t& id, const uint8_t& request, const bool& response,
                                       const bool& completed) const
{
  mission_sequencer::MissionResponse msg;

  // TODO(cbo): add request topics part
  msg.header = std_msgs::Header();
  msg.header.stamp = ros::Time::now();
  msg.request.id = id;
  msg.request.request = request;
  msg.response = response;
  msg.completed = completed;

  pub_ms_response_.publish(msg);
};

geometry_msgs::PoseStamped MissionSequencer::waypointToPoseStamped(const ParseWaypoint::Waypoint& waypoint)
{
  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = waypoint.z;

  tf2::Quaternion waypointQuaternion;
  waypointQuaternion.setRotation(tf2::Vector3(0, 0, 1), waypoint.yaw * DEG_TO_RAD);
  waypointQuaternion.normalize();
  if (b_wp_are_relativ_)
  {
    tf2::Quaternion startingQuaternion(
        starting_vehicle_pose_.pose.orientation.x, starting_vehicle_pose_.pose.orientation.y,
        starting_vehicle_pose_.pose.orientation.z, starting_vehicle_pose_.pose.orientation.w);

    waypointQuaternion = startingQuaternion * waypointQuaternion;

    double startingYaw, startingPitch, startingRoll;
    tf2::Matrix3x3(startingQuaternion).getEulerYPR(startingYaw, startingPitch, startingRoll);

    pose.pose.position.x =
        (waypoint.x * cos(startingYaw) - waypoint.y * sin(startingYaw)) + starting_vehicle_pose_.pose.position.x;
    pose.pose.position.y =
        (waypoint.x * sin(startingYaw) + waypoint.y * cos(startingYaw)) + starting_vehicle_pose_.pose.position.y;
    pose.pose.position.z = waypoint.z + starting_vehicle_pose_.pose.position.z;
  }

  pose.pose.orientation.x = waypointQuaternion[0];
  pose.pose.orientation.y = waypointQuaternion[1];
  pose.pose.orientation.z = waypointQuaternion[2];
  pose.pose.orientation.w = waypointQuaternion[3];

  return pose;
};

void MissionSequencer::logic(void)
{
  switch (current_sequencer_state_)
  {
    case SequencerState::IDLE:
      performIdle();
      return;

    case SequencerState::PREARM:
      if (b_do_verbose_)
      {
        ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::PREARM");
      }
      return;

    case SequencerState::ARM:
      performArming();
      break;

    case SequencerState::TAKEOFF:
      performTakeoff();
      break;

    case SequencerState::HOVER:
      performHover();
      break;

    case SequencerState::MISSION:
      performMission();
      break;

    case SequencerState::LAND:
      performLand();
      break;

    case SequencerState::DISARM:
      performDisarming();
      break;

    case SequencerState::HOLD:
      performHold();
      break;
  }
};

void MissionSequencer::publishPoseSetpoint(void)
{
  if (current_vehicle_state_.connected && b_pose_is_valid_)
  {
    setpoint_vehicle_pose_.header = std_msgs::Header();
    setpoint_vehicle_pose_.header.stamp = ros::Time::now();
    pub_pose_setpoint_.publish(setpoint_vehicle_pose_);
  }
};

void MissionSequencer::performIdle()
{
  if (b_do_verbose_)
  {
    ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::IDLE");
  }
}

void MissionSequencer::performArming()
{
  if (b_do_verbose_)
  {
    ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::ARM");
  }

  // check if we are already armed
  if (!current_vehicle_state_.armed)
  {
    // arm the vehicle
    if (current_vehicle_state_.mode != "OFFBOARD" && (ros::Time::now().toSec() - offboardRequestTime_.toSec() > 2.5))
    {
      if (srv_mavros_set_mode_.call(offboardMode_) && offboardMode_.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      offboardRequestTime_ = ros::Time::now();
    }
    else
    {
      if (!current_vehicle_state_.armed && (ros::Time::now().toSec() - armRequestTime_.toSec() > 2.5))
      {
        if (srv_mavros_arm_.call(armCmd_) && armCmd_.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        armRequestTime_ = ros::Time::now();
      }
    }
  }
  else
  {
    if (b_do_auto_state_change_)
    {
      ROS_INFO("Starting Mission");
      ROS_INFO("Taking off");
      // Publish response of start
      current_sequencer_state_ = SequencerState::MISSION;
      b_wp_is_reached_ = false;
      b_is_landed_ = true;
    }
  }
}

void MissionSequencer::performTakeoff()
{
  ROS_DEBUG_STREAM_THROTTLE(dbg_throttle_rate_, "* SequencerState::TAKEOFF");

  // check for takeoff type
  if (sequencer_params_.takeoff_type_ == TakeoffType::POSITION)
  {
    // check if takeoff position has been reached
    if (checkWaypoint(setpoint_takeoff_pose_))
    {
      // setpoint reached --> go into hover mode
      ROS_INFO_STREAM("==> TAKEOFF completed");
      current_sequencer_state_ = SequencerState::HOVER;
    }
  }
}

void MissionSequencer::performMission()
{
  // check if we still have waypoints in the list
  if (waypointList_.size() > 0)
  {
    // check if waypoint has been reached
    if (!b_wp_is_reached_ && checkWaypoint(waypointToPoseStamped(waypointList_[0])))
    {
      // set waypoint reached and reset timer
      b_wp_is_reached_ = true;
      time_last_wp_reached_ = ros::Time::now();

      // transition to hover state at waypoint
      current_sequencer_state_ = SequencerState::HOVER;
    }

    /// \todo maybe automatically go to hover here, and continue if time has been exceeded
    // check if holdtime was exceeded
    //    if (b_wp_is_reached_ && (ros::Time::now().toSec() - time_last_wp_reached_.toSec()) >
    //    waypointList_[0].holdtime)
    //    {
    //      ROS_INFO_STREAM("Waited for: " << waypointList_[0].holdtime << " Seconds");
    //      waypointList_.erase(waypointList_.begin());
    //      // FIX(scm): make sure the list is not empty!!!
    //      if (waypointList_.size() != 0)
    //      {
    //        setpoint_vehicle_pose_ = waypointToPoseStamped(waypointList_[0]);
    //        b_wp_is_reached_ = false;
    //      }
    //    }
  }
  else
  {
    ROS_DEBUG_STREAM_THROTTLE(dbg_throttle_rate_, "* No more waypoints to follow...");
    // check for automatic transition to land state
    if (b_do_automatically_land_)
    {
    }

    // publish mission completion
    publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::UNDEF, false, true);

    //    if (srv_mavros_land_.call(landCmd_) || b_do_automatically_land_)
    //    {
    //      if (landCmd_.response.success)
    //      {
    //        ROS_INFO("Landing");
    //        current_sequencer_state_ = SequencerState::LAND;

    //        // Respond that mission succefully finished
    //        publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::UNDEF, false, true);
    //      }
    //    }
  }

  //  if (b_do_verbose_)
  //  {
  ROS_DEBUG_STREAM_THROTTLE(dbg_throttle_rate_,
                            "* currentFollowerState__::MISSION; waypoints left: " << waypointList_.size());
  //  }
}

void MissionSequencer::performHover()
{
  // check size of waypoints list
  if (!waypointList_.empty())
  {
    bool b_transition_to_mission = true;
    // check if the goal is to hold at waypoint
    if (b_wp_is_reached_)
    {
      // holdtime check
      if ((ros::Time::now().toSec() - time_last_wp_reached_.toSec()) > waypointList_[0].holdtime)
      {
        // waypoint has been completed, delete from list
        ROS_INFO_STREAM("Waited for: " << waypointList_[0].holdtime << " Seconds");
        waypointList_.erase(waypointList_.begin());

        // FIX(scm): make sure the list is not empty!!!
        if (waypointList_.size() != 0)
        {
          setpoint_vehicle_pose_ = waypointToPoseStamped(waypointList_[0]);
          b_wp_is_reached_ = false;
        }
        else
        {
          // no more waypoints
          b_transition_to_mission = false;

          // publish mission completion
          publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::UNDEF, false, true);

          // check if automatically land
          if (b_do_automatically_land_)
          {
            // transition to new state
            current_sequencer_state_ = SequencerState::LAND;
          }
        }
      }
      else
      {
        // continue to hover
        b_transition_to_mission = false;
      }
    }

    // check if change to mission state is necessary
    if (b_transition_to_mission)
    {
      current_sequencer_state_ = SequencerState::MISSION;
    }
  }
  else
  {
    // no more waypoints --> hover at current setpoint
    ROS_DEBUG_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::HOVER; hovering until new waypoint "
                                                  "arrives...");
  }
}

void MissionSequencer::performLand()
{
  // call landing if not executed yet correctly
  if (!b_executed_landing_)
    b_executed_landing_ = executeLanding();

  // mavros check to see if landed
  if (current_vehicle_ext_state_.landed_state == current_vehicle_ext_state_.LANDED_STATE_ON_GROUND)
  {
    // set landed states
    b_is_landed_ = true;

    // check if vehicle is armed
    if (!current_vehicle_state_.armed)
    {
      // not armed --> go into IDLE
      ROS_DEBUG_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::LAND->LANDED --> set state to "
                                                    "IDLE");

      // transition to new state
      current_sequencer_state_ = SequencerState::IDLE;
    }
    else
    {
      // still armed
      ROS_DEBUG_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::LAND->LANDED --> Vehicle still "
                                                    "ARMED");

      // perform auto disarm
      if (b_do_automatically_disarm_)
      {
        ROS_INFO_STREAM("* performLand(): automaticall disarming vehicle");

        // transition to disarm state
        current_sequencer_state_ = SequencerState::DISARM;
      }
    }
  }
}

void MissionSequencer::performHold()
{
}

void MissionSequencer::performDisarming()
{
  // check if vehicle is disarmed
  bool is_disarmed = true;
  if (current_vehicle_state_.armed)
  {
    // vehicle is currently armed, issue disarm command
    is_disarmed = false;
    if (srv_mavros_disarm_.call(mavros_cmds_.disarm_cmd_))
    {
      if (mavros_cmds_.disarm_cmd_.response.success)
      {
        is_disarmed = true;
      }
    }
  }

  // at this stage it is disarmed
  if (is_disarmed)
  {
    ROS_INFO("Disarmed");

    // transition to IDLE or PREARM state depending on stuff
    /// \todo TODO(scm): make this part a function called transitionToIDLE/PREARM which checks the wyapointlist etc.
    filenames_.erase(filenames_.begin());
    waypointList_.clear();
    if (filenames_.size() == 0)
    {
      current_sequencer_state_ = SequencerState::IDLE;
    }
    else
    {
      current_sequencer_state_ = SequencerState::PREARM;
    }
  }
}

void MissionSequencer::performAbort()
{
}

bool MissionSequencer::checkWaypoint(const geometry_msgs::PoseStamped& current_waypoint)
{
  // set the current setpoint
  setpoint_vehicle_pose_ = current_waypoint;

  double diff_position, diff_yaw;
  // calculate position difference
  double diff_squared =
      std::pow(std::fabs(current_vehicle_pose_.pose.position.x - current_waypoint.pose.position.x), 2) +
      std::pow(std::fabs(current_vehicle_pose_.pose.position.y - current_waypoint.pose.position.y), 2) +
      std::pow(std::fabs(current_vehicle_pose_.pose.position.z - current_waypoint.pose.position.z), 2);
  diff_position = std::sqrt(diff_squared);
  ROS_DEBUG_STREAM("-   diff_position: " << diff_position);

  // claculate yaw difference
  diff_yaw = std::fabs(
      2.0 *
      double(tf2::Quaternion(current_vehicle_pose_.pose.orientation.x, current_vehicle_pose_.pose.orientation.y,
                             current_vehicle_pose_.pose.orientation.z, current_vehicle_pose_.pose.orientation.w)
                 .angle(tf2::Quaternion(current_waypoint.pose.orientation.x, current_waypoint.pose.orientation.y,
                                        current_waypoint.pose.orientation.z, current_waypoint.pose.orientation.w))));
  diff_yaw = std::fmod(diff_yaw, 2 * M_PI);
  if (diff_yaw > M_PI)
  {
    diff_yaw -= 2 * M_PI;
  }
  ROS_DEBUG_STREAM("-   diff_yaw:      " << diff_yaw);

  // check if waypoint has been reached

  if (diff_position < sequencer_params_.threshold_position_ && std::fabs(diff_yaw) < sequencer_params_.threshold_yaw_)
  {
    ROS_INFO_STREAM("Reached Waypoint: x = " << waypointList_[0].x << ", y = " << waypointList_[0].y
                                             << ", z = " << waypointList_[0].z << ", yaw = " << waypointList_[0].yaw);
    return true;
  }

  // waypoint not reached --> return fals
  return false;
}

bool MissionSequencer::checkStateChange(const SequencerState new_state) const
{
  switch (current_sequencer_state_)
  {
    case SequencerState::IDLE:
      // IDLE -> request(ARM) --> PREARM/ARM
      if (new_state == SequencerState::ARM)
        return true;
      break;
    case SequencerState::ARM:
      // ARM -> request(TAKEOFF) --> TAKEOFF
      // ARM -> request(DISARM) --> DISARM
      if (new_state == SequencerState::TAKEOFF)
        return true;
      else if (new_state == SequencerState::DISARM)
        return true;
      break;

    case SequencerState::MISSION:
      // MISSION -> request(HOLD) --> HOLD
      // MISSION -> request(LAND) --> LAND
      // MISSION -> request(HOVER) --> HOVER
      if (new_state == SequencerState::HOLD)
        return true;
      else if (new_state == SequencerState::LAND)
        return true;
      else if (new_state == SequencerState::HOVER)
        return true;
      break;

    case SequencerState::HOLD:
      // HOLD -> request(RESUME) -> prevstate in MISSION, TAKEOFF, LAND, HOVER --> PREV_STATE
      // HOLD -> request(LAND) -> LAND
      if (new_state == SequencerState::LAND)
        return true;
      else if (new_state == SequencerState::MISSION)
        return true;
      else if (new_state == SequencerState::HOVER)
        return true;
      else if (new_state == SequencerState::TAKEOFF)
        return true;
      break;

    case SequencerState::LAND:
      // LAND -> request(DISARM) -> DISARM
      if (new_state == SequencerState::DISARM)
        return true;
      else if (new_state == SequencerState::HOLD)
        return true;
      break;
  }

  // state change was not approved
  ROS_ERROR_STREAM("=> No known state transition from " << current_sequencer_state_ << " to " << new_state);
  ROS_ERROR_STREAM("   Staying in " << current_sequencer_state_);
  return false;
}

bool MissionSequencer::executeLanding()
{
  if (srv_mavros_land_.call(landCmd_))
  {
    if (landCmd_.response.success)
    {
      ROS_INFO("--> Landing");
      return true;
    }
  }

  return false;
}

bool MissionSequencer::checkMissionID(const uint8_t& mission_id, const uint8_t& request_id) const
{
  // check mission ID
  if (current_mission_ID_ != mission_id)
  {
    // WRONG ID
    // check if request was to read mission files (only accepted in IDLE or PREARM states
    if (request_id == mission_sequencer::MissionRequest::READ &&
        (current_sequencer_state_ == SequencerState::IDLE || current_sequencer_state_ == SequencerState::PREARM))
    {
      return true;
    }

    return false;
  }

  // same ID
  return true;
}

}  // namespace mission_sequencer
