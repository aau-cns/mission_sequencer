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

#include "mission_sequencer.hpp"

#include "utils/message_conversion.hpp"
#include "utils/parser_ros.hpp"
#include "utils/parser_waypoints.hpp"

namespace std
{
///
/// \see https://stackoverflow.com/a/4609795
///
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}
}  // namespace std

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
  : nh_(nh)
  , pnh_(pnh)
  , current_sequencer_state_{ SequencerState::IDLE }
  , previous_sequencer_state_{ SequencerState::IDLE }
{
  // setup parameters
  sequencer_params_ = parse_ros_nodehandle(pnh_);

  // setup navigation variables
  current_vehicle_state_ = mavros_msgs::State();
  current_vehicle_ext_state_ = mavros_msgs::ExtendedState();
  current_vehicle_pose_ = geometry_msgs::PoseStamped();
  starting_vehicle_pose_ = geometry_msgs::PoseStamped();
  setpoint_vehicle_pose_ = geometry_msgs::PoseStamped();

  waypoint_list_ = std::vector<Waypoint>(0);
  time_last_wp_reached_ = ros::Time::now();

  // TODO(scm): maybe make the mission_id uninitialized just in case
  current_mission_ID_ = 0;

  // setup communication variables
  mavros_cmds_ = MissionSequencer::MavrosCommands();

  filenames_ = std::vector<std::string>(0);

  // setup ROS subscribers
  //  sub_vehicle_state_ = nh.subscribe("/mavros/state", 10, &MissionSequencer::cbVehicleState, this);
  //  sub_extended_vehicle_state_ =
  //      nh.subscribe("/mavros/extended_state", 10, &MissionSequencer::cbExtendedVehicleState, this);
  //  sub_vehicle_pose_ = nh.subscribe("/mavros/local_position/pose", 10, &MissionSequencer::cbPose, this);
  //  sub_ms_request_ = nh.subscribe("/autonomy/request", 10, &MissionSequencer::cbMSRequest, this);

  // setup ROS subscribers (relative to node's namespace)
  sub_vehicle_state_ = nh_.subscribe("mavros/state", 10, &MissionSequencer::cbVehicleState, this);
  sub_extended_vehicle_state_ =
      nh_.subscribe("mavros/extended_state", 10, &MissionSequencer::cbExtendedVehicleState, this);
  sub_vehicle_pose_ = nh_.subscribe(sequencer_params_.topic_ref_pose_, 1, &MissionSequencer::cbPose, this);
  sub_vehicle_odom_ = nh_.subscribe(sequencer_params_.topic_ref_odom_, 1, &MissionSequencer::cbOdom, this,
                                    ros::TransportHints().tcpNoDelay(true));
  sub_ms_request_ = nh_.subscribe("autonomy/request", 10, &MissionSequencer::cbMSRequest, this);
  sub_waypoint_file_name_ = pnh_.subscribe("waypoint_filename", 10, &MissionSequencer::cbWaypointFilename, this);
  sub_waypoint_list_ = pnh_.subscribe("waypoint_list", 10, &MissionSequencer::cbWaypointList, this);

  // setup ROS publishers (relative to node's namespace)
  pub_pose_setpoint_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  pub_ms_response_ = nh_.advertise<mission_sequencer::MissionResponse>("autonomy/response", 10);
  pub_waypoint_list_ = nh_.advertise<mission_sequencer::MissionWaypointArray>("get_waypoint_list", 1);
  pub_waypoint_reached_ = nh_.advertise<mission_sequencer::MissionWaypointStamped>("waypoint_reached", 1);

  // setup ROS services (relative to node's namespace)
  srv_mavros_arm_ = nh_.serviceClient<mavros_msgs::CommandBool>(sequencer_params_.srv_cmd_arming_);
  srv_mavros_disarm_ = nh_.serviceClient<mavros_msgs::CommandLong>(sequencer_params_.srv_cmd_command_);
  srv_mavros_land_ = nh_.serviceClient<mavros_msgs::CommandTOL>(sequencer_params_.srv_cmd_land_);
  srv_mavros_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>(sequencer_params_.srv_cmd_set_mode_);

  // setup ROS service servers
  srv_get_start_pose_ = pnh_.advertiseService("getStartPose", &MissionSequencer::srvGetStartPose, this);

  // set waypoints, if required
  if (sequencer_params_.b_wp_from_file_)
    // set waypoint_fn_
    if (setWaypointFilename(sequencer_params_.filename_wps_))
    {
      // load waypoint_fn_ filename into filenames_
      reloadFilenames();
    }
}

MissionSequencer::~MissionSequencer()
{
}

void MissionSequencer::cbVehicleState(const mavros_msgs::State::ConstPtr& msg)
{
  b_state_is_valid_ = true;
  current_vehicle_state_ = *msg;

  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_,
                            " * msg received: armed " << (int)current_vehicle_state_.armed);
}

void MissionSequencer::cbExtendedVehicleState(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  b_extstate_is_valid_ = true;
  current_vehicle_ext_state_ = *msg;

  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_,
                            " * msg received: landed state " << (int)current_vehicle_ext_state_.landed_state);
}

void MissionSequencer::cbPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // call update pose
  updatePose(*msg);
};

void MissionSequencer::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  // update pose
  geometry_msgs::PoseStamped pose;
  pose.pose = msg->pose.pose;
  pose.header = msg->header;
  updatePose(pose);

  // update velocity
  geometry_msgs::TwistStamped twist;
  twist.twist = msg->twist.twist;
  twist.header = msg->header;

  // update current vehicle velocity
  current_vehicle_twist_ = twist;

  // update validity
  b_odom_is_valid_ = true;
}

void MissionSequencer::updatePose(const geometry_msgs::PoseStamped& pose)
{
  // check if starting position has been determined, i.e. the current pose is valid and MS is in IDLE

  if (!b_pose_is_valid_ || (current_sequencer_state_ == SequencerState::IDLE))
  {
    /// \todo make this to Eigen variable
    starting_vehicle_pose_ = pose;

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
      ROS_INFO_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_,
                               "* Initial (PX4/NED) yaw= " << startingYaw * RAD_TO_DEG
                                                           << "[deg], (OptiTrack/ENU) pos x="
                                                           << starting_vehicle_pose_.pose.position.x
                                                           << " pos y=" << starting_vehicle_pose_.pose.position.y
                                                           << " pos z=" << starting_vehicle_pose_.pose.position.z);
    }

    mavros_cmds_.land_cmd_.request.yaw = startingYaw * RAD_TO_DEG;
    mavros_cmds_.land_cmd_.request.altitude = starting_vehicle_pose_.pose.position.z;

    // set the current goal to the current pose
    setpoint_vehicle_pose_ = starting_vehicle_pose_;
    b_pose_is_valid_ = true;
  }

  // update the current vehicle pose
  current_vehicle_pose_ = pose;
}

bool MissionSequencer::reloadFilenames()
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
      ROS_WARN_STREAM("MissionSequencer::reloadFilenames(): failure! Could not get file paths for CSV waypoint files: "
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

bool MissionSequencer::parseFilename()
{
  if (filenames_.size() != 0)
  {
    // Take first entry of filename list
    std::string filename = filenames_[0];

    std::vector<std::string> header_default = { "x", "y", "z", "yaw", "holdtime" };
    std::shared_ptr<WaypointParser> waypoint_parser = std::make_shared<WaypointParser>(filename, header_default);

    // Parse waypoint file
    waypoint_parser->readParseCsv();

    // Get the data
    waypoint_list_ = waypoint_parser->getData();
    return true;
  }
  return false;
}

bool MissionSequencer::setWaypointFilename(const std::string waypoint_fn)
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
      return true;
    }
  }

  return false;
};

void MissionSequencer::cbMSRequest(const mission_sequencer::MissionRequest::ConstPtr& msg)
{
  bool b_wrong_input = false;

  // check mission id
  if (!checkMissionID(msg->id, msg->request))
  {
    // WRONG ID
    // check if the request is to read mission files
    ROS_INFO_STREAM("WRONG MISSION ID FOR CURRENT STATE: " << current_sequencer_state_ << "; Local mission ID: "
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
      ROS_INFO_STREAM("* mission_sequencer::request::ARM...");
      if (checkStateChange(SequencerState::ARM))
      {
        // set time for valid request
        time_last_valid_request_ = ros::Time::now().toSec();

        if (b_pose_is_valid_ && b_state_is_valid_ && b_extstate_is_valid_)
        {
          // Set initial pose
          starting_vehicle_pose_ = current_vehicle_pose_;
          setpoint_vehicle_pose_ = current_vehicle_pose_;

          // Preparation for arming
          mavros_cmds_.arm_cmd_.request.value = true;  // QUESTION(scm); is this actually needed?
          mavros_cmds_.time_arm_request = ros::Time::now();
          mavros_cmds_.time_offboard_request = ros::Time::now();
          current_sequencer_state_ = SequencerState::ARM;

          // Respond to request
          publishResponse(current_mission_ID_, msg->request, true, false);
        }
        else if (checkRequestTime())
        {
          // wait
          ROS_DEBUG_STREAM("* mission_sequencer::request::ARM - waiting for successful mavros communication");
        }
        else
        {
          ROS_ERROR_STREAM("* mission_sequencer::request::ARM - failed! mavros communication or PREARM error!");
          ROS_WARN_STREAM("*   Sanity checks: Pose=" << b_pose_is_valid_ << ", State=" << b_state_is_valid_
                                                     << ", extState=" << b_extstate_is_valid_);

          b_wrong_input = true;
        }
      }
      else
      {
        ROS_ERROR_STREAM("* mission_sequencer::request::ARM - failed! State change not allowed.");
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::TAKEOFF: {
      ROS_DEBUG_STREAM("* mission_sequencer::request::TAKEOFF...");
      // check if change is approved
      /// \todo TODO(scm): if enum conversion from string exists (or int) then this can be put outside the switch
      if (checkStateChange(SequencerState::TAKEOFF))
      {
        // set time for valid request
        time_last_valid_request_ = ros::Time::now().toSec();

        if (current_vehicle_state_.armed)
        {
          // state change to TAKEOFF is approved
          if (sequencer_params_.takeoff_type_ == TakeoffType::POSITION)
          {
            // set starting position if they are relative
            if (sequencer_params_.b_wp_are_relative_)
            {
              starting_vehicle_pose_ = current_vehicle_pose_;
            }
            ROS_DEBUG_STREAM("*   starting_pose: " << starting_vehicle_pose_);

            // set takeoff position
            setpoint_takeoff_pose_ = starting_vehicle_pose_;
            setpoint_takeoff_pose_.pose.position.z += sequencer_params_.takeoff_z_;

            // set waypoint reached to false and transition to new state
            b_wp_is_reached_ = false;
            b_is_landed_ = false;
            current_sequencer_state_ = SequencerState::TAKEOFF;

            // Respond to request
            publishResponse(current_mission_ID_, msg->request, true, false);
          }
          else
          {
            ROS_FATAL_STREAM("=> TakeoffType not implemented");
          }
        }
        else if (checkRequestTime())
        {
          // wait
          ROS_DEBUG_STREAM("* mission_sequencer::request::TAKEOFF - waiting for successful arming");
        }
        else
        {
          ROS_ERROR_STREAM("* mission_sequencer::request::TAKEOFF - failed! Probably not armed.");
          b_wrong_input = true;
        }
      }
      else
      {
        ROS_ERROR_STREAM("* mission_sequencer::request::TAKEOFF - failed! State change not allowed.");
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::HOLD: {
      ROS_WARN_STREAM("* mission_sequencer::request::HOLD...");
      // check if change is approved
      if (checkStateChange(SequencerState::HOLD))
      {
        // set time for valid request
        time_last_valid_request_ = ros::Time::now().toSec();

        // save previous state
        previous_sequencer_state_ = current_sequencer_state_;
        ROS_DEBUG_STREAM("- prehold state: " << previous_sequencer_state_);

        // set holding position
        setpoint_vehicle_pose_ = current_vehicle_pose_;
        // predict waypoint based on current velocity
        if (sequencer_params_.b_predict_hold_wp_ && b_odom_is_valid_)
        {
          // calculate position with max decelaration of -1.0m/s^2 per axis
          setpoint_vehicle_pose_.pose.position.x +=
              0.5 * current_vehicle_twist_.twist.linear.x * std::fabs(current_vehicle_twist_.twist.linear.x) / 1.0;
          setpoint_vehicle_pose_.pose.position.y +=
              0.5 * current_vehicle_twist_.twist.linear.y * std::fabs(current_vehicle_twist_.twist.linear.y) / 1.0;
          setpoint_vehicle_pose_.pose.position.z +=
              0.5 * current_vehicle_twist_.twist.linear.z * std::fabs(current_vehicle_twist_.twist.linear.z) / 1.0;
        }
        ROS_INFO_STREAM("Hold Position: "
                        << "  x = " << setpoint_vehicle_pose_.pose.position.x
                        << ", y = " << setpoint_vehicle_pose_.pose.position.y
                        << ", z = " << setpoint_vehicle_pose_.pose.position.z);

        // updating velocity reached
        current_vel_reached_[0] = false;
        current_vel_reached_[1] = false;
        current_vel_reached_[2] = false;

        // transition to new state
        current_sequencer_state_ = SequencerState::HOLD;

        // respond to request --> completed immediatly
        publishResponse(current_mission_ID_, msg->request, true, true);
      }
      else
      {
        ROS_ERROR_STREAM("* mission_sequencer::request::HOLD - failed! Not in MISSION!");
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::RESUME: {
      ROS_INFO_STREAM("* mission_sequencer::request::RESUME...");
      // check if change is approved
      if (checkStateChange(SequencerState::RESUME))
      {
        // set time for valid request
        time_last_valid_request_ = ros::Time::now().toSec();

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
          ROS_ERROR_STREAM("* mission_sequencer::request::RESUME - failed! Not in HOLD!");
        }
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::LAND: {
      ROS_DEBUG_STREAM("* mission_sequencer::request::LAND...");
      if (checkStateChange(SequencerState::LAND))
      {
        // set time for valid request
        time_last_valid_request_ = ros::Time::now().toSec();

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
        ROS_ERROR_STREAM("* mission_sequencer::request::LAND - failed! Not in above ground state.");
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::HOVER: {
      ROS_DEBUG_STREAM("* mission_sequencer::request::HOVER...");
      if (checkStateChange(SequencerState::HOVER))
      {
        // set time for valid request
        time_last_valid_request_ = ros::Time::now().toSec();

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
      else
      {
        ROS_ERROR_STREAM("* mission_sequencer::request::HOVER - failed!");
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::ABORT: {
      if (current_sequencer_state_ == SequencerState::PREARM)
      {
        if (checkStateChange(SequencerState::IDLE))
        {
          // set time for valid request
          time_last_valid_request_ = ros::Time::now().toSec();

          ROS_WARN_STREAM("=> mission_sequencer: Abort Mission - discard loaded waypoints!");
          current_sequencer_state_ = SequencerState::IDLE;

          // Respond that loaded waypoints have been discarded
          publishResponse(current_mission_ID_, int(msg->request), true, false);
        }
      }
      else if (current_sequencer_state_ == SequencerState::IDLE)
      {
        publishResponse(current_mission_ID_, int(msg->request), true, false);
      }
      else
      {
        ROS_FATAL_STREAM("* mission_sequencer::request::ABORT...");
        ROS_WARN_STREAM("=> mission_sequencer: Abort Mission - NOT IMPLEMENTED");
        b_wrong_input = true;

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
      }
      break;
    }

    case mission_sequencer::MissionRequest::DISARM: {
      ROS_INFO_STREAM("* mission_sequencer::request::DISARM...");
      if (checkStateChange(SequencerState::DISARM))
      {
        // set time for valid request
        time_last_valid_request_ = ros::Time::now().toSec();

        // Preparation for arming
        mavros_cmds_.time_disarm_request = ros::Time::now();

        // transition to new state
        current_sequencer_state_ = SequencerState::DISARM;

        // Respond to request --> completed upon disarming
        publishResponse(current_mission_ID_, msg->request, true, false);
      }
      else
      {
        // wrong input - probably due to being in dis/armed or idle state
        ROS_WARN_STREAM("* mission_sequencer::request::DISARM - failed! Not in safe armed state.");
        b_wrong_input = true;
      }
      break;
    }

    case mission_sequencer::MissionRequest::READ: {
      // READ means: reload a list of CSV filenames with waypoints and go to PREARM
      ROS_DEBUG_STREAM("* mission_sequencer::request::READ...");

      // if (checkStateChange(SequencerState::PREARM) || checkStateChange(SequencerState::MISSION))
      if (checkStateChange(SequencerState::PREARM))
      {
        try
        {
          // Get filepaths
          if (!reloadFilenames())
          {
            // Respond that mission could not be loaded
            publishResponse(current_mission_ID_, int(msg->request), false, false);
            ROS_WARN_STREAM("* mission_sequencer::request::READ: CAN NOT SET WAYPOINT FILE NAMES FOR MISSION(S)");
            return;
          }

          // Respond that mission has been loaded
          publishResponse(current_mission_ID_, int(msg->request), true, false);

          // clear waypoints before going to PREARM to parse new waypoints:
          waypoint_list_.clear();
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
          ROS_WARN_STREAM("* mission_sequencer::request::READ - failed! Not in IDLE!");
        }
        b_wrong_input = true;
      }
      break;
    }

    default:
      ROS_ERROR_STREAM("=> mission_sequencer: REQUEST NOT DEFINED! (msg->request=" << msg->request << ")");
      b_wrong_input = true;
      break;
  }

  // respond if false input was provided
  if (b_wrong_input)
  {
    ROS_ERROR_STREAM("=> mission_sequencer: ISSUE WITH REQUEST FOR CURRENT STATE: "
                     << current_sequencer_state_ << ", (msg->request=" << msg->request << ")");
    // Respond if input was wrong
    publishResponse(current_mission_ID_, msg->request, false, false);
  }

  // publish any message now to let the autonomy know if request is accepted
  ros::spinOnce();
};

void MissionSequencer::cbWaypointFilename(const std_msgs::String::ConstPtr& msg)
{
  std::string fn = msg->data.c_str();
  bool res = setWaypointFilename(fn);
  if (b_do_verbose_)
  {
    ROS_INFO_STREAM("Received new waypoint_filename: " << fn << "; accepted:" << res);
  }
}

void MissionSequencer::cbWaypointList(const mission_sequencer::MissionWaypointArrayConstPtr& msg)
{
  // parse waypoints and check if all of them are global
  std::vector<Waypoint> new_waypoints;
  /// \deprecated msg->is_global will be removed in future releases
  if (msg->is_global)
    new_waypoints = MSMsgConv::WaypointArray2WaypointList(msg->waypoints, Waypoint::ReferenceFrame::GLOBAL);
  else
    new_waypoints =
        MSMsgConv::WaypointArray2WaypointList(msg->waypoints, static_cast<Waypoint::ReferenceFrame>(msg->reference));

  // check if waypoints were provided
  if (new_waypoints.empty())
  {
    ROS_ERROR_STREAM("=> cbWaypointList: Could not add new waypoints as list is empty.");
    return;
  }

  // perform action as requested
  if (msg->action == mission_sequencer::MissionWaypointArray::CLEAR)
  {
    // clear waypoint list, then append
    waypoint_list_.clear();
    waypoint_list_ = std::move(new_waypoints);

    // reset WP reached
    b_wp_is_reached_ = false;
  }
  else
  {
    // otherwise the wp are added to list (depending on append or insert
    std::vector<Waypoint>::iterator it_cur_wp = waypoint_list_.end();
    if (msg->action == mission_sequencer::MissionWaypointArray::INSERT)
    {
      //      ROS_WARN_STREAM("=> cbWaypointList: INSERT is not yet implemented, APPENDING waypoints");
      // set iterator to idx or end if idx>size
      it_cur_wp = msg->idx < waypoint_list_.size() ? waypoint_list_.begin() + msg->idx : waypoint_list_.end();
    }

    // "insert" vector
    waypoint_list_.insert(it_cur_wp, new_waypoints.begin(), new_waypoints.end());
  }

  // debug output
  ROS_DEBUG_STREAM("=> cbWaypointList:\n"
                   << "\tAdded " << new_waypoints.size() << " WPs\n"
                   << "\tHave  " << waypoint_list_.size() << " WPs");
}

bool MissionSequencer::srvGetStartPose(mission_sequencer::GetStartPose::Request& /*req*/,
                                       mission_sequencer::GetStartPose::Response& res)
{
  // check if we have received a valid pose yet
  /// \todo TODO(scm): maybe also check if we are at least armed, because otherwise we do not have a 'valid' starting
  /// pose
  if (!b_pose_is_valid_)
    return false;

  // set message header
  res.header.stamp = mavros_cmds_.time_arm_request;
  res.header.frame_id = "global";

  /// \todo TODO(scm): simplify variable type usage for poses, and add parser!
  if (sequencer_params_.b_wp_are_relative_)
  {
    res.start_wp.x = starting_vehicle_pose_.pose.position.x;
    res.start_wp.y = starting_vehicle_pose_.pose.position.y;
    res.start_wp.z = starting_vehicle_pose_.pose.position.z;

    // derive starting quaternion
    tf2::Quaternion startingQuaternion(
        starting_vehicle_pose_.pose.orientation.x, starting_vehicle_pose_.pose.orientation.y,
        starting_vehicle_pose_.pose.orientation.z, starting_vehicle_pose_.pose.orientation.w);

    double startingYaw, startingPitch, startingRoll;
    tf2::Matrix3x3(startingQuaternion).getEulerYPR(startingYaw, startingPitch, startingRoll);

    res.start_wp.yaw = startingYaw;
  }
  else
  {
    res.start_wp.x = 0;
    res.start_wp.y = 0;
    res.start_wp.z = 0;
    res.start_wp.yaw = 0;
  }

  return true;
}

void MissionSequencer::publishResponse(const uint8_t& id, const uint8_t& request, const bool& response,
                                       const bool& completed) const
{
  mission_sequencer::MissionResponse msg;

  msg.header = std_msgs::Header();
  msg.header.stamp = ros::Time::now();
  msg.request.id = id;
  msg.request.request = request;
  msg.response = response;
  msg.completed = completed;

  pub_ms_response_.publish(msg);
};

geometry_msgs::PoseStamped MissionSequencer::waypointToPoseStamped(const Waypoint& waypoint)
{
  // generate pose from waypoint
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = waypoint.z;

  tf2::Quaternion waypointQuaternion;
  waypointQuaternion.setRotation(tf2::Vector3(0, 0, 1), waypoint.yaw * DEG_TO_RAD);
  waypointQuaternion.normalize();

  // check for relative settings
  if (sequencer_params_.b_wp_are_relative_ || waypoint.ref_frame == Waypoint::ReferenceFrame::LOCAL ||
      waypoint.ref_frame == Waypoint::ReferenceFrame::CUR_POSE)
  {
    // check if waypoint is relative to current or starting pose
    if (waypoint.ref_frame >= Waypoint::ReferenceFrame::CUR_POS)
    {
      ROS_DEBUG_STREAM("Updating WP relative to CURRENT POSE");
      // relative to current pose
      tf2::Quaternion curentQuaternion(
          current_vehicle_pose_.pose.orientation.x, current_vehicle_pose_.pose.orientation.y,
          current_vehicle_pose_.pose.orientation.z, current_vehicle_pose_.pose.orientation.w);

      // get yaw
      double currentYaw, currentPitch, currentRoll;
      tf2::Matrix3x3(curentQuaternion).getEulerYPR(currentYaw, currentPitch, currentRoll);

      // update pose
      waypointQuaternion = curentQuaternion * waypointQuaternion;
      pose.pose.position.x =
          (waypoint.x * cos(currentYaw) - waypoint.y * sin(currentYaw)) + current_vehicle_pose_.pose.position.x;
      pose.pose.position.y =
          (waypoint.x * sin(currentYaw) + waypoint.y * cos(currentYaw)) + current_vehicle_pose_.pose.position.y;
      pose.pose.position.z = waypoint.z + current_vehicle_pose_.pose.position.z;
    }
    else
    {
      ROS_DEBUG_STREAM("Updating WP relative to STARTING POSE");
      // relative to starting pose
      tf2::Quaternion startingQuaternion(
          starting_vehicle_pose_.pose.orientation.x, starting_vehicle_pose_.pose.orientation.y,
          starting_vehicle_pose_.pose.orientation.z, starting_vehicle_pose_.pose.orientation.w);

      // get yaw
      double startingYaw, startingPitch, startingRoll;
      tf2::Matrix3x3(startingQuaternion).getEulerYPR(startingYaw, startingPitch, startingRoll);

      // update pose
      waypointQuaternion = startingQuaternion * waypointQuaternion;
      pose.pose.position.x =
          (waypoint.x * cos(startingYaw) - waypoint.y * sin(startingYaw)) + starting_vehicle_pose_.pose.position.x;
      pose.pose.position.y =
          (waypoint.x * sin(startingYaw) + waypoint.y * cos(startingYaw)) + starting_vehicle_pose_.pose.position.y;
      pose.pose.position.z = waypoint.z + starting_vehicle_pose_.pose.position.z;
    }
  }
  // check for relative position
  else if (waypoint.ref_frame == Waypoint::ReferenceFrame::CUR_POS)
  {
    ROS_DEBUG_STREAM("Updating WP relative to CURRENT POSITION");
    // update position
    pose.pose.position.x += current_vehicle_pose_.pose.position.x;
    pose.pose.position.y += current_vehicle_pose_.pose.position.y;
    pose.pose.position.z += current_vehicle_pose_.pose.position.z;
  }

  // update quaternion
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
      break;

    case SequencerState::PREARM:
      performPrearming();
      break;

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
    ros::Time pub_time = ros::Time::now();
    setpoint_vehicle_pose_.header = std_msgs::Header();
    setpoint_vehicle_pose_.header.stamp = pub_time;
    pub_pose_setpoint_.publish(setpoint_vehicle_pose_);

    /// \todo TODO(scm): make this function part of the parser
    // publish waypoint list
    MissionWaypointArray wps_msg;
    wps_msg.waypoints.clear();
    for (const auto wp_value : waypoint_list_)
    {
      MissionWaypoint wp;
      wp.x = wp_value.x;
      wp.y = wp_value.y;
      wp.z = wp_value.z;
      wp.yaw = wp_value.yaw;
      wp.holdtime = wp_value.holdtime;
      wps_msg.waypoints.push_back(wp);
    }
    wps_msg.header = std_msgs::Header();
    wps_msg.header.stamp = pub_time;

    pub_waypoint_list_.publish(wps_msg);
  }
};

void MissionSequencer::performIdle()
{
  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::IDLE");

  // set all mission member variables to default:
  b_wp_is_reached_ = false;
  b_is_landed_ = true;

  // transition to PREARM state when reading from file
  if (sequencer_params_.b_wp_from_file_ && filenames_.size() != 0)
  {
    // delete waypoint list to parse new waypoints from file
    waypoint_list_.clear();
    current_sequencer_state_ = SequencerState::PREARM;
  }
  // transition to ARM state when autosequencing
  else if (sequencer_params_.b_do_autosequence_)
  {
    ROS_INFO_STREAM("* performIdle(): automatically transitioning");
    ROS_INFO_STREAM("* performIdle(): ... arming");

    current_sequencer_state_ = SequencerState::ARM;
  }
}

void MissionSequencer::performPrearming()
{
  // waypoint_list is cleared when entering this state!
  // Transitions possible:
  // - from PREARM to ARM via ARM request or sequencer_params_.b_do_autosequence_
  // - from PREARM to IDLE via ABORT request
  ROS_INFO_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::PREARM");

  // clear waypoint list
  waypoint_list_.clear();

  if (waypoint_list_.empty())
  {
    // Parse waypoints from a file path or list of filenames
    if (parseFilename())
    {
      if (waypoint_list_.empty())
      {
        // no waypoints: PREARM -> IDLE -> PREAM  (repeats until waypoints or no more filenames!)
        ROS_INFO_STREAM("* SequencerState::PREARM: no WAYPOINT(S) read from file...");
        current_sequencer_state_ = SequencerState::IDLE;
      }
      else
      {
        ROS_INFO_STREAM("* SequencerState::PREARM: got " << waypoint_list_.size() << " waypoints from file ...");
        current_sequencer_state_ = SequencerState::PREARM;

        // publish success of file reading
        publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::READ, false, true);

        // auto-transition if parameter is set
        if (sequencer_params_.b_do_autosequence_)
        {
          ROS_INFO_STREAM("* performPrearming(): automatically transitioning");
          ROS_INFO_STREAM("* performPrearming(): ... arming");

          current_sequencer_state_ = SequencerState::ARM;
        }
      }
      // erase current filename
      filenames_.erase(filenames_.begin());
    }
    else
    {
      ROS_INFO_STREAM("* SequencerState::PREARM: no files to read WAYPOINT(S) from ...");
      current_sequencer_state_ = SequencerState::IDLE;
    }
  }
}

void MissionSequencer::performArming()
{
  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::ARM");

  // check if we are already armed
  if (!current_vehicle_state_.armed)
  {
    // check offboard mode
    if (current_vehicle_state_.mode != "OFFBOARD")
    {
      if (ros::Time::now().toSec() - mavros_cmds_.time_offboard_request.toSec() > 2.5)
      {
        if (srv_mavros_set_mode_.call(mavros_cmds_.offboard_mode_) && mavros_cmds_.offboard_mode_.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
        mavros_cmds_.time_offboard_request = ros::Time::now();
      }
    }
    else
    {
      // check if we can arm: Timeout 2.5 sec
      if (ros::Time::now().toSec() - mavros_cmds_.time_arm_request.toSec() > 2.5)
      {
        if (srv_mavros_arm_.call(mavros_cmds_.arm_cmd_) && mavros_cmds_.arm_cmd_.response.success)
        {
          ROS_INFO("Vehicle armed");
          // set armed to true as the next callback might be further away than the request to takeoff
          current_vehicle_state_.armed = true;

          // respond to completion of arming
          publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::ARM, false, true);
        }
        else
        {
          ROS_WARN("* SequencerState::ARM: service mavros->arm failed!");
        }
        mavros_cmds_.time_arm_request = ros::Time::now();
      }
      else
      {
        ROS_INFO_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::ARM: arming timeout ...");
      }
    }
  }
  // we are already armed
  else
  {
    ROS_INFO_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::ARM: arming completed");
    if (sequencer_params_.b_do_autosequence_)
    {
      ROS_INFO_STREAM("* performArming(): automatically starting mission");
      ROS_INFO_STREAM("* performArming(): ... taking off");

      // set takeoff position
      setpoint_takeoff_pose_ = starting_vehicle_pose_;
      setpoint_takeoff_pose_.pose.position.z += sequencer_params_.takeoff_z_;

      // set waypoint reached to false and transition to new state
      b_wp_is_reached_ = false;
      b_is_landed_ = false;
      current_sequencer_state_ = SequencerState::TAKEOFF;
    }
  }
}

void MissionSequencer::performTakeoff()
{
  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::TAKEOFF");

  // check for takeoff type
  if (sequencer_params_.takeoff_type_ == TakeoffType::POSITION)
  {
    // check if takeoff position has been reached
    if (checkWaypoint(setpoint_takeoff_pose_))
    {
      // setpoint reached --> go into hover mode
      ROS_INFO_STREAM("==> TAKEOFF completed");

      // performHover reads "b_wp_is_reached_" thus this flag might be set in advance!
      b_wp_is_reached_ = false;
      b_is_landed_ = false;
      current_sequencer_state_ = SequencerState::HOVER;

      // respond to completion of takeoff
      publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::TAKEOFF, false, true);
    }
  }
}

void MissionSequencer::performMission()
{
  // check if we still have waypoints in the list, otherwise got to HOVER
  if (waypoint_list_.size() > 0)
  {
    geometry_msgs::PoseStamped next_wp = waypointToPoseStamped(waypoint_list_[0]);
    /// \todo TODO(scm): make this check part of the WP callback, such that it is not checked at the check rate, i.e.
    /// 20Hz

    // interpret waypoints relative to current pose
    if (waypoint_list_[0].ref_frame >= Waypoint::ReferenceFrame::CUR_POS)
    {
      ROS_DEBUG_STREAM("-> updated CURPOS waypoint to: \n"
                       << "  x: " << waypoint_list_[0].x << " -> " << next_wp.pose.position.x << "\n"
                       << "  y: " << waypoint_list_[0].y << " -> " << next_wp.pose.position.y << "\n"
                       << "  z: " << waypoint_list_[0].z << " -> " << next_wp.pose.position.z);
      waypoint_list_[0].x = next_wp.pose.position.x;
      waypoint_list_[0].y = next_wp.pose.position.y;
      waypoint_list_[0].z = next_wp.pose.position.z;
      waypoint_list_[0].ref_frame = Waypoint::ReferenceFrame::GLOBAL;
    }

    // check if waypoint is within boundaries
    // Eigen::Vector3d cur_pos =
    //    Eigen::Vector3d(next_wp.pose.position.x, next_wp.pose.position.y, next_wp.pose.position.z);
    // Eigen::Vector3d calc_min = cur_pos;
    // Eigen::Vector3d calc_max = -cur_pos;

    // check if waypoint is within boundaries
    /// \todo TODO(alf): rename cur_pos to next_waypoint, cur_pos it's misleading!
    Eigen::Array3d cur_pos = Eigen::Array3d(next_wp.pose.position.x, next_wp.pose.position.y, next_wp.pose.position.z);
    Eigen::Array3d calc_min = cur_pos;
    Eigen::Array3d calc_max = -cur_pos;

    switch (sequencer_params_.bound_ref_)
    {
      case MissionSequencerOptions::BoundReference::GLOBAL: {
        calc_min -= sequencer_params_.bound_min_;
        calc_max += sequencer_params_.bound_max_;
        break;
      }
      case MissionSequencerOptions::BoundReference::LOCAL: {
        // Eigen::Vector3d start_pos(starting_vehicle_pose_.pose.position.x, starting_vehicle_pose_.pose.position.y,
        //                          starting_vehicle_pose_.pose.position.z);
        Eigen::Array3d start_pos(starting_vehicle_pose_.pose.position.x, starting_vehicle_pose_.pose.position.y,
                                 starting_vehicle_pose_.pose.position.z);
        // Component-wise opertaion
        calc_min -= (sequencer_params_.bound_min_ + start_pos);
        calc_max += (sequencer_params_.bound_max_ + start_pos);
        break;
      }
    }

    if (calc_min.x() < 0 || calc_min.y() < 0 || calc_min.z() < 0 || calc_max.x() < 0 || calc_max.y() < 0 ||
        calc_max.z() < 0)
    {
      ROS_WARN_STREAM("next waypoint out of bounds; skipping ... (" << cur_pos.transpose() << ")");
      ROS_DEBUG_STREAM("bounds diff\n\tmin: " << calc_min.transpose() << "\n\tmax: " << calc_max.transpose());
      waypoint_list_.erase(waypoint_list_.begin());
      return;
      /// \todo TODO(scm) this could be solved better, such that we iterate through all next waypoints until suitable is
      /// found!
    }

    // check if waypoint has been reached
    if (!b_wp_is_reached_ && checkWaypoint(next_wp))
    {
      // set waypoint reached and reset timer
      b_wp_is_reached_ = true;
      time_last_wp_reached_ = ros::Time::now();

      // transition to hover state at waypoint
      current_sequencer_state_ = SequencerState::HOVER;
    }
  }
  else
  {
    ROS_DEBUG_STREAM("* SequencerState::MISSION; No more waypoints to follow...");

    // publish mission completion
    publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::UNDEF, false, true);

    ROS_DEBUG_STREAM("=> mission_sequencer: transition to hover while waiting for next action/waypoints");

    // set waypoint reached and reset timer
    b_wp_is_reached_ = true;
    time_last_wp_reached_ = ros::Time::now();

    // transition to hover state at last active waypoint
    current_sequencer_state_ = SequencerState::HOVER;
  }

  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_,
                            "* SequencerState::MISSION; waypoints left: " << waypoint_list_.size());
}

void MissionSequencer::performHover()
{
  // check size of waypoints list
  if (!waypoint_list_.empty())
  {
    bool b_transition_to_mission = true;
    // check if the goal is to hold at waypoint
    if (b_wp_is_reached_)
    {
      // holdtime check
      if ((ros::Time::now().toSec() - time_last_wp_reached_.toSec()) > waypoint_list_[0].holdtime)
      {
        // waypoint has been completed, delete from list
        ROS_INFO_STREAM("-    waited for: " << waypoint_list_[0].holdtime << " Seconds");
        waypoint_list_.erase(waypoint_list_.begin());

        // make sure the list is not empty
        if (!waypoint_list_.empty())
        {
          //          setpoint_vehicle_pose_ = waypointToPoseStamped(waypoint_list_[0]);
          b_wp_is_reached_ = false;
        }
        else
        {
          ROS_INFO_STREAM("=> mission_sequencer: no more waypoints to fly to ...");

          // no more waypoints
          b_transition_to_mission = false;

          // publish mission completion
          publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::UNDEF, false, true);
        }
      }
      else
      {
        // continue to hover
        b_transition_to_mission = false;

        ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::HOVER; hovering until "
                                                                           "holdtime at waypoint "
                                                                           "is reached ...");
      }
    }
    else
    {
      ROS_INFO_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::HOVER: waypoint not yet "
                                                                        "reached...");
    }

    // check if change to mission state is necessary
    if (b_transition_to_mission)
    {
      current_sequencer_state_ = SequencerState::MISSION;
    }
  }
  // no more waypoints in waypoint_list_
  else
  {
    // check if automatically land
    if (sequencer_params_.b_do_automatically_land_ || sequencer_params_.b_do_autosequence_)
    {
      ROS_INFO_STREAM("* SequencerState::HOVER; No more waypoints to follow: automatically triggered landing...");

      // transition to new state
      b_executed_landing_ = false;
      current_sequencer_state_ = SequencerState::LAND;
    }
    else
    {
      // no more waypoints --> hover at current setpoint
      ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::HOVER; hovering until new "
                                                                         "waypoint "
                                                                         "arrives ...");
    }
  }
}

void MissionSequencer::performLand()
{
  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::LAND: performing landing ...");
  // call landing if not executed yet correctly
  if (!b_executed_landing_)
  {
    b_executed_landing_ = executeLanding();
    if (!b_executed_landing_)
    {
      ROS_WARN_STREAM("* SequencerState::LAND; executing mavors landing command failed! ...");
    }
  }

  // mavros check to see if landed
  if (current_vehicle_ext_state_.landed_state == current_vehicle_ext_state_.LANDED_STATE_ON_GROUND)
  {
    if (!b_is_landed_)
    {
      // set landed states
      b_is_landed_ = true;

      // respond to completion of landing
      publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::LAND, false, true);
    }

    // check if vehicle is armed
    if (!current_vehicle_state_.armed)
    {
      // not armed --> go into IDLE
      ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::LAND->LANDED --> "
                                                                         "set state to "
                                                                         "IDLE");

      // transition to new state
      current_sequencer_state_ = SequencerState::IDLE;

      // respond to completion of disarm
      publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::DISARM, false, true);
    }
    else
    {
      // still armed
      ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::LAND->LANDED --> "
                                                                         "Vehicle definitely still "
                                                                         "ARMED");

      // perform auto disarm
      if (sequencer_params_.b_do_automatically_disarm_ || sequencer_params_.b_do_autosequence_)
      {
        ROS_INFO_STREAM("* performLand(): automatically disarming vehicle");

        // transition to disarm state
        current_sequencer_state_ = SequencerState::DISARM;
      }
    }
  }
  else
  {
    // still armed
    ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::LAND->LANDED --> "
                                                                       "Vehicle still might be still "
                                                                       "ARMED");
  }
}

void MissionSequencer::performHold()
{
  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::HOLD");

  if (sequencer_params_.b_hold_zero_vel_)
  {
    geometry_msgs::TwistStamped set_vel;
    set_vel.twist.linear.x = 0.0;
    set_vel.twist.linear.y = 0.0;
    set_vel.twist.linear.z = 0.0;
    if (checkVelocity(set_vel))
      ROS_DEBUG_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_, " =>  Halted vehicle.");
  }
}

void MissionSequencer::performDisarming()
{
  ROS_DEBUG_STREAM_THROTTLE(sequencer_params_.topic_debug_interval_, "* SequencerState::DISARM");
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
    ROS_INFO("=> mission_sequencer: Disarmed!");

    // respond to completion of disarm
    publishResponse(current_mission_ID_, mission_sequencer::MissionRequest::DISARM, false, true);
    current_sequencer_state_ = SequencerState::IDLE;
  }
}

void MissionSequencer::performAbort()
{
}

bool MissionSequencer::checkWaypoint(const geometry_msgs::PoseStamped& current_waypoint)
{
  // set the current setpoint
  setpoint_vehicle_pose_ = current_waypoint;
  ROS_DEBUG_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_,
                            "-   waypoint:      " << setpoint_vehicle_pose_.pose);
  ROS_DEBUG_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_,
                            "-   pose:          " << current_vehicle_pose_.pose);

  double diff_position, diff_yaw;
  // calculate position difference
  double diff_squared =
      std::pow(std::fabs(current_vehicle_pose_.pose.position.x - current_waypoint.pose.position.x), 2) +
      std::pow(std::fabs(current_vehicle_pose_.pose.position.y - current_waypoint.pose.position.y), 2) +
      std::pow(std::fabs(current_vehicle_pose_.pose.position.z - current_waypoint.pose.position.z), 2);
  diff_position = std::sqrt(diff_squared);
  ROS_DEBUG_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_, "-   diff_position: " << diff_position);

  // claculate yaw difference
  /// \bug BUG(scm): this calculation does calculate the absulute angle between the two quaternions, should also
  /// include roll/pitch!!!
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
  ROS_DEBUG_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_, "-   diff_yaw:      " << diff_yaw);

  // check if waypoint has been reached

  if (diff_position < sequencer_params_.threshold_position_ && std::fabs(diff_yaw) < sequencer_params_.threshold_yaw_)
  {
    /// \todo TODO(scm): provide the yaw somehow here
    ROS_INFO_STREAM("Reached Waypoint: x = "
                    << current_waypoint.pose.position.x << ", y = " << current_waypoint.pose.position.y
                    << ", z = " << current_waypoint.pose.position.z /*<< ", yaw = " << waypoint_list_[0].yaw*/);

    // publish reached waypoint
    MissionWaypointStamped wp_msg;
    wp_msg.header.stamp = ros::Time::now();
    wp_msg.header.frame_id = "todo";
    wp_msg.waypoint.x = current_waypoint.pose.position.x;
    wp_msg.waypoint.y = current_waypoint.pose.position.y;
    wp_msg.waypoint.z = current_waypoint.pose.position.z;
    wp_msg.waypoint.yaw = 0.0;       // waypoint_list_[0].yaw;
    wp_msg.waypoint.holdtime = 0.0;  // waypoint_list_[0].holdtime;
    pub_waypoint_reached_.publish(wp_msg);

    return true;
  }

  // waypoint not reached --> return fals
  return false;
}

bool MissionSequencer::checkVelocity(const geometry_msgs::TwistStamped& set_velocity)
{
  // check if velocity is valid
  if (!b_odom_is_valid_)
  {
    ROS_WARN_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_, " ->  Cannot check for velocity, velocity "
                                                                            "not yet valid!");
    return false;
  }

  ROS_DEBUG_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_,
                            "-   set_vel:       " << set_velocity.twist.linear);
  ROS_DEBUG_STREAM_THROTTLE(0.5 * sequencer_params_.topic_debug_interval_,
                            "-   velocity:      " << current_vehicle_twist_.twist.linear);

  Eigen::Vector3d diff(std::fabs(current_vehicle_twist_.twist.linear.x - set_velocity.twist.linear.x),
                       std::fabs(current_vehicle_twist_.twist.linear.y - set_velocity.twist.linear.y),
                       std::fabs(current_vehicle_twist_.twist.linear.z - set_velocity.twist.linear.z));

  /// \todo TODO(scm): once Eigen is used this can be nicer coded
  // if (!current_vel_reached_[axis] && diff[axis] < sequencer_params_.threshold_velocity_)
  // setpoint_hveicle_pose.position[axis] = ...

  bool vel_reached = true;

  // check for x
  if (!current_vel_reached_[0] && diff[0] < sequencer_params_.threshold_velocity_)
  {
    // update waypoint to current position
    setpoint_vehicle_pose_.pose.position.x = current_vehicle_pose_.pose.position.x;
    current_vel_reached_[0] = true;
    ROS_INFO_STREAM("Reached Velocity: x = " << set_velocity.twist.linear.x
                                             << "=> Set waypoint: x = " << setpoint_vehicle_pose_.pose.position.x);
  }
  else
    vel_reached &= current_vel_reached_[0];
  // check for y
  if (!current_vel_reached_[1] && diff[1] < sequencer_params_.threshold_velocity_)
  {
    // update waypoint to current position
    setpoint_vehicle_pose_.pose.position.y = current_vehicle_pose_.pose.position.y;
    current_vel_reached_[1] = true;
    ROS_INFO_STREAM("Reached Velocity: y = " << set_velocity.twist.linear.y
                                             << "=> Set waypoint: y = " << setpoint_vehicle_pose_.pose.position.y);
  }
  else
    vel_reached &= current_vel_reached_[1];
  // check for z
  if (!current_vel_reached_[2] && diff[2] < sequencer_params_.threshold_velocity_)
  {
    // update waypoint to current position
    setpoint_vehicle_pose_.pose.position.z = current_vehicle_pose_.pose.position.z;
    current_vel_reached_[2] = true;
    ROS_INFO_STREAM("Reached Velocity: z = " << set_velocity.twist.linear.z
                                             << "=> Set waypoint: z = " << setpoint_vehicle_pose_.pose.position.z);
  }
  else
    vel_reached &= current_vel_reached_[2];

  return vel_reached;
}

bool MissionSequencer::checkStateChange(const SequencerState& new_state) const
{
  switch (current_sequencer_state_)
  {
    case SequencerState::IDLE:
      // IDLE -> request(ARM) --> PREARM/ARM
      if (new_state == SequencerState::ARM)
        return true;
      else if (new_state == SequencerState::PREARM)
        return true;
      break;

    case SequencerState::PREARM:
      // PREARM -> request(ARM) -> ARM
      // PREARM -> request(READ) -> PREARM
      // PREARM -> IDLE
      if (new_state == SequencerState::ARM)
        return true;
      if (new_state == SequencerState::IDLE)
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

    case SequencerState::TAKEOFF:
      // TAKEOFF -> request(HOLD) --> HOLD
      // TAKEOFF -> request(LAND) --> LAND
      if (new_state == SequencerState::HOLD)
        return true;
      else if (new_state == SequencerState::LAND)
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

    case SequencerState::HOVER:
      // HOVER -> request(LAND) --> LAND
      // HOVER -> request(HOLD) --> HOLD
      // HOVER -> request(HOVER) --> HOVER
      if (new_state == SequencerState::LAND)
        return true;
      else if (new_state == SequencerState::HOLD)
        return true;
      else if (new_state == SequencerState::HOVER)
        return true;
      break;

    case SequencerState::HOLD:
      // HOLD -> request(RESUME) -> prevstate in MISSION, TAKEOFF, LAND, HOVER --> PREV_STATE
      // HOLD -> request(LAND) -> LAND
      //      if (new_state == SequencerState::LAND)
      //        return true;
      //      else if (new_state == SequencerState::MISSION)
      //        return true;
      //      else if (new_state == SequencerState::HOVER)
      //        return true;
      //      else if (new_state == SequencerState::TAKEOFF)
      //        return true;
      if (new_state == SequencerState::RESUME)
      {
        if (previous_sequencer_state_ == SequencerState::LAND)
          return true;
        else if (previous_sequencer_state_ == SequencerState::MISSION)
          return true;
        else if (previous_sequencer_state_ == SequencerState::HOVER)
          return true;
        else if (previous_sequencer_state_ == SequencerState::TAKEOFF)
          return true;
      }
      else if (new_state == SequencerState::LAND)
        return true;
      break;

    case SequencerState::LAND:
      // LAND -> request(DISARM) -> DISARM
      // LAND -> request(LAND) -> warning + LAND
      if (new_state == SequencerState::DISARM)
        return true;
      else if (new_state == SequencerState::HOLD)
        return true;
      else if (new_state == SequencerState::LAND)
      {
        ROS_WARN_STREAM("* mission_sequencer::request::LAND LAND requested although already in LAND state...");
      }
      break;

    case SequencerState::DISARM:
    default:
      // DISARM -> request(X) --> FAULT no change allowed
      // default -> request(X) --> FAULT no change allowed
      break;
  }

  // state change was not approved
  ROS_ERROR_STREAM("=> No known state transition from " << current_sequencer_state_ << " to " << new_state);
  ROS_ERROR_STREAM("   Staying in " << current_sequencer_state_);
  return false;
}

bool MissionSequencer::executeLanding()
{
  if (srv_mavros_land_.call(mavros_cmds_.land_cmd_))
  {
    if (mavros_cmds_.land_cmd_.response.success)
    {
      ROS_INFO("--> Landing");
      return true;
    }
  }

  return false;
}

bool MissionSequencer::checkMissionID(const uint8_t& mission_id, const uint8_t& request_id)
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
    else if (request_id == mission_sequencer::MissionRequest::ARM && current_sequencer_state_ == SequencerState::IDLE)
    {
      current_mission_ID_ = mission_id;
      return true;
    }

    return false;
  }

  // same ID
  return true;
}

}  // namespace mission_sequencer
