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

#include "mission_sequencer.hpp"

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

AmazeMissionSequencer::AmazeMissionSequencer(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
  currentVehicleState_ = mavros_msgs::State();
  currentExtendedVehicleState_ = mavros_msgs::ExtendedState();
  currentVehiclePose_ = geometry_msgs::PoseStamped();

  startingVehiclePose_ = geometry_msgs::PoseStamped();

  vehiclePoseSetpoint_ = geometry_msgs::PoseStamped();

  missionID_ = 0;
  requestNumber_ = 0;
  currentFollowerState_ = IDLE;

  waypointList_ = std::vector<ParseWaypoint::Waypoint>(0);
  reachedWaypoint_ = false;
  reachedWaypointTime_ = ros::Time::now();

  filenames_ = std::vector<std::string>(0);

  offboardMode_.request.custom_mode = "OFFBOARD";
  armCmd_.request.value = true;
  landCmd_.request.yaw = 0;
  landCmd_.request.latitude = 0;
  landCmd_.request.longitude = 0;
  landCmd_.request.altitude = 0;
  armRequestTime_ = ros::Time::now();
  offboardRequestTime_ = ros::Time::now();

  stateValid_ = false;
  extendedStateValid_ = false;
  poseValid_ = false;

  relWaypoints_ = true;

  landed_ = false;

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
  pnh_.param<bool>("automatic_landing", automatically_land_, false);
  pnh_.param<bool>("verbose", verbose_, false);
  pnh_.param<bool>("relative_waypoints", relWaypoints_, true);

  // Subscribers
  rosSubscriberVehicleState_ = nh.subscribe("/mavros/state", 10, &AmazeMissionSequencer::rosVehicleStateCallback, this);
  rosSubscriberExtendedVehicleState_ =
      nh.subscribe("/mavros/extended_state", 10, &AmazeMissionSequencer::rosExtendedVehicleStateCallback, this);
  rosSubscriberVehiclePose_ =
      nh.subscribe("/mavros/local_position/pose", 10, &AmazeMissionSequencer::rosPoseCallback, this);
  rosSubscriberRequest_ = nh.subscribe("/autonomy/request", 10, &AmazeMissionSequencer::rosRequestCallback, this);

  // Subscribers (relative to node's namespace)
  rosSubscriberVehicleState_ = nh_.subscribe("mavros/state", 10, &AmazeMissionSequencer::rosVehicleStateCallback, this);
  rosSubscriberExtendedVehicleState_ =
      nh_.subscribe("mavros/extended_state", 10, &AmazeMissionSequencer::rosExtendedVehicleStateCallback, this);
  rosSubscriberVehiclePose_ =
      nh_.subscribe("mavros/local_position/pose", 10, &AmazeMissionSequencer::rosPoseCallback, this);
  rosSubscriberRequest_ = nh_.subscribe("autonomy/request", 10, &AmazeMissionSequencer::rosRequestCallback, this);
  rosSubscriberWayPointFileName_ =
      pnh_.subscribe("waypoint_filename", 10, &AmazeMissionSequencer::rosWaypointFilenameCallback, this);

  // Publishers (relative to node's namespace)
  rosPublisherPoseSetpoint_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  rosPublisherResponse_ = nh_.advertise<mission_sequencer::MissionResponse>("autonomy/response", 10);

  // Services (relative to node's namespace)
  std::string service_mavros_cmd_arming;
  pnh_.param<std::string>("mavros_cmd_arming_o", service_mavros_cmd_arming, "mavros/cmd/arming");
  std::string service_mavros_cmd_command;
  pnh_.param<std::string>("mavros_cmd_command_o", service_mavros_cmd_command, "mavros/cmd/command");
  std::string service_mavros_cmd_land;
  pnh_.param<std::string>("mavros_cmd_land_o", service_mavros_cmd_land, "mavros/cmd/land");
  std::string service_mavros_set_mode;
  pnh_.param<std::string>("mavros_set_mode_o", service_mavros_set_mode, "mavros/set_mode");

  rosServiceArm_ = nh_.serviceClient<mavros_msgs::CommandBool>(service_mavros_cmd_arming);
  rosServiceDisrm_ = nh_.serviceClient<mavros_msgs::CommandLong>(service_mavros_cmd_command);
  rosServiceLand_ = nh_.serviceClient<mavros_msgs::CommandTOL>(service_mavros_cmd_land);
  rosServiceSetMode_ = nh_.serviceClient<mavros_msgs::SetMode>(service_mavros_set_mode);

  setFilename(waypoint_fn);
};

AmazeMissionSequencer::~AmazeMissionSequencer(){

};

void AmazeMissionSequencer::rosVehicleStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  if (!stateValid_)
  {
    stateValid_ = true;
  }

  currentVehicleState_ = *msg;
};

void AmazeMissionSequencer::rosExtendedVehicleStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  if (!extendedStateValid_)
  {
    extendedStateValid_ = true;
  }

  currentExtendedVehicleState_ = *msg;
};

void AmazeMissionSequencer::rosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!poseValid_ || (currentFollowerState_ == IDLE))
  {
    startingVehiclePose_ = *msg;

    // Initial YAW as it is used for LANDING
    // The vehilce pose is in ENU:
    tf2::Quaternion q_ENU_BODY(startingVehiclePose_.pose.orientation.x, startingVehiclePose_.pose.orientation.y,
                               startingVehiclePose_.pose.orientation.z, startingVehiclePose_.pose.orientation.w);

    tf2::Quaternion q_NED_2_ENU(0, 0, 0.7071068, 0.7071068);  // [x,y,z,w]
    // q_NED_2_ENU.setRotation(tf2::Vector3(0.7071068, 0, 0.7071068), M_PI);
    // q_NED_2_ENU.normalize();
    tf2::Quaternion q_y;
    q_y.setRotation(tf2::Vector3(0, 1, 0), M_PI);

    tf2::Quaternion q_NED_BODY = q_y * q_NED_2_ENU * q_ENU_BODY;
    double startingYaw, startingPitch, startingRoll;
    tf2::Matrix3x3(q_NED_BODY).getEulerYPR(startingYaw, startingPitch, startingRoll);
    startingYaw = warp_to_pi(startingYaw);
    if (verbose_)
    {
      ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* Initial (PX4/NED) yaw= "
                                                       << startingYaw * RAD_TO_DEG << "[deg], (OptiTrack/ENU) pos x="
                                                       << startingVehiclePose_.pose.position.x
                                                       << " pos y=" << startingVehiclePose_.pose.position.y
                                                       << " pos z=" << startingVehiclePose_.pose.position.z);
    }

    landCmd_.request.yaw = startingYaw * RAD_TO_DEG;
    landCmd_.request.altitude = startingVehiclePose_.pose.position.z;
    vehiclePoseSetpoint_ = startingVehiclePose_;
    poseValid_ = true;
  }

  currentVehiclePose_ = *msg;
};

bool AmazeMissionSequencer::getFilenames()
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
    if (!nh_.getParam("autonomy/missions/mission_" + std::to_string(missionID_) + "/filepaths", filepaths))
    {
      // [TODO] Manage error
      ROS_WARN_STREAM("AmazeMissionSequencer::getFilenames(): failure! Cound not get file paths for mission:"
                      << std::to_string(missionID_));
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

bool AmazeMissionSequencer::setFilename(std::string const waypoint_fn)
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

void AmazeMissionSequencer::rosRequestCallback(const mission_sequencer::MissionRequest::ConstPtr& msg)
{
  bool wrongInput = false;

  // Get mission id
  if (missionID_ != int(msg->id))
  {
    if (int(msg->request) == mission_sequencer::MissionRequest::READ &&
        (currentFollowerState_ == IDLE || currentFollowerState_ == PREARM))
    {
      currentFollowerState_ = IDLE;
      missionID_ = int(msg->id);
    }
    else
    {
      ROS_INFO_STREAM("WRONG MISSION ID FOR CURRENT STATE: " << StateStr[currentFollowerState_] << "; Local mission ID:"
                                                             << missionID_ << " msg ID: " << int(msg->id));
      // Respond if input was wrong
      publishResponse(missionID_, int(msg->request), false, false);
      return;
    }
  }

  switch (int(msg->request))
  {
    case mission_sequencer::MissionRequest::READ:
      if (verbose_)
      {
        ROS_INFO_STREAM("* amaze_mission_sequencer::request::READ...");
      }
      if (currentFollowerState_ == IDLE)
      {
        try
        {
          // Get filepaths
          if (!getFilenames())
          {
            // Respond that mission could not be loaded
            publishResponse(missionID_, int(msg->request), false, false);
            ROS_INFO_STREAM("CAN NOT READ MISSION(S)");
            return;
          }

          // Respond that mission has been loaded
          publishResponse(missionID_, int(msg->request), true, false);

          currentFollowerState_ = PREARM;
        }
        catch (const std::exception& e)
        {
          // Respond that mission could not be loaded
          publishResponse(missionID_, int(msg->request), false, false);
          ROS_INFO_STREAM("CAN NOT READ MISSION(S) - Exception");
          std::cerr << e.what() << '\n';
        }
      }
      else
      {
        if (verbose_)
        {
          ROS_WARN_STREAM("* amaze_mission_sequencer::request::READ - failed! Not in IDLE nor!");
        }
        wrongInput = true;
      }
      break;

    case mission_sequencer::MissionRequest::ARM:
      ROS_INFO_STREAM("* amaze_mission_sequencer::request::ARM...");
      if (currentFollowerState_ == PREARM && poseValid_ && stateValid_ && extendedStateValid_)
      {
        // Take first entry of filename list
        std::string filename = filenames_[0];

        std::vector<std::string> header_default = { "x", "y", "z", "yaw", "holdtime" };
        std::shared_ptr<ParseWaypoint> WaypointParser = std::make_shared<ParseWaypoint>(filename, header_default);

        // Parse waypoint file
        WaypointParser->readParseCsv();

        // Get the data
        waypointList_ = WaypointParser->getData();

        // Set initial pose
        startingVehiclePose_ = currentVehiclePose_;
        vehiclePoseSetpoint_ = startingVehiclePose_;

        if (waypointList_.size() == 0)
        {
          // Error if waypoint list is empty
          publishResponse(missionID_, int(msg->request), false, false);
          ROS_INFO_STREAM("MISSION FILE EMPTY");
          return;
        }

        // Preparation for arming
        armCmd_.request.value = true;
        armRequestTime_ = ros::Time::now();
        offboardRequestTime_ = ros::Time::now();
        currentFollowerState_ = ARM;

        // Respond to request
        publishResponse(missionID_, int(msg->request), true, false);
      }
      else
      {
        if (verbose_)
        {
          ROS_WARN_STREAM("* amaze_mission_sequencer::request::ARM - failed! Not in PREARM!");
        }
        if (verbose_)
        {
          ROS_WARN_STREAM("*   Valids: Pose=" << poseValid_ << ", State=" << stateValid_
                                              << ", extState=" << extendedStateValid_);
        }
        wrongInput = true;
      }
      break;

    case mission_sequencer::MissionRequest::HOLD:
      if (verbose_)
      {
        ROS_INFO_STREAM("* amaze_mission_sequencer::request::HOLD...");
      }
      if (currentFollowerState_ == MISSION)
      {
        ROS_INFO_STREAM("Holding Position: x = " << currentVehiclePose_.pose.position.x
                                                 << ", y = " << currentVehiclePose_.pose.position.y
                                                 << ", z = " << currentVehiclePose_.pose.position.z);
        vehiclePoseSetpoint_ = currentVehiclePose_;
        currentFollowerState_ = HOLD;

        // Respond to request
        publishResponse(missionID_, int(msg->request), true, false);
      }
      else
      {
        if (verbose_)
        {
          ROS_WARN_STREAM("* amaze_mission_sequencer::request::HOLD - failed! Not in MISSION!");
        }
        wrongInput = true;
      }
      break;
    case mission_sequencer::MissionRequest::RESUME:
      ROS_INFO_STREAM("* amaze_mission_sequencer::request::RESUME...");
      if (currentFollowerState_ == HOLD)
      {
        ROS_INFO_STREAM("Resuming Mission");
        currentFollowerState_ = MISSION;

        // Respond to request
        publishResponse(missionID_, int(msg->request), true, false);
      }
      else
      {
        if (verbose_)
        {
          ROS_WARN_STREAM("* amaze_mission_sequencer::request::RESUME - failed! Not in HOLD!");
        }
        wrongInput = true;
      }
      break;

    case mission_sequencer::MissionRequest::ABORT:
      if (verbose_)
      {
        ROS_INFO_STREAM("* amaze_mission_sequencer::request::ABORT...");
      }
      ROS_INFO("Abort Mission - Landing");
      if (rosServiceLand_.call(landCmd_))
      {
        if (landCmd_.response.success)
        {
          ROS_INFO("Landing");
          currentFollowerState_ = LAND;
        }
      }
      currentFollowerState_ = LAND;

      // Respond to request
      publishResponse(missionID_, int(msg->request), true, false);
      break;

    case mission_sequencer::MissionRequest::LAND:
      if (verbose_)
      {
        ROS_INFO_STREAM("* amaze_mission_sequencer::request::LAND...");
      }
      if (rosServiceLand_.call(landCmd_))
      {
        if (landCmd_.response.success)
        {
          ROS_INFO("Landing");
          currentFollowerState_ = LAND;
        }
      }
      currentFollowerState_ = LAND;

      // Respond to request
      publishResponse(missionID_, int(msg->request), true, false);
      break;

    case mission_sequencer::MissionRequest::DISARM:

      // Preparation for arming
      disarmCmd_.request.broadcast = false;
      disarmCmd_.request.command = 400;
      disarmCmd_.request.confirmation = 0;
      disarmCmd_.request.param1 = 0.0;
      disarmCmd_.request.param2 = 21196.0;
      disarmCmd_.request.param3 = 0.0;
      disarmCmd_.request.param4 = 0.0;
      disarmCmd_.request.param5 = 0.0;
      disarmCmd_.request.param6 = 0.0;
      disarmCmd_.request.param7 = 0.0;
      disarmRequestTime_ = ros::Time::now();

      // Set state
      ROS_INFO_STREAM("* amaze_mission_sequencer::request::DISARM...");
      currentFollowerState_ = DISARM;

      // Respond to request
      publishResponse(missionID_, int(msg->request), true, false);
      break;

    default:
      ROS_ERROR("REQUEST NOT DEFINED");
      break;
  }

  if (wrongInput)
  {
    ROS_INFO_STREAM("WRONG REQUEST FOR CURRENT STATE: " << StateStr[currentFollowerState_]);
    // Respond if input was wrong
    publishResponse(missionID_, int(msg->request), false, false);
  }
};

void AmazeMissionSequencer::rosWaypointFilenameCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string fn = msg->data.c_str();
  bool res = setFilename(fn);
  if (verbose_)
  {
    ROS_INFO_STREAM("Received new waypoint_filename: " << fn << "; accepted:" << res);
  }
}

void AmazeMissionSequencer::publishResponse(int id, int request, bool response, bool completed)
{
  mission_sequencer::MissionResponse msg;

  // TODO: add request topics part
  msg.header = std_msgs::Header();
  msg.header.stamp = ros::Time::now();
  msg.request.id = uint8_t(id);
  msg.request.request = uint8_t(request);
  msg.response = response;
  msg.completed = completed;

  rosPublisherResponse_.publish(msg);
};

geometry_msgs::PoseStamped AmazeMissionSequencer::waypointToPoseStamped(const ParseWaypoint::Waypoint& waypoint)
{
  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = waypoint.z;

  tf2::Quaternion waypointQuaternion;
  waypointQuaternion.setRotation(tf2::Vector3(0, 0, 1), waypoint.yaw * DEG_TO_RAD);
  waypointQuaternion.normalize();
  if (relWaypoints_)
  {
    tf2::Quaternion startingQuaternion(startingVehiclePose_.pose.orientation.x, startingVehiclePose_.pose.orientation.y,
                                       startingVehiclePose_.pose.orientation.z,
                                       startingVehiclePose_.pose.orientation.w);

    waypointQuaternion = startingQuaternion * waypointQuaternion;

    double startingYaw, startingPitch, startingRoll;
    tf2::Matrix3x3(startingQuaternion).getEulerYPR(startingYaw, startingPitch, startingRoll);

    pose.pose.position.x =
        (waypoint.x * cos(startingYaw) - waypoint.y * sin(startingYaw)) + startingVehiclePose_.pose.position.x;
    pose.pose.position.y =
        (waypoint.x * sin(startingYaw) + waypoint.y * cos(startingYaw)) + startingVehiclePose_.pose.position.y;
    pose.pose.position.z = waypoint.z + startingVehiclePose_.pose.position.z;
  }

  pose.pose.orientation.x = waypointQuaternion[0];
  pose.pose.orientation.y = waypointQuaternion[1];
  pose.pose.orientation.z = waypointQuaternion[2];
  pose.pose.orientation.w = waypointQuaternion[3];

  return pose;
};

void AmazeMissionSequencer::logic(void)
{
  switch (currentFollowerState_)
  {
    case IDLE:
      if (verbose_)
      {
        ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::IDLE");
      }
      return;

    case PREARM:
      if (verbose_)
      {
        ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::PREARM");
      }
      return;

    case ARM:
      if (verbose_)
      {
        ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::ARM");
      }
      if (!currentVehicleState_.armed)
      {
        if (currentVehicleState_.mode != "OFFBOARD" && (ros::Time::now().toSec() - offboardRequestTime_.toSec() > 2.5))
        {
          if (rosServiceSetMode_.call(offboardMode_) && offboardMode_.response.mode_sent)
          {
            ROS_INFO("Offboard enabled");
          }
          offboardRequestTime_ = ros::Time::now();
        }
        else
        {
          if (!currentVehicleState_.armed && (ros::Time::now().toSec() - armRequestTime_.toSec() > 2.5))
          {
            if (rosServiceArm_.call(armCmd_) && armCmd_.response.success)
            {
              ROS_INFO("Vehicle armed");
            }
            armRequestTime_ = ros::Time::now();
          }
        }
      }
      else
      {
        ROS_INFO("Starting Mission");
        ROS_INFO("Taking off");
        // Publish response of start
        currentFollowerState_ = MISSION;
        reachedWaypoint_ = false;
        landed_ = true;
      }
      break;

    case MISSION:
      if (waypointList_.size() > 0)
      {
        double differencePosition;
        double differenceYaw;
        geometry_msgs::PoseStamped currentWaypoint = waypointToPoseStamped(waypointList_[0]);
        vehiclePoseSetpoint_ = currentWaypoint;

        double differenceSquared = pow(abs(currentVehiclePose_.pose.position.x - currentWaypoint.pose.position.x), 2) +
                                   pow(abs(currentVehiclePose_.pose.position.y - currentWaypoint.pose.position.y), 2) +
                                   pow(abs(currentVehiclePose_.pose.position.z - currentWaypoint.pose.position.z), 2);
        differencePosition = sqrt(differenceSquared);
        // std::cout << "Pos: " << differencePosition << std::endl;

        differenceYaw = std::abs(
            2.0 * double(tf2::Quaternion(currentVehiclePose_.pose.orientation.x, currentVehiclePose_.pose.orientation.y,
                                         currentVehiclePose_.pose.orientation.z, currentVehiclePose_.pose.orientation.w)
                             .angle(tf2::Quaternion(
                                 currentWaypoint.pose.orientation.x, currentWaypoint.pose.orientation.y,
                                 currentWaypoint.pose.orientation.z, currentWaypoint.pose.orientation.w))));
        differenceYaw = std::fmod(differenceYaw, 2 * M_PI);
        if (differenceYaw > M_PI)
        {
          differenceYaw -= 2 * M_PI;
        }

        // std::cout << "Yaw: " << differenceYaw << std::endl;
        if (!reachedWaypoint_ && differencePosition < thresholdPosition_ && std::abs(differenceYaw) < thresholdYaw_)
        {
          ROS_INFO_STREAM("Reached Waypoint: x = " << waypointList_[0].x << ", y = " << waypointList_[0].y << ", z = "
                                                   << waypointList_[0].z << ", yaw = " << waypointList_[0].yaw);
          reachedWaypoint_ = true;
          reachedWaypointTime_ = ros::Time::now();
        }

        if (reachedWaypoint_ && (ros::Time::now().toSec() - reachedWaypointTime_.toSec()) > waypointList_[0].holdtime)
        {
          ROS_INFO_STREAM("Waited for: " << waypointList_[0].holdtime << " Seconds");
          waypointList_.erase(waypointList_.begin());
          // FIX(scm): make sure the list is not empty!!!
          if (waypointList_.size() != 0)
          {
            vehiclePoseSetpoint_ = waypointToPoseStamped(waypointList_[0]);
            reachedWaypoint_ = false;
          }
        }
      }
      else
      {
        if (verbose_)
        {
          ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* No more waypoints to follow...");
        }
        if (rosServiceLand_.call(landCmd_) || automatically_land_)
        {
          if (landCmd_.response.success)
          {
            ROS_INFO("Landing");
            currentFollowerState_ = LAND;

            // Respond that mission succefully finished
            publishResponse(missionID_, mission_sequencer::MissionRequest::UNDEF, false, true);
          }
        }
      }
      if (verbose_)
      {
        ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_,
                                 "* currentFollowerState__::MISSION; waypoints left: " << waypointList_.size());
      }
      break;

    case LAND:
      if (currentExtendedVehicleState_.landed_state == currentExtendedVehicleState_.LANDED_STATE_ON_GROUND)
      {
        landed_ = true;

        if (!currentVehicleState_.armed)
        {
          if (verbose_)
          {
            ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::LAND->LANDED --> set state to "
                                                         "IDLE");
          }
          currentFollowerState_ = IDLE;
        }
        else
        {
          if (verbose_)
          {
            ROS_INFO_STREAM_THROTTLE(dbg_throttle_rate_, "* currentFollowerState__::LAND->LANDED --> Vehicle still "
                                                         "ARMED");
          }
        }
      }
      break;

    case DISARM: {
      bool is_disarmed = true;
      if (currentVehicleState_.armed)
      {
        is_disarmed = false;
        if (rosServiceDisrm_.call(disarmCmd_))
        {
          if (disarmCmd_.response.success)
          {
            is_disarmed = true;
          }
        }

        // If you still want to let the PX$ check for vehicle on the ground than
        // de-comment the following code and move the above snippent into
        // the else condition below

        // if(!this.landed_)
        //{
        //	if (currentExtendedVehicleState_.landed_state == currentExtendedVehicleState_.LANDED_STATE_ON_GROUND)
        //	{
        //		landed_ = true;
        //		ROS_INFO("Landed");
        //	}
        //} else {
        //
        //}
      }

      if (is_disarmed)
      {
        ROS_INFO("Disarmed");
        filenames_.erase(filenames_.begin());
        waypointList_.clear();
        if (filenames_.size() == 0)
        {
          currentFollowerState_ = IDLE;
        }
        else
        {
          currentFollowerState_ = PREARM;
        }

        // If you still want to let the PX$ check for vehicle on the ground than
        // de-comment the following code and move the above snippent into
        // the else condition below

        // if(!this.landed_)
        //{
        //  if (currentExtendedVehicleState_.landed_state == currentExtendedVehicleState_.LANDED_STATE_ON_GROUND)
        //  {
        //      landed_ = true;
        //      ROS_INFO("Landed");
        //  }
        //} else {
        //
        //}
      }
    }
    break;

    case HOLD:
      break;
  }
};

void AmazeMissionSequencer::publishPoseSetpoint(void)
{
  if (currentVehicleState_.connected && poseValid_)
  {
    vehiclePoseSetpoint_.header = std_msgs::Header();
    vehiclePoseSetpoint_.header.stamp = ros::Time::now();
    rosPublisherPoseSetpoint_.publish(vehiclePoseSetpoint_);
  }
};
