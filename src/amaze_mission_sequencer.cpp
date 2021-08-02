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

#include "amaze_mission_sequencer.h"


AmazeMissionSequencer::AmazeMissionSequencer(ros::NodeHandle &nh) :
  nh_(nh)
{
    this->currentVehicleState_ = mavros_msgs::State();
    this->currentExtendedVehicleState_ = mavros_msgs::ExtendedState();
    this->currentVehiclePose_ = geometry_msgs::PoseStamped();

    this->startingVehiclePose_ = geometry_msgs::PoseStamped();

    this->vehiclePoseSetpoint_ = geometry_msgs::PoseStamped();

    this->missionID_ = 0;
    this->requestNumber_ = 0;
    this->currentFollowerState_ = IDLE;

    this->waypointList_ = std::vector<ParseWaypoint::Waypoint>(0);
    this->reachedWaypoint_ = false;
    this->reachedWaypointTime_ = ros::Time::now();

    this->filenames_ = std::vector<std::string>(0);

    this->offboardMode_.request.custom_mode = "OFFBOARD";
    this->armCmd_.request.value = true;
    this->landCmd_.request.yaw = 0;
    this->landCmd_.request.latitude = 0;
    this->landCmd_.request.longitude = 0;
    this->landCmd_.request.altitude = 0;
    this->armRequestTime_ = ros::Time::now();
    this->offboardRequestTime_ = ros::Time::now();

    this->stateValid_ = false;
    this->extendedStateValid_ = false;
    this->poseValid_ = false;

    this->relWaypoints_ = true;

    // Load Threshold Parameters
    if (!nh_.getParam("/amaze_mission_sequencer/threshold_position", this->thresholdPosition_))
    {
      ROS_WARN("Could not retrieve threshold for position, setting to 0.3m");
      this->thresholdPosition_ = 0.3;
    }
    if (!nh_.getParam("/amaze_mission_sequencer/threshold_yaw", this->thresholdYaw_))
    {
      ROS_WARN("Could not retrieve threshold for yaw, setting to 0.1 rad");
      this->thresholdYaw_ = 0.1;
    }

    // Subscriber
    this->rosSubscriberVehicleState_ = nh.subscribe("/mavros/state", 10, &AmazeMissionSequencer::rosVehicleStateCallback, this);
    this->rosSubscriberExtendedVehicleState_ = nh.subscribe("/mavros/extended_state", 10, &AmazeMissionSequencer::rosExtendedVehicleStateCallback, this);
    this->rosSubscriberVehiclePose_ = nh.subscribe("/mavros/local_position/pose", 10, &AmazeMissionSequencer::rosPoseCallback, this);
    this->rosSubscriberRequest_ = nh.subscribe("/autonomy/request", 10, &AmazeMissionSequencer::rosRequestCallback, this);

    // Publisher
    this->rosPublisherPoseSetpoint_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    this->rosPublisherResponse_ = nh.advertise<amaze_mission_sequencer::response>("/autonomy/response", 10);

    this->rosServiceArm_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    this->rosServiceLand_ = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    this->rosServiceSetMode_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
};

AmazeMissionSequencer::~AmazeMissionSequencer()
{

};

void AmazeMissionSequencer::rosVehicleStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    if (!this->stateValid_)
    {
        this->stateValid_ = true;
    }

    this->currentVehicleState_ = *msg;
};

void AmazeMissionSequencer::rosExtendedVehicleStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    if (!this->extendedStateValid_)
    {
        this->extendedStateValid_ = true;
    }

    this->currentExtendedVehicleState_ = *msg;
};

void AmazeMissionSequencer::rosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!this->poseValid_)
    {
        this->startingVehiclePose_ = *msg;
        this->vehiclePoseSetpoint_ = this->startingVehiclePose_;
        this->poseValid_ = true;
    }

    this->currentVehiclePose_ = *msg;
};


bool AmazeMissionSequencer::getFilenames()
{
    // Define filepaths
    XmlRpc::XmlRpcValue filepaths;

    // get filepaths
    if (!nh_.getParam("/autonomy/missions/mission_" + std::to_string(this->missionID_) + "/filepaths", filepaths))
    {
        // [TODO] Manage error
        return false;
    }

    // Check type to be array
    if (filepaths.getType() ==  XmlRpc::XmlRpcValue::TypeArray)
    {
        // Loop through filepaths
        for (int j = 0; j < filepaths.size(); ++j)
        {
            // Check type to be string
            if (filepaths[j].getType() ==  XmlRpc::XmlRpcValue::TypeString)
            {
                // assign filename
                this->filenames_.emplace_back(std::string(filepaths[j]));
            }
        }

        return true;
    }
    else
    {
        // [TODO] Manage error
        return false;
    }
};

void AmazeMissionSequencer::rosRequestCallback(const amaze_mission_sequencer::request::ConstPtr& msg)
{
    bool wrongInput = false;

    // Get mission id
    if (this->missionID_ != int(msg->id))
    {
        if (int(msg->request) == amaze_mission_sequencer::request::READ && (this->currentFollowerState_ == IDLE || this->currentFollowerState_ == PREARM))
        {
            this->currentFollowerState_ = IDLE;
            this->missionID_ = int(msg->id);
        }
        else
        {
            ROS_INFO_STREAM("WRONG REQUEST FOR CURRENT STATE: " << StateStr[this->currentFollowerState_]);
            // Respond if input was wrong
            this->publishResponse(this->missionID_, int(msg->request), false, false);
            return;
        }
    }

    switch (int(msg->request))
    {
        case amaze_mission_sequencer::request::READ:
            if (this->currentFollowerState_ == IDLE)
            {
                try
                {
                    // Get filepaths
                    if (!getFilenames())
                    {
                        // Respond that mission could not be loaded
                        this->publishResponse(this->missionID_, int(msg->request), false, false);
                        ROS_INFO_STREAM("CAN NOT READ MISSION(S)");
                        return;
                    }

                    // Respond that mission has been loaded
                    this->publishResponse(this->missionID_, int(msg->request), true, false);

                    this->currentFollowerState_ = PREARM;
                }
                catch(const std::exception& e)
                {
                    // Respond that mission could not be loaded
                    this->publishResponse(this->missionID_, int(msg->request), false, false);
                    ROS_INFO_STREAM("CAN NOT READ MISSION(S) - Exception");
                    std::cerr << e.what() << '\n';
                }
            }
            else
            {
                wrongInput = true;
            }
            break;
        case amaze_mission_sequencer::request::ARM:
            if (this->currentFollowerState_ == PREARM && this->poseValid_ && this->stateValid_ && this->extendedStateValid_)
            {
                // Take first entry of filename list
                std::string filename = this->filenames_[0];

                std::vector<std::string> header_default = {"x", "y", "z", "yaw", "holdtime"};
                std::shared_ptr<ParseWaypoint> WaypointParser =  std::make_shared<ParseWaypoint>(filename, header_default);

                // Parse waypoint file
                WaypointParser->readParseCsv();

                // Get the data
                this->waypointList_ = WaypointParser->getData();

                // Set initial pose
                this->startingVehiclePose_ = this->currentVehiclePose_;
                this->vehiclePoseSetpoint_ = this->startingVehiclePose_;

                if (this->waypointList_.size() == 0)
                {
                    // Error if waypoint list is empty
                    this->publishResponse(this->missionID_, int(msg->request), false, false);
                    ROS_INFO_STREAM("MISSION FILE EMPTY");
                    return;
                }

                // Preparation for arming
                this->armCmd_.request.value = true;
                this->armRequestTime_ = ros::Time::now();
                this->offboardRequestTime_ = ros::Time::now();
                this->currentFollowerState_ = ARM;

                // Respond that mission starts
                this->publishResponse(this->missionID_, int(msg->request), true, false);
            }
            else
            {
                wrongInput = true;
            }
            break;
        case amaze_mission_sequencer::request::HOLD:
            if (this->currentFollowerState_ == MISSION)
            {
                ROS_INFO_STREAM("Holding Position: x = " << this->currentVehiclePose_.pose.position.x << ", y = " << this->currentVehiclePose_.pose.position.y << ", z = " << this->currentVehiclePose_.pose.position.z);
                this->vehiclePoseSetpoint_ = this->currentVehiclePose_;
                this->currentFollowerState_ = HOLD;

                // Respond that mission starts
                this->publishResponse(this->missionID_, int(msg->request), true, false);
            }
            else
            {
                wrongInput = true;
            }
            break;
        case amaze_mission_sequencer::request::RESUME:
            if (this->currentFollowerState_ == HOLD)
            {
                ROS_INFO_STREAM("Resuming Mission");
                this->currentFollowerState_ = MISSION;

                // Respond that mission starts
                this->publishResponse(this->missionID_, int(msg->request), true, false);
            }
            else
            {
                wrongInput = true;
            }
            break;
        case amaze_mission_sequencer::request::ABORT:
            ROS_INFO("Abort Mission - Landing");
            if (this->rosServiceLand_.call(this->landCmd_))
            {
                if (this->landCmd_.response.success)
                {
                    ROS_INFO("Landing");
                    this->currentFollowerState_ = LAND;
                }
            }
            this->currentFollowerState_ = LAND;

                // Respond that mission starts
                this->publishResponse(this->missionID_, int(msg->request), true, false);
            break; 
    case amaze_mission_sequencer::request::DISARM:
        this->currentFollowerState_ = DISARM;
        // Respond that mission starts
        this->publishResponse(this->missionID_, int(msg->request), true, false);
        break;
        default:
            ROS_ERROR("REQUEST NOT DEFINED");
            break;
    }

    if (wrongInput)
    {
        ROS_INFO_STREAM("WRONG REQUEST FOR CURRENT STATE: " << StateStr[this->currentFollowerState_]);
        // Respond if input was wrong
        this->publishResponse(this->missionID_, int(msg->request), false, false);
    }
};

void AmazeMissionSequencer::publishResponse(int id, int request, bool response, bool completed)
{
    amaze_mission_sequencer::response msg;

    // TODO: add request topics part
    msg.header = std_msgs::Header();
    msg.header.stamp = ros::Time::now();
    msg.request.id = uint8_t(id);
    msg.request.request = uint8_t(request);
    msg.response = response;
    msg.completed = completed;

    this->rosPublisherResponse_.publish(msg);
};

geometry_msgs::PoseStamped AmazeMissionSequencer::waypointToPoseStamped(const ParseWaypoint::Waypoint& waypoint)
{
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.position.z = waypoint.z;

    tf2::Quaternion waypointQuaternion;
    waypointQuaternion.setRotation(tf2::Vector3(0, 0, 1), waypoint.yaw*DEG_TO_RAD);
    waypointQuaternion.normalize();
    if (this->relWaypoints_)
    {
        tf2::Quaternion startingQuaternion(this->startingVehiclePose_.pose.orientation.x, this->startingVehiclePose_.pose.orientation.y,this->startingVehiclePose_.pose.orientation.z, this->startingVehiclePose_.pose.orientation.w);

        waypointQuaternion = startingQuaternion * waypointQuaternion;

        double startingYaw, startingPitch, startingRoll;
        tf2::Matrix3x3(startingQuaternion).getEulerYPR(startingYaw, startingPitch, startingRoll);

        pose.pose.position.x = (waypoint.x*cos(startingYaw) - waypoint.y*sin(startingYaw)) + this->startingVehiclePose_.pose.position.x;
        pose.pose.position.y = (waypoint.x*sin(startingYaw) + waypoint.y*cos(startingYaw)) + this->startingVehiclePose_.pose.position.y;
        pose.pose.position.z = waypoint.z + this->startingVehiclePose_.pose.position.z;
    }

    pose.pose.orientation.x = waypointQuaternion[0];
    pose.pose.orientation.y = waypointQuaternion[1];
    pose.pose.orientation.z = waypointQuaternion[2];
    pose.pose.orientation.w = waypointQuaternion[3];

    return pose;
};

void AmazeMissionSequencer::logic(void)
{
    switch (this->currentFollowerState_)
    {
        case IDLE:
            return;

        case PREARM:
            return;

        case ARM:
            if (!this->currentVehicleState_.armed)
            {
                if (this->currentVehicleState_.mode != "OFFBOARD" && (ros::Time::now() - this->offboardRequestTime_ > ros::Duration(2.5)))
                {
                    if (this->rosServiceSetMode_.call(this->offboardMode_) && this->offboardMode_.response.mode_sent)
                    {
                        ROS_INFO("Offboard enabled");
                    }
                    this->offboardRequestTime_ = ros::Time::now();
                }
                else
                {
                    if (!this->currentVehicleState_.armed && (ros::Time::now() - this->armRequestTime_ > ros::Duration(2.5)))
                    {
                        if (this->rosServiceArm_.call(this->armCmd_) && this->armCmd_.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        this->armRequestTime_ = ros::Time::now();
                    }
                }
            }
            else
            {
                ROS_INFO("Starting Mission");
                ROS_INFO("Taking off");
                // Publish response of start
                this->currentFollowerState_ = MISSION;
                this->reachedWaypoint_ = false;
            }
            break;

        case MISSION:
            if (this->waypointList_.size() != 0)
            {
                double differencePosition;
                double differenceYaw;
                geometry_msgs::PoseStamped currentWaypoint = this->waypointToPoseStamped(this->waypointList_[0]);
                this->vehiclePoseSetpoint_ = currentWaypoint;

                double differenceSquared =  pow(abs(this->currentVehiclePose_.pose.position.x - currentWaypoint.pose.position.x), 2) +
                                            pow(abs(this->currentVehiclePose_.pose.position.y - currentWaypoint.pose.position.y), 2) +
                                            pow(abs(this->currentVehiclePose_.pose.position.z - currentWaypoint.pose.position.z), 2);
                differencePosition = sqrt(differenceSquared);
                // std::cout << "Pos: " << differencePosition << std::endl;

                differenceYaw = std::abs(2.0*double(tf2::Quaternion(this->currentVehiclePose_.pose.orientation.x, this->currentVehiclePose_.pose.orientation.y,this->currentVehiclePose_.pose.orientation.z, this->currentVehiclePose_.pose.orientation.w).angle(tf2::Quaternion(currentWaypoint.pose.orientation.x, currentWaypoint.pose.orientation.y, currentWaypoint.pose.orientation.z, currentWaypoint.pose.orientation.w))));
                if (differenceYaw > 2*M_PI)
                {
                    differenceYaw -= 2*M_PI;
                }

                // std::cout << "Yaw: " << differenceYaw << std::endl;
                if (!this->reachedWaypoint_ && differencePosition<this->thresholdPosition_ && abs(differenceYaw)<this->thresholdYaw_)
                {
                    ROS_INFO_STREAM("Reached Waypoint: x = " << this->waypointList_[0].x << ", y = " << this->waypointList_[0].y << ", z = " << this->waypointList_[0].z << ", yaw = " << this->waypointList_[0].yaw);
                    this->reachedWaypoint_ = true;
                    this->reachedWaypointTime_ = ros::Time::now();
                }

                if (this->reachedWaypoint_ && (ros::Time::now() - this->reachedWaypointTime_)>ros::Duration(this->waypointList_[0].holdtime) )
                {
                    ROS_INFO_STREAM("Waited for: " << this->waypointList_[0].holdtime << " Seconds");
                    this->waypointList_.erase(this->waypointList_.begin());
                    this->vehiclePoseSetpoint_ = this->waypointToPoseStamped(this->waypointList_[0]);
                    this->reachedWaypoint_ = false;
                }
            }
            else
            {
                if (this->rosServiceLand_.call(this->landCmd_))
                {
                    if (this->landCmd_.response.success)
                    {
                        ROS_INFO("Landing");
                        this->currentFollowerState_ = LAND;

                        // Respond that mission succefully finished
                        this->publishResponse(this->missionID_, amaze_mission_sequencer::request::UNDEF, false, true);
                    }
                }
            }
            break;

        case LAND:
            if (this->currentExtendedVehicleState_.landed_state == this->currentExtendedVehicleState_.LANDED_STATE_ON_GROUND)
            {
                ROS_INFO("Landed");
                //this->armCmd_.request.value = false;
                //this->currentFollowerState_ = DISARM;
            }
            break;

        case DISARM:
            this->armCmd_.request.value = false;
            if (this->rosServiceArm_.call(this->armCmd_))
            {
                if (this->armCmd_.response.success)
                {
                    ROS_INFO("Disarmed");
                    this->filenames_.erase(this->filenames_.begin());
                    this->waypointList_.clear();
                    if (this->filenames_.size() == 0)
                    {
                        this->currentFollowerState_ = IDLE;
                    }
                    else
                    {
                        this->currentFollowerState_ = PREARM;
                    }

                }
            }
            break;

        case HOLD:
            break;

        default:
            break;
    }
};

void AmazeMissionSequencer::publishPoseSetpoint(void)
{
    if (this->currentVehicleState_.connected && this->poseValid_)
    {
        this->vehiclePoseSetpoint_.header = std_msgs::Header();
        this->vehiclePoseSetpoint_.header.stamp = ros::Time::now();
        this->rosPublisherPoseSetpoint_.publish(vehiclePoseSetpoint_);
    }

};
