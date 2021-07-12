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

    waypointList_ = std::vector<ParseWaypoint::Waypoint>(0);
    this->reachedWaypoint_ = false;
    this->reachedWaypointTime_ = ros::Time::now();

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

    this->thresholdPosition_ = 0.3;
    this->thresholdYaw_ = 0.1;
    
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


bool AmazeMissionSequencer::getFilenames() {

  // Define filepaths
  XmlRpc::XmlRpcValue filepaths;

  // get filepaths
  if (!nh_.getParam("/autonomy/missions/mission_" + std::to_string(missionID_) + "/filepaths", filepaths)) {

    // [TODO] Manage error
    return false;
  }

  // Check type to be array
  if (filepaths.getType() ==  XmlRpc::XmlRpcValue::TypeArray) {

    // Loop through filepaths
    for (int j = 0; j < filepaths.size(); ++j) {


      // Check type to be string
      if (filepaths[j].getType() ==  XmlRpc::XmlRpcValue::TypeString) {

        // assign filename
        filenames_.emplace_back(std::string(filepaths[j]));

        return true;
      }
    }
  } else {

    // [TODO] Manage error
    return false;
  }
}

void AmazeMissionSequencer::rosRequestCallback(const amaze_mission_sequencer::request::ConstPtr& msg)
{
    bool wrongInput = false;

    // Get mission id
    this->missionID_ = int(msg->id);

    // TODO: This is read mission case later
    // Get filepaths
    if (!getFilenames()) {
      // [TODO] Manage error
    }
    // [TODO] Temporary we use only the first filename of filenames
    //        Here a logic to handle multiple filenames shuld be implemented
    std::string filename = filenames_[0];

    switch (int(msg->request))
    {
        // This will be the arm case
        case 1: //amaze_mission_sequencer::request::START:
            if (this->currentFollowerState_ == IDLE && this->poseValid_ && this->stateValid_ && this->extendedStateValid_)
            {
                std::vector<std::string> header_default = {"x", "y", "z", "yaw", "holdtime"};
                std::shared_ptr<ParseWaypoint> WaypointParser =  std::make_shared<ParseWaypoint>(filename, header_default);

                // Parse waypoint file
                // TODO: SOLVE WRONG ID OR 0-LENGTH WITH C++ EXCEPTION - respond with false
                WaypointParser->readParseCsv();

                // Get the data
                this->waypointList_ = WaypointParser->getData();

                // Set initial pose
                this->startingVehiclePose_ = this->currentVehiclePose_;
                this->vehiclePoseSetpoint_ = this->startingVehiclePose_;
        
        // TODO: A bit of handeling above ^

        // TODO: This is a different case - arm iff this->waypointList_ is not empty and mission ID set
        // this->waypointList_ needs to be cleared in disarm

                // Preparation for arming
                this->armCmd_.request.value = true;
                this->armRequestTime_ = ros::Time::now();
                this->offboardRequestTime_ = ros::Time::now();
                this->currentFollowerState_ = ARM;

                // Respond that mission starts
                this->publishResponse(this->missionID_, true, false);
            }
            else
            {
                wrongInput = true;
            }
            break;
        case 2:
            if (this->currentFollowerState_ == MISSION)
            {
                ROS_INFO_STREAM("Holding Position: x = " << this->currentVehiclePose_.pose.position.x << ", y = " << this->currentVehiclePose_.pose.position.y << ", z = " << this->currentVehiclePose_.pose.position.z);
                this->vehiclePoseSetpoint_ = this->currentVehiclePose_;
                this->currentFollowerState_ = HOLD;

                // Respond that mission starts
                this->publishResponse(this->missionID_, true, false);
            }
            else
            {
                wrongInput = true;
            }
            break;
        case 3:
            if (this->currentFollowerState_ == HOLD)
            {
                ROS_INFO_STREAM("Resuming Mission");
                this->currentFollowerState_ = MISSION;

                // Respond that mission starts
                this->publishResponse(this->missionID_, true, false);
            }
            else
            {
                wrongInput = true;
            }
            break;
        case 4:
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
                this->publishResponse(this->missionID_, true, false);
            break;        
        default:
            ROS_ERROR("REQUEST NOT DEFINED");
            break;
    }

    if (wrongInput)
    {
        ROS_INFO_STREAM("WRONG REQUEST FOR CURRENT STATE: " << StateStr[this->currentFollowerState_]);
        // Respond if input was wrong
        this->publishResponse(this->missionID_, false, false);
    }
};

void AmazeMissionSequencer::publishResponse(int id, bool response, bool completed)
{
    amaze_mission_sequencer::response msg;

    msg.header = std_msgs::Header();
    msg.header.stamp = ros::Time::now();
    msg.id = uint8_t(id);
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

                differenceYaw = 2.0*double(tf2::Quaternion(this->currentVehiclePose_.pose.orientation.x, this->currentVehiclePose_.pose.orientation.y,this->currentVehiclePose_.pose.orientation.z, this->currentVehiclePose_.pose.orientation.w).angle(tf2::Quaternion(currentWaypoint.pose.orientation.x, currentWaypoint.pose.orientation.y, currentWaypoint.pose.orientation.z, currentWaypoint.pose.orientation.w)));
                if (differenceYaw > M_PI)
                {
                    differenceYaw -= 2.0*M_PI;
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
			 this->publishResponse(this->missionID_, false, true);
                    }
                }
            }              
            break;
        
        case LAND:
            if (this->currentExtendedVehicleState_.landed_state == this->currentExtendedVehicleState_.LANDED_STATE_ON_GROUND)
            {
                ROS_INFO("Landed");
                this->armCmd_.request.value = false;
                this->currentFollowerState_ = DISARM;
            }
            break;
        
        case DISARM:
            if (this->rosServiceArm_.call(this->armCmd_))
            {
                if (this->armCmd_.response.success)
                {
                    ROS_INFO("Disarmed");
                    this->currentFollowerState_ = IDLE;
                    // pop top entry of filenames_
                    // Iff list not empty - move tor "prearm"
                    // This "loops" through missions
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
