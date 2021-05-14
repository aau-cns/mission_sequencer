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


AmazeMissionSequencer::AmazeMissionSequencer(ros::NodeHandle &nh)
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

void AmazeMissionSequencer::rosRequestCallback(const amaze_mission_sequencer::request::ConstPtr& msg)
{
    bool wrongInput = false;    

    switch (int(msg->request))
    {
        case 1:
            if (this->currentFollowerState_ == IDLE && this->poseValid_ && this->stateValid_ && this->extendedStateValid_)
            {
                this->missionID_ = int(msg->id);

                // Define waypoint parser
                // std::string filename = "/home/chriboehm/workspaces/mission_ws/src/amaze_mission_sequencer/trajectories/test_trajectory.csv";
                std::string filename = "/home/core/catkin_ws/src/amaze_mission_sequencer/trajectories/test_trajectory.csv";
                std::vector<std::string> header_default = {"x", "y", "z", "yaw"};
                std::shared_ptr<ParseWaypoint> WaypointParser =  std::make_shared<ParseWaypoint>(filename, header_default);

                // Parse waypoint file
                // TODO: SOLVE WRONG ID OR 0-LENGTH WITH C++ EXCEPTION - respond with false
                WaypointParser->readParseCsv();

                // Get the data
                this->waypointList_ = WaypointParser->getData();

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
    
    pose.pose.position.x = waypoint.x + double(this->relWaypoints_)*this->startingVehiclePose_.pose.position.x;
    pose.pose.position.y = waypoint.y + double(this->relWaypoints_)*this->startingVehiclePose_.pose.position.y;
    pose.pose.position.z = waypoint.z + double(this->relWaypoints_)*this->startingVehiclePose_.pose.position.z;
    
    tf2::Quaternion quaternion;
    quaternion.setRotation(tf2::Vector3(0, 0, 1), waypoint.yaw*DEG_TO_RAD);
    quaternion.normalize();
    if (this->relWaypoints_)
    {
        quaternion = tf2::Quaternion(this->startingVehiclePose_.pose.orientation.x, this->startingVehiclePose_.pose.orientation.y,this->startingVehiclePose_.pose.orientation.z, this->startingVehiclePose_.pose.orientation.w) * quaternion;
    }    
    pose.pose.orientation.x = quaternion[0];
    pose.pose.orientation.y = quaternion[1];
    pose.pose.orientation.z = quaternion[2];
    pose.pose.orientation.w = quaternion[3];

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
                // std::cout << differencePosition << std::endl;

                differenceYaw = 2.0*double(tf2::Quaternion(this->currentVehiclePose_.pose.orientation.x, this->currentVehiclePose_.pose.orientation.y,this->currentVehiclePose_.pose.orientation.z, this->currentVehiclePose_.pose.orientation.w).angle(tf2::Quaternion(currentWaypoint.pose.orientation.x, currentWaypoint.pose.orientation.y, currentWaypoint.pose.orientation.z, currentWaypoint.pose.orientation.w)));
                if (differenceYaw > M_PI)
                {
                    differenceYaw -= 2.0*M_PI;
                }                
                // std::cout << differenceYaw << std::endl;

                if (differencePosition<this->thresholdPosition_ && abs(differenceYaw)<this->thresholdYaw_)
                {
                    ROS_INFO_STREAM("Reached Waypoint: x = " << this->waypointList_[0].x << ", y = " << this->waypointList_[0].y << ", z = " << this->waypointList_[0].z << ", yaw = " << this->waypointList_[0].yaw);
                    this->waypointList_.erase(this->waypointList_.begin());
                    this->vehiclePoseSetpoint_ = this->waypointToPoseStamped(this->waypointList_[0]);
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

                    // Respond that mission starts
                    this->publishResponse(this->missionID_, false, true);
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
