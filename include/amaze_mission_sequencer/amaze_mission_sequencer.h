#ifndef AMAZE_MISSION_SEQUENCER_H
#define AMAZE_MISSION_SEQUENCER_H


#include <ros/ros.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Include Subscriber Messages
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/PoseStamped.h>
#include <amaze_mission_sequencer/request.h>

// Include Publisher Messages
#include <amaze_mission_sequencer/response.h>

// Include Services
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

// Waypoint list
#include "parse_waypoints.hpp"


#define DEG_TO_RAD  M_PI/180.0

enum State {IDLE, ARM, MISSION, HOLD, LAND, DISARM};
static const char *StateStr[] = { "IDLE", "ARM", "MISSION", "HOLD", "LAND", "DISARM" };

class AmazeMissionSequencer
{
private:
    mavros_msgs::State currentVehicleState_;
    mavros_msgs::ExtendedState currentExtendedVehicleState_;
    geometry_msgs::PoseStamped currentVehiclePose_;

    geometry_msgs::PoseStamped startingVehiclePose_;

    geometry_msgs::PoseStamped vehiclePoseSetpoint_;
    
    int missionID_;
    int requestNumber_;
    State currentFollowerState_;

    std::vector<ParseWaypoint::Waypoint> waypointList_;

    mavros_msgs::SetMode offboardMode_;
    mavros_msgs::CommandBool armCmd_;
    mavros_msgs::CommandTOL landCmd_;
    ros::Time armRequestTime_;
    ros::Time offboardRequestTime_;

    bool stateValid_;
    bool extendedStateValid_;
    bool poseValid_;

    bool relWaypoints_;

    double thresholdPosition_;
    double thresholdYaw_;

    ros::Subscriber rosSubscriberVehicleState_;
    ros::Subscriber rosSubscriberExtendedVehicleState_;
    ros::Subscriber rosSubscriberVehiclePose_;
    ros::Subscriber rosSubscriberRequest_;

    ros::Publisher rosPublisherPoseSetpoint_;
    ros::Publisher rosPublisherResponse_;

    ros::ServiceClient rosServiceArm_;
    ros::ServiceClient rosServiceLand_;
    ros::ServiceClient rosServiceSetMode_;

    void rosVehicleStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void rosExtendedVehicleStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void rosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void rosRequestCallback(const amaze_mission_sequencer::request::ConstPtr& msg);
    
    void publishResponse(int id, bool response, bool completed);

    geometry_msgs::PoseStamped waypointToPoseStamped(const ParseWaypoint::Waypoint& waypoint);

public:
    AmazeMissionSequencer(ros::NodeHandle &nh);
    ~AmazeMissionSequencer();

    void logic(void);
    void publishPoseSetpoint(void);
};



#endif