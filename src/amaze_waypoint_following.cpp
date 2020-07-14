/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

 //https://uenota.github.io/px4_teleop/px4__teleop__cmds_8hpp_source.html

/*
    TODO: - relative navigation from start.
          - whole mission outline

*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandTOLRequest.h>
#include <mavros_msgs/CommandTOLResponse.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pose;
bool pose_ok(false);
void current_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    pose_ok = true;
}

int main(int argc, char **argv)
{
    bool hovering = false;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber current_sub = nh.subscribe<geometry_msgs::PoseStamped>
            // ("/mavros/local_position/pose", 10, current_cb);
            ("/ov_pose", 10, current_cb);
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !pose_ok)
    {
        ros::spinOnce();
        rate.sleep();
    }
    

    geometry_msgs::PoseStamped pose;
    pose = current_pose;
    pose.pose.position.x += 0;
    pose.pose.position.y += 0;
    pose.pose.position.z += 1;
    printf("Current Pose: [%1.2f, %1.2f, %1.2f, %1.2f]\n",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.z);
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool reached = false;
    while(ros::ok()){
        // printf("Current Pose: [%1.2f, %1.2f, %1.2f, %1.2f]\n",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.z);
        // printf("Set Pose: [%1.2f, %1.2f, %1.2f, %1.2f]\n",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,pose.pose.orientation.z);
        
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        } else {
            
            if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
        }

        if(!reached && current_pose.pose.position.z >= 1.3)
        {
            ROS_INFO("Height reached.");
            reached = true;
            pose.pose.position.z -= 1;
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
