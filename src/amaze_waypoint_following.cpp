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
#include <string>
#include <vector>
#include <sstream> //istringstream
#include <iostream> // cout
#include <fstream> // ifstream
#include <math.h>
using namespace std;

double l2_norm(vector<double> const& u)
{
    double accum = 0.;
    for (int i = 0; i < u.size(); ++i)
    {
        accum += u[i] * u[i];
    }
    return sqrt(accum);
}
/**
 * Reads csv file into table, exported as a vector of vector of doubles.
 * @param inputFileName input file name (full path).
 * @return data as vector of vector of doubles.
 */
vector< vector<double> > parse2DCsvFile(string inputFileName)
{
    vector<vector<double> > data;
    ifstream inputFile(inputFileName);
    int l = 0;
 
    while (inputFile) {
        l++;
        string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            istringstream ss(s);
            vector<double> record;
 
            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << inputFileName << " line " << l
                         << endl;
                    e.what();
                }
            }
 
            data.push_back(record);
        }
    }
 
    if (!inputFile.eof()) {
        cerr << "Could not read file " << inputFileName << "\n";
        __throw_invalid_argument("File not found.");
    }
 
    return data;
}

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

enum States { EXIT, READ, WAIT, SEND, CHECK };

int main(int argc, char **argv)
{
    bool hovering = false;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber current_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, current_cb);
            // ("/ov_pose", 10, current_cb);
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

    // geometry_msgs::PoseStamped pose;
    // pose = current_pose;
    // pose.pose.position.x += 0;
    // pose.pose.position.y += 0;
    // pose.pose.position.z += 1;
    printf("Current Pose: [%1.2f, %1.2f, %1.2f, %1.2f]\n",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.z);
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(current_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0; 

    ros::Time last_request = ros::Time::now();

    bool reached = false;
    geometry_msgs::PoseStamped set_pose;
    geometry_msgs::PoseStamped starting_pose;

    bool run_FSM = false;

    States current_state_FSM = READ;

    vector<vector<double>> data = parse2DCsvFile("/home/chriboehm/workspaces/CNS/catkin_ws_amaze/trajectory.txt");
    int i = 0;

    starting_pose = current_pose;
    set_pose = starting_pose;
    ros::Time start_time = ros::Time::now();

    bool exit_programm = false;

    vector<double> vect_1;

    while( ros::ok() && !exit_programm )
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)) )
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent )
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }         
        else
        {      
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) )
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success )
                {
                    ROS_INFO("Vehicle armed");
                    run_FSM = true;
                }
                last_request = ros::Time::now();
            }     
        }

        if ( run_FSM == true )
        {
            vect_1 = { set_pose.pose.position.x-current_pose.pose.position.x, 
                    set_pose.pose.position.y-current_pose.pose.position.y,
                    set_pose.pose.position.z-current_pose.pose.position.z };

            printf("Current Pose: [%1.2f, %1.2f, %1.2f, %1.2f]\n",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.z);
            printf("Set Pose: [%1.2f, %1.2f, %1.2f, %1.2f]\n",set_pose.pose.position.x,set_pose.pose.position.y,set_pose.pose.position.z,set_pose.pose.orientation.z);
            printf("Current State: %d\n",current_state_FSM);
            printf("Current 2-Norm value: %f\n",l2_norm(vect_1));

            switch (current_state_FSM)
            {
                case READ:
                    if ( i == static_cast<int>(data.size()) )
                    {
                        printf("No Waypoints left.\n");
                        current_state_FSM = EXIT;
                        break;
                    } 
                    start_time = ros::Time::now();
                    current_state_FSM = WAIT;
                    break;
                case WAIT:
                    if ( ros::Time::now() - start_time > ros::Duration(data.at(i).at(0)) )
                    {
                        current_state_FSM = SEND;
                    }                    
                    break;
                case SEND:
                    printf("Waypoints sent.\n");
                    set_pose.pose.position.x = starting_pose.pose.position.x + data.at(i).at(1);
                    set_pose.pose.position.y = starting_pose.pose.position.y + data.at(i).at(2);
                    set_pose.pose.position.z = starting_pose.pose.position.z + data.at(i).at(3);
                    current_state_FSM = CHECK;
                    break;
                case CHECK:
                    if ( l2_norm(vect_1) < 0.1 )
                    {
                        printf("Waypoints reached.\n");
                        i++;
                        current_state_FSM = READ;
                    }
                    break;                
                default:
                    set_pose.pose.position.x = starting_pose.pose.position.x;
                    set_pose.pose.position.y = starting_pose.pose.position.y;
                    set_pose.pose.position.z = starting_pose.pose.position.z;
                    set_pose.pose.orientation.w = starting_pose.pose.orientation.w;
                    set_pose.pose.orientation.x = starting_pose.pose.orientation.x;
                    set_pose.pose.orientation.y = starting_pose.pose.orientation.y;
                    set_pose.pose.orientation.z = starting_pose.pose.orientation.z;
                    if ( l2_norm(vect_1) < 0.2 )
                    {
                        if( landing_client.call(land_cmd) && land_cmd.response.success )
                        {
                            ROS_INFO("Vehicle landed");
                            run_FSM = false;
                            exit_programm = true;
                        }
                        last_request = ros::Time::now();
                    }
                    break;
            }
        }
        
        set_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(set_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
