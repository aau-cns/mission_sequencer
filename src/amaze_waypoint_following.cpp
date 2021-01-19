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

///
/// \brief PX4 offboard waypoint service nide
/// Tested in Gazebo SITL and real-world
///

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>  //velocity control
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>

// Vehicle state subscriber
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

// position Subscriber

geometry_msgs::PoseStamped current_pose;
bool pose_ok(false);
void current_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
  pose_ok = true;
}

// landing state
static mavros_msgs::ExtendedState current_ext_state_;
static bool ext_state_msg_ok_ = false;
void ext_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  current_ext_state_ = *msg;
  ext_state_msg_ok_ = true;
}

// tolerance of achieving waypoint =20cm
static double threshold_wp_reached_ = 0.2;

// degrees to radians transformation constant
static double deg_rad_ = 1 / (180.0 / M_PI);

//////////////////reading from text file////////////////////
/*waypoints can be either on the same line in .txt file
or each waypoint is represented in an individual line.
In the .txt file after clarifying the waypoints
configurations the mode should be clarified either 1 absolute or 2 relative*/
static int number_of_wp_ = 0;       // total number of waypoints
static int counter_number_wp_ = 0;  // counter of waypoints
static int wp_index_ = 0;           // index of each waypoint

struct Number
{
  double value;
  operator double() const
  {
    return value;
  }
};

std::istream& operator>>(std::istream& is, Number& number)
{
  is >> number.value;
  // fail istream on anything other than ',' or whitespace
  // end reading on ',' or '\n'
  for (char c = is.get();; c = is.get())
  {
    if (c == ',')
      break;
    if (c == '\n')
    {
      number_of_wp_ = number_of_wp_ + 1;
      break;
    }
    if (std::isspace(c))
      continue;

    is.setstate(std::ios_base::failbit);
    break;
  }
  return is;
}

static std::vector<double> wp_config_vec;
int i = 0;
bool readDATA()
{
  Number value;
  std::ifstream myFile;
  myFile.open("waypoints_cordinates.txt", std::ios::app);
  if (myFile.is_open())
  {
    std::cout << "File is open." << std::endl;
    while (myFile >> value)
    {
      wp_config_vec.push_back(value);
      std::cout << "value is " << value << std::endl;
      std::cout << "wp_config_vec" << i << "=" << wp_config_vec[i] << std::endl;
      i = i + 1;
    }
    myFile.close();
  }
  else
    std::cout << "Unable to open the file";

  // check that there's no missing in the waypoints configurations
  if ((i - 1) % 5 != 0)
  {
    std::cout << "There are missing values in the .txt file" << i << std::endl;
    return 0;
  }
  return 1;
}
///////////////// end of reading from text file ///////////

struct waypoint_config
{
  double x;
  double y;
  double z;
  double yaw;
  double holdtime;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;  // class: ros::NodeHandle, object: nh
  // create publishers and subscribers
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber current_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, current_cb);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::Publisher local_pos_vel = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Subscriber ext_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10, ext_state_cb);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("connecting to FCT...");
  }

  // read data from .txt file
  if (readDATA() == false)
  {
    return 0;
  }
  // printing number of waypoints
  std::cout << "number_of_wp" << number_of_wp_ - 1 << std::endl;
  std::cout << "mode is" << wp_config_vec[i - 1] << std::endl;

  while (pose_ok == false)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("Wait for valid init pose...");
  }

  geometry_msgs::PoseStamped init_pose(current_pose);
  // ROS_INFO_STREAM("Got init pose [x,y,z]: " << init_pose.pose.position.x << " "
  //                                           << init_pose.pose.position.x << " "
  //                                           << init_pose.pose.position.x);

  geometry_msgs::PoseStamped pose;
  // geometry_msgs::TwistStamped vel;
  tf2::Quaternion quat_rot;  // quaternion for rotation

  if (wp_config_vec[i - 1] == 2)
  {
    pose.pose.position.x = wp_config_vec[0] + init_pose.pose.position.x;
    pose.pose.position.y = wp_config_vec[1] + init_pose.pose.position.y;
    pose.pose.position.z = wp_config_vec[2] + init_pose.pose.position.z;
  }
  if (wp_config_vec[i - 1] == 1)
  {
    pose.pose.position.x = wp_config_vec[0];
    pose.pose.position.y = wp_config_vec[1];
    pose.pose.position.z = wp_config_vec[2];
  }
  // send a few setpoints before starting
  for (int j = 100; ros::ok() && j > 0; --j)
  {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = 0;
  land_cmd.request.longitude = 0;
  land_cmd.request.altitude = 0;

  ros::Time last_request = ros::Time::now();

  // change to offboard mode and arm
  while (ros::ok() && !current_state.armed)
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.5)))
    {
      ROS_INFO(current_state.mode.c_str());
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.5)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  tf2::Quaternion q_wp_init;  // quaternion presenting the initial rotation
  if (wp_config_vec[i - 1] == 2)
  {
    q_wp_init[3] = init_pose.pose.orientation.w;
    q_wp_init[0] = init_pose.pose.orientation.x;
    q_wp_init[1] = init_pose.pose.orientation.y;
    q_wp_init[2] = init_pose.pose.orientation.z;
    q_wp_init.normalize();
  }

  while (counter_number_wp_ < (number_of_wp_ - 1))
  {
    waypoint_config wp_config;
    wp_config.x = wp_config_vec[0 + wp_index_];
    wp_config.y = wp_config_vec[1 + wp_index_];
    wp_config.z = wp_config_vec[2 + wp_index_];
    wp_config.yaw = wp_config_vec[3 + wp_index_];
    wp_config.holdtime = wp_config_vec[4 + wp_index_];

    if (wp_config_vec[i - 1] == 2)
    {
      pose.pose.position.x = wp_config.x + init_pose.pose.position.x;
      pose.pose.position.y = wp_config.y + init_pose.pose.position.y;
      pose.pose.position.z = wp_config.z + init_pose.pose.position.z;
    }
    if (wp_config_vec[i - 1] == 1)
    {
      pose.pose.position.x = wp_config.x;
      pose.pose.position.y = wp_config.y;
      pose.pose.position.z = wp_config.z;
    }

    double req_yaw = wp_config.yaw * deg_rad_;  // requested rotation angle
    quat_rot.setRotation(tf2::Vector3(0, 0, 1), req_yaw);
    quat_rot.normalize();

    if (wp_config_vec[i - 1] == 2)
    {
      quat_rot = q_wp_init * quat_rot;
    }
    pose.pose.orientation.w = quat_rot[3];  // real part of quaternion
    pose.pose.orientation.x = quat_rot[0];
    pose.pose.orientation.y = quat_rot[1];
    pose.pose.orientation.z = quat_rot[2];
    //////end of rotation/////////////
    std::cout << "going to the " << counter_number_wp_ << " waypoint" << std::endl;
    bool reached = false;
    ros::Time last_request2 = ros::Time::now();

    while (ros::ok())
    {
      // this loop will remain for infinite seconds
      // going to the requested waypoint + holding at the requested waypoint
      local_pos_pub.publish(pose);
      if (abs(pose.pose.position.x - current_pose.pose.position.x) < threshold_wp_reached_ &&
          abs(pose.pose.position.y - current_pose.pose.position.y) < threshold_wp_reached_ &&
          abs(pose.pose.position.z - current_pose.pose.position.z) < threshold_wp_reached_)
      {
        std::cout << "waypoint " << counter_number_wp_ << " got achieved" << std::endl;
        if (reached == false)
        {
          last_request2 = ros::Time::now();
          reached = true;
        }
        if ((ros::Time::now() - last_request2) > ros::Duration(wp_config.holdtime))
        {
          break;
        }
      }
      ros::spinOnce();
      rate.sleep();
    }
    counter_number_wp_++;       // waypoints counter
    wp_index_ = wp_index_ + 5;  // waypoint index
  }

  // land
  while (ros::ok())
  {
    if (land_client.call(land_cmd))
    {
      // only continue if the vehicle landed
      if (land_cmd.response.success)
      {
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok() && !(current_ext_state_.landed_state == current_ext_state_.LANDED_STATE_ON_GROUND))
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Vehicle landed");

  arm_cmd.request.value = false;

  ROS_INFO("Disarming...");

  while (ros::ok())
  {
    if (arming_client.call(arm_cmd))
    {
      if (arm_cmd.response.success)
      {
        break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Disarmed - Engines got turned off");

  return 0;
}
