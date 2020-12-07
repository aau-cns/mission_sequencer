---
title: amaze_waypoint_following
author:
  - Control of Networked Systems, University of Klagenfurt, Austria
date: 07.12.2020
subtitle: Version 1.0

documentclass: scrartcl
numbersections: true

toc: true
---

# Amaze Waypoint Following

Waypoint service for a PX4 navigation setup including a minimal service client for comand line interaction.

Maintainer: Christoph Böhm christoph.boehm@aau.at

## Getting Started

Clone the project into your workspace and build it in the ROS environment.
```sh
git clone git@gitlab.aau.at:aau-cns/amaze-waypoint-following.git
```

## Usage (waypoint_service)
**Build the amaze waypoint following package**

```sh
catkin build amaze_waypoint_following
```

**Run the waypoint server**

```sh
rosrun amaze_waypoint_following waypoint_service
```

**Ros Topics:**

The node will only send waypoints if the following messages have been received:

| topic                      | publisher / subscriber | type                       | content                                                      |
| -------------------------- | ---------------------- | -------------------------- | ------------------------------------------------------------ |
| mavros/local_position/pose | subscriber             | geometry_msgs::PoseStamped | Current pose of the vehicle                                  |
| mavros/state               | subscriber             | mavros_msgs::State         | Current state of the vehicle (check if armed)                |
| mavros/extended_state      | subscriber             | mavros_msgs::ExtendedState | Current estended state of the vehicle (check if landed or in air) |

**Additional Topics:**

| topic                          | publisher / subscriber | type                                 | content                                  |
| ------------------------------ | ---------------------- | ------------------------------------ | ---------------------------------------- |
| mavros/setpoint_position/local | publisher              | geometry_msgs::PoseStamped           | Publishing of the waypoint               |
| mavros/cmd/arming              | service client         | geometry_msgs::PoseStamped           | Used for arming the vehicle              |
| mavros/cmd/land                | service client         | mavros_msgs::CommandBool             | Used to send land comand to the vehicle  |
| mavros/set_mode                | service client         | mavros_msgs::CommandTOL              | Used to enable the offboard mode         |
| wp_service                     | service server         | amaze_waypoint_following::wp_service | Own service to receive waypoint requests |

## Usage (waypoint_service_client)
**Run the waypoint client:**

```sh
rosrun amaze_waypoint_following waypoint_service_client mode x y z yaw
```

| Parameter |                                                              |
| --------- | ------------------------------------------------------------ |
| mode      | **0:** Landing **1:** Absolute waypoint **2:** Relative waypoint (to be implemented) |
| x,y,z [m] | 3D coordinates of the waypoint                               |
| yaw [deg] | Yaw of of the Vehicle                                        |

**Ros Topics:**

| topic      | publisher / subscriber | type                                 | content               |
| ---------- | ---------------------- | ------------------------------------ | --------------------- |
| wp_service | service client         | amaze_waypoint_following::wp_service | Requesting a waypoint |

## References

Arming and switching to offboard mode: https://dev.px4.io/master/en/ros/mavros_offboard.html
