# CNS FlightStack: Mission Sequencer

[![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{cns_flightstack22,
   author   = {Martin Scheiber and Alessandro Fornasier and Roland Jung and Christoph Boehm and Rohit Dhakate
               and Christian Stewart and Jan Steinbrener and Stephan Weiss and Christian Brommer},
   journal  = {under review},
   title    = {Flight Stack for Reproducible and Customizable Autonomy Applications in Research and Industry},
   year     = {2022},
}
```

## Getting Started

### Prerequesites
This package is part of the [CNS FlightStack] and thus depends on the other packages of the flightstack:
- [CNS FlightStack: Autonomy Engine]
- MavROS
- PX4Bridge

Further the following libraries are required
- Eigen
- ROS noetic



### Build

As this is a ROS package, please build it within the catkin environment with

```bash
catkin build mission_sequencer
```

## Usage

The intended usage is together with the [CNS FlightStack: Autonomy Engine], which will interact with the mission sequencer. Use the provided launchfile to start the Mission Sequencer

```bash
roslaunch mission_sequencer mission_sequencer.launch
```

The following parameters can be set to modify the sequencer's behavior:

### ROS Parameters

| ROS parameter | description | default value |
|---------------|-------------|---------------|
| `threshold_position_m`    | threshold for position upon which a WP is accepted | `0.3` |
| `threshold_yaw_rad`       | threshold for yaw upon which a WP is accepted | `0.1` |
| `takeoff_type`            | type for takeoff (`POSITION=0` or `VELOCITY=1`) | `0` |
| `takeoff_z_m`             | height for takeoff in `POSITION`-mode | `1` |
| `bound_max`               | boundaries (max) for xyz values | `[1.0,1.0,1.0]` |
| `bound_min`               | boundaries (min) for xyz values | `[-1.0,-1.0,0.0]` |
| `boundary_reference`      | reference frame for boundaries (`LOCAL` or `GLOBAL`) | `LOCAL` |
| `do_automatic_sequence`   | automatic state changes in sequencer (if used without Autonomy Engine) | `false` |
| `do_automatically_disarm` | automatically disarm when MavROS detects landing | `false` |
| `do_automatically_land`   | automatically land when last waypoint was reached | `false` |
| `wp_are_relative`         | general interpretation that waypoints are relative | `false` |
| `do_hold_at_zero_vel`     | when holding use velocity to reduce speed and hold when zero velocity reached | `true` |
| `do_predict_hold_wp`      | (_experimental_) predict hold position given current velocity | `false` |
| `topic_in_pose`           | topic name for pose subscriber | `ref/pose` |
| `topic_in_odom`           | topic name for odometry subscriber | `ref/odom` |
| `verbose`                 | (_deprecated_) enable verbose flag, replace by `ROS_DEBUG` | `false` |
| `wp_from_file`            | (_deprecated_) read waypoints from file | `false` |
| `waypoint_filename`       | (_deprecated_) filename to read waypoints from  | ` ` |

### Default Launchfile Parameters

TODO

### Usage without Autonomy Engine

If required the sequencer can be used without the [CNS FlightStack: Autonomy Engine]. The easiest way to do this is to enable the automatic sequencing, i.e. setting `do_automatic_sequence` to `true`. Then, when ready start the mission by publishing the `ARM` request:

```bash
rostopic pub /autonomy/request mission_sequencer/MissionRequest \
"{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, id: 0, request: 1}" -1
```

If the automatic sequencing shall remain disabled, one can still execute the requests manually. To do so use the above publication as a template. Further the request type definition can be found in [`msg/MissionRequest.msg`](msg/MissionRequest.msg):

```
# request type definition
uint8 UNDEF   = 0 # Not used for request, used only for MS -> A communication without a specific request
uint8 ARM     = 1 # Arming (no takeoff)
uint8 TAKEOFF = 2 # Takeoff
uint8 HOLD    = 3 # mission hold request -- blocking, hold no until a resume request
uint8 RESUME  = 4 # resume mission request
uint8 LAND    = 5 # land request
uint8 HOVER   = 6 # hover request -- non blocking, when a new waypoint is received fly to it
uint8 ABORT   = 7 # mission aborted request - safety pilot will take manual control
uint8 DISARM  = 8 # Disarming
```

### Waypoint Modification

By default the Mission Sequencer gets the waypoints from the Autonomy [CNS FlightStack: Autonomy Engine]. However, any node can publish waypoints on the `/mission_sequencer/waypoint_list` topic (message definition [`msg/MissionWaypointArray.msg`](msg/MissionWaypointArray.msg)). Here the following options exist:

| Waypoint Action | description |
|---------------|-------------|
| `CLEAR  = 0`    | clear the currently buffered list of WPs and update to the provided ones |
| `APPEND = 1`    | append the provided WPs to the currently buffered list |
| `INSERT = 2`    | insert the provided WPs at index `idx` of the currently buffered list; if the `idx` exceeds the size fo the buffered list, the provided WPs will be appended |

| Reference Frame | description |
|---------------|-------------|
| `GLOBAL  = 0`    | the provided WPs are interpreted in the 'global' navigation frame |
| `LOCAL   = 1`    | the provided WPs are interpreted w.r.t. the starting pose, i.e. 'local frame' |
| `CUR_POS = 2`    | the provided WPs are interpreted w.r.t. to the current position, i.e. added to the current position, when they are evaluated |
| `CUR_POSE = 3`   | the provided WPs are interpreted w.r.t. to the current pose (position + yaw), i.e. added to the current position and yaw, when they are evaluated |

If required for your node (obstacle detection), you can get the current list of waypoints with the service: TODO

<!-- Command to set filepath to test_trajectory.csv:
```cmd
rosparam set /autonomy/missions/mission_0/filepaths ["/home/chriboehm/workspaces/mission_ws/src/amaze_mission_sequencer/trajectories/test_trajectory.csv"]
``` -->

## Architecture

Please refer to the academic paper for further insights of the Mission Sequencer.

TODO: include the state machine graph here

## Known Issues

None at this point in time, please submit an issue request using the git interface if any issues arise.

## Package Layout

```console
/path/to/mission_sequencer$ tree -L 3 --noreport --charset unicode
.
|-- cmake
|   |-- Custom.cmake
|   |-- GetGitRevisionDescription.cmake
|   `-- GetGitRevisionDescription.cmake.in
|-- CMakeLists.txt
|-- docs
|   |-- CMakeLists.txt
|   |-- pkg-docs
|   `-- resources
|-- include
|   `-- mission_sequencer
|       |-- mission_sequencer.hpp
|       |-- parse_waypoints.hpp
|       |-- types
|       `-- utils
|-- launch
|   `-- mission_sequencer.launch
|-- LICENSE.md
|-- msg
|   |-- MissionRequest.msg
|   |-- MissionResponse.msg
|   |-- MissionWaypointArray.msg
|   `-- MissionWaypoint.msg
|-- package.xml
|-- README.md
|-- src
|   |-- mission_sequencer.cpp
|   |-- mission_sequencer_node.cpp
|   `-- parse_waypoints.cpp
|-- srv
|   `-- GetStartPose.srv
`-- trajectories
    `-- test_trajectory.csv
```


<!-- LINKS: -->
[CNS FlightStack]: http://sst.aau.at/cns
[CNS FlightStack: Autonomy Engine]: http://sst.aau.at/cns
