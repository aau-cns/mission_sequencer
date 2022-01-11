// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef MS_SEQUENCER_OPTIONS_HPP_
#define MS_SEQUENCER_OPTIONS_HPP_

#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include "types/sequencer_types.hpp"

namespace mission_sequencer
{
struct MissionSequencerOptions
{
  enum class BoundReference
  {
    LOCAL,
    GLOBAL,
  };

  friend inline std::ostream& operator<<(std::ostream& os, BoundReference ref)
  {
    switch(ref)
    {
      case BoundReference::GLOBAL:
        return os << "GLOBAL";
      case BoundReference::LOCAL:
        return os << "LOCAL";
    }

    return os;
  }

  // ==========================================================================
  // NAVIGATION ===============================================================
  /// threshold in position to determine if waypoint was reached in meters
  double threshold_position_{ 0.3 };

  /// threshold in yaw (attitude) to determine if waypoint was reached in radian
  double threshold_yaw_{ 0.1 };

  /// type of takeoff to use
  /// \warning TakeoffType::VELOCITY has no takeoff implementation yet
  TakeoffType takeoff_type_{ TakeoffType::POSITION };

  /// takeoff height/velocity in z-direction in meters (per second)
  /// the interpretation depends on the TakeoffType chosen in MissionSequencerOptions::takeoff_type_
  double takeoff_z_{ 1.0 };

  /// path to file where potential waypoints are stored
  /// this will only be used if MissionSequencerOptions::b_wp_from_file_ is set to true
  std::string filename_wps_{ "" };

  /// maximum boundary for new waypoints (w.r.t. starting pose or global)
  Eigen::Vector3d bound_max_{ 1.0, 1.0, 1.0 };

  /// minimum boundary for new waypoints (w.r.t. starting pose or global)
  Eigen::Vector3d bound_min_{ -1.0, -1.0, 0.0 };

  /// reference frame of boundary
  BoundReference bound_ref_{ BoundReference::GLOBAL };

  inline void printNavigation()
  {
    ROS_INFO_STREAM("==> sequencer_options: Parameter Summary -- Navigation");
    ROS_INFO_STREAM("\t- threshold_position_:         " << threshold_position_);
    ROS_INFO_STREAM("\t- threshold_yaw_:              " << threshold_yaw_);
    ROS_INFO_STREAM("\t- takeoff_type_:               " << takeoff_type_);
    ROS_INFO_STREAM("\t- takeoff_z_:                  " << takeoff_z_);
    ROS_INFO_STREAM("\t- filename_wps_:               " << filename_wps_);
    ROS_INFO_STREAM("\t- bound_max_:                  " << bound_max_.transpose());
    ROS_INFO_STREAM("\t- bound_min_:                  " << bound_min_.transpose());
    ROS_INFO_STREAM("\t- bound_ref_:                  " << bound_ref_);
  }

  // ==========================================================================
  // SEQUENCER OPTIONS ========================================================

  /// flag to determine if mission sequencer should automatically switch states
  /// if 'true' this overwrites MissionSequencerOptions::b_do_automatically_land_ and
  /// MissionSequencerOptions::b_do_automatically_disarm_
  bool b_do_autosequence_{ false };

  /// flag to determine if mission sequencer should automatically land once all waypoints have been reached
  /// \attention disable this if you want to fly with manual inputs
  bool b_do_automatically_land_{ false };

  /// flag to determine if mission sequencer should automatically disarm once landed
  bool b_do_automatically_disarm_{ false };

  /// flag to determine if received WP are relative to starting pose
  /// the starting pose is determined when the TAKEOFF request has been sent
  bool b_wp_are_relative_{ false };

  /// flag to determine if waypoints are read from a file
  /// this requires filname_wps_ to include a filename or transmition of file parameters through the file topic
  bool b_wp_from_file_{ false };

  /// flag to determine if additional verbose output is turned on
  /// \deprecated this will be removed and switched to ROS_DEBUG_STREAM for verbose output
  bool b_do_verbose{ false };

  inline void printSequencer()
  {
    ROS_INFO_STREAM("==> sequencer_options: Parameter Summary -- Sequencer");
    ROS_INFO_STREAM("\t- b_do_autosequence_:          " << b_do_autosequence_);
    ROS_INFO_STREAM("\t- b_do_automatically_land_:    " << b_do_automatically_land_);
    ROS_INFO_STREAM("\t- b_do_automatically_disarm_:  " << b_do_automatically_disarm_);
    ROS_INFO_STREAM("\t- b_wp_are_relative_:          " << b_wp_are_relative_);
    ROS_INFO_STREAM("\t- b_wp_from_file_:             " << b_wp_from_file_);
    ROS_INFO_STREAM("\t- b_do_verbose:                " << b_do_verbose);
  }

  // ==========================================================================
  // ROS OPTIONS ==============================================================

  /// ROS service used for arming the vehicle
  std::string srv_cmd_arming_{ "/mavros/cmd/arming" };

  /// ROS service used for disarming the vehicle
  std::string srv_cmd_command_{ "/mavros/cmd/command" };

  /// ROS service used for issuing land command
  std::string srv_cmd_land_{ "/mavros/cmd/land" };

  /// ROS service used to set the mission mode
  std::string srv_cmd_set_mode_{ "/mavros/set_mode" };

  inline void printROS()
  {
    ROS_INFO_STREAM("==> sequencer_options: Parameter Summary -- ROS");
    ROS_INFO_STREAM("\t- srv_cmd_arming_:             " << srv_cmd_arming_);
    ROS_INFO_STREAM("\t- srv_cmd_command_:            " << srv_cmd_command_);
    ROS_INFO_STREAM("\t- srv_cmd_land_:               " << srv_cmd_land_);
    ROS_INFO_STREAM("\t- srv_cmd_set_mode_:           " << srv_cmd_set_mode_);
  }

  // ==========================================================================
  // DEBUG OPTIONS ============================================================

  /// determines the time period for writing and saving ROS debug messages
  static constexpr double topic_debug_interval_ = 4.0;

  inline void printDebug()
  {
    ROS_DEBUG_STREAM("==> sequencer_options: Parameter Summary -- ROS");
    ROS_DEBUG_STREAM("\t- topic_debug_interval_:       " << topic_debug_interval_);
  }

};  // class MissionSequencerOptions
}  // namespace mission_sequencer

#endif  // MS_SEQUENCER_OPTIONS_HPP_
