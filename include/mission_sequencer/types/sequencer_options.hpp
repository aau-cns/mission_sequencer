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

#include "types/sequencer_types.hpp"

namespace mission_sequencer
{
struct MissionSequencerOptions
{
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
  /// the interpretation depends on the TakeoffType chosen in takeoff_type_
  double takeoff_z_{ 1.0 };

  /// path to file where potential waypoints are stored
  /// this will only be used if b_wp_from_file_ is set to true
  std::string filename_wps_{ "" };

  void printNavigation()
  {
  }

  // ==========================================================================
  // SEQUENCER OPTIONS ========================================================

  bool b_do_autosequence_{ false };
  bool b_do_automatically_land_{ false };
  bool b_do_automatically_disarm_{ false };
  bool b_wp_are_relative_{ false };
  bool b_wp_from_file_{ false };
  bool b_do_verbose{ false };

  void printSequencer()
  {
  }

  // ==========================================================================
  // ROS OPTIONS ==============================================================

  std::string srv_cmd_arming_{ "/mavros/cmd/arming" };
  std::string srv_cmd_command_{ "/mavros/cmd/command" };
  std::string srv_cmd_land_{ "/mavros/cmd/land" };
  std::string srv_cmd_set_mode_{ "/mavros/set_mode" };

};  // class MissionSequencerOptions
}  // namespace mission_sequencer

#endif  // MS_SEQUENCER_OPTIONS_HPP_
