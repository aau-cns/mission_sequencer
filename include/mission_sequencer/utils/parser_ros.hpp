// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef MS_PARSER_ROS_HPP_
#define MS_PARSER_ROS_HPP_

#include <ros/ros.h>

#include "types/sequencer_options.hpp"

namespace mission_sequencer
{
MissionSequencerOptions parse_ros_nodehandle(ros::NodeHandle& nh)
{
  MissionSequencerOptions params;

  // ==========================================================================
  // NAVIGATION ===============================================================
  if (!nh.param<double>("threshold_position_m", params.threshold_position_, params.threshold_position_))
  {
    ROS_WARN_STREAM("=> mission_sequencer: cannot load parameter for position threshold, using default of "
                    << params.threshold_position_ << " m.");
  }
  if (!nh.param<double>("threshold_yaw_rad", params.threshold_yaw_, params.threshold_yaw_))
  {
    ROS_WARN_STREAM("=> mission_sequencer: cannot load parameter for yaw threshold, using default of "
                    << params.threshold_yaw_ << " deg.");
  }

  // check if yaw is within expected boundaries - i.e. it does not make sense to have more than 180deg=pi in threshold
  if (params.threshold_yaw_ > M_PI)
  {
    ROS_WARN_STREAM("=> mission_sequencer: yaw threshold expected in rad but was given in deg.\n"
                    << "\tautomatically converted " << params.threshold_yaw_ << " deg to "
                    << DEG_TO_RAD * params.threshold_yaw_ << " rad.");
    params.threshold_yaw_ *= DEG_TO_RAD;
  }

  // determine takeoff type
  int to_type = 0;
  nh.param<int>("takeoff_type", to_type, to_type);
  switch(to_type)
  {
    case 0:
      params.takeoff_type_=TakeoffType::POSITION;
      break;
    case 1:
      params.takeoff_type_ = TakeoffType::VELOCITY;
      break;
    default:
      ROS_WARN_STREAM("=> mission_sequencer: unknown takeoff type " << to_type <<". Defaulting to POSITION.");
      params.takeoff_type_=TakeoffType::POSITION;
      break;
  }

  nh.param<double>("takeoff_z_m", params.takeoff_z_, params.takeoff_z_);
  nh.param<std::string>("waypoint_filename", params.filename_wps_, params.filename_wps_);

  std::vector<double> bound_max, bound_min;
  nh.param<std::vector<double>>("bound_max", bound_max, { 1.0, 1.0, 1.0 });
  params.bound_max_ << bound_max.at(0), bound_max.at(1), bound_max.at(2);
  nh.param<std::vector<double>>("bound_min", bound_min, { -1.0, -1.0, 0.0 });
  params.bound_min_ << bound_min.at(0), bound_min.at(1), bound_min.at(2);

  // output navigation params
  params.printNavigation();

  // ==========================================================================
  // SEQUENCER OPTIONS ========================================================

  nh.param<bool>("do_automatically_disarm", params.b_do_automatically_disarm_, params.b_do_automatically_disarm_);
  nh.param<bool>("do_automatically_land", params.b_do_automatically_land_, params.b_do_automatically_land_);
  nh.param<bool>("wp_are_relative", params.b_wp_are_relative_, params.b_wp_are_relative_);
  nh.param<bool>("wp_from_file", params.b_wp_from_file_, params.b_wp_from_file_);
  nh.param<bool>("verbose", params.b_do_verbose, params.b_do_verbose);

  params.printSequencer();

  return params;
};  // class ParserROS
}  // namespace mission_sequencer

#endif  // MS_PARSER_ROS_HPP_
