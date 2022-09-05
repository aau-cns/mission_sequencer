// Copyright (C) 2022 Martin Scheiber, Christoph Boehm,
// and others, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <martin.scheiber@ieee.org>,
// and <christoph.boehm@aau.at>

#include "mission_sequencer.hpp"

int main(int argc, char** argv)
{
  std::string name(argv[0]);
  ROS_INFO("%s started", name.c_str());
  ros::init(argc, argv, name.c_str());

#ifdef NDEBUG
  // nondebug
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#else
  // debug
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  mission_sequencer::MissionSequencer sequencer(nh, private_nh);

  ros::Rate rate(20.0);

  while (ros::ok())
  {
    ros::spinOnce();    // perform callback first such that topics are up to date
    sequencer.logic();  // Non-blocking logic ;)
    sequencer.publishPoseSetpoint();
    rate.sleep();
  }

  // // Check for new messages and loop
  // ros::spin();

  return 0;
}
