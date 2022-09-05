// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef MS_MESSAGE_CONVERSION_HPP_
#define MS_MESSAGE_CONVERSION_HPP_

#include <mission_sequencer/MissionWaypointArray.h>

#include "types/sequencer_waypoint.hpp"

namespace mission_sequencer
{
class MSMsgConv
{
public:
  static std::vector<Waypoint>
  WaypointArray2WaypointList(const std::vector<mission_sequencer::MissionWaypoint>& waypoint_array,
                             const Waypoint::ReferenceFrame ref_frame = Waypoint::ReferenceFrame::LOCAL)
  {
    std::vector<Waypoint> waypoint_list;
    for (const auto wp : waypoint_array)
    {
      waypoint_list.push_back(Waypoint(wp.x, wp.y, wp.z, wp.yaw, wp.holdtime, ref_frame));
    }

    return waypoint_list;
  }
};  // MSMsgConv
}  // namespace mission_sequencer

#endif  // MS_MESSAGE_CONVERSION_HPP_
