// Copyright (C) 2022 Alessandro Fornasier, Martin Scheiber,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@aau.at>
// and <martin.scheiber@aau.at>.

#ifndef MS_SEQUENCER_WAYPOINT_HPP_
#define MS_SEQUENCER_WAYPOINT_HPP_

#include <sstream>
#include <string>

namespace mission_sequencer
{
///
/// \brief The Waypoint struct for saving basic waypoint information
///
struct Waypoint
{
  ///
  /// \brief The ReferenceFrame enum describes the relative frame to which new waypoints are interpreted to
  ///
  enum class ReferenceFrame
  {
    GLOBAL = 0,   //!<  in global frame
    LOCAL = 1,    //!< relative to starting position and yaw
    CUR_POS = 2,  //!< relative to current position
    CUR_POSE = 3  //!< relative to current position and yaw
  };

  inline friend std::ostream& operator<<(std::ostream& os, ReferenceFrame ref)
  {
    switch (ref)
    {
      case ReferenceFrame::GLOBAL:
        return os << "GLOBAL ";
      case ReferenceFrame::LOCAL:
        return os << "LOCAL  ";
      case ReferenceFrame::CUR_POS:
        return os << "CURPOS ";
      case ReferenceFrame::CUR_POSE:
        return os << "CURPOSE";
        // omit default case to trigger compiler warning for missing cases
    }

    return os;
  }

  double x{ 0.0 };                                     //!< Waypoint x coordinate
  double y{ 0.0 };                                     //!< Waypoint y coordinate
  double z{ 0.0 };                                     //!< Waypoint z coordinate
  double yaw{ 0.0 };                                   //!< Waypoint yaw (z-rotation)
  double holdtime{ 0.0 };                              //!< Time to stay at waypoint
  ReferenceFrame ref_frame{ ReferenceFrame::GLOBAL };  //!< Reference coordinate system (frame)

  ///
  /// \brief Default constructor for Waypoint, initializing all components to 0.0 and frame ReferenceFrame::GLOBAL.
  ///
  Waypoint() = default;

  ///
  /// \brief Waypoint Advanced constructor for Waypoint
  /// \param _x Waypoint x coordinate
  /// \param _y Waypoint y coordinate
  /// \param _z Waypoint z coordinate
  /// \param _yaw Waypoint yaw (z-rotation)
  /// \param _holdtime Time to stay at waypoint
  /// \param _ref_frame Reference coordinate system (frame)
  ///
  Waypoint(double _x, double _y, double _z, double _yaw, double _holdtime, ReferenceFrame _ref_frame)
    : x(_x), y(_y), z(_z), yaw(_yaw), holdtime(_holdtime), ref_frame(_ref_frame){};
};

}  // namespace mission_sequencer

#endif  // MS_SEQUENCER_WAYPOINT_HPP_
