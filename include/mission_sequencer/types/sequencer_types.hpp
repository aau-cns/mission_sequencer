// Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@aau.at>

#ifndef MS_SEQUENCER_TYPES_HPP_
#define MS_SEQUENCER_TYPES_HPP_

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

#include <iostream>

namespace mission_sequencer
{
///
/// \brief The SequencerState enum describes the current state of the MissionSequencer
///
/// \todo make include for magic_enum
/// \todo create enum for SequencerRequests (including READ & RESUME)
/// \see https://github.com/Neargye/magic_enum
///
enum class SequencerState
{
  IDLE,     //!< IDLE state, when sequencer has not been started
  PREARM,   //!< PREARM state, when sequencer has loaded waypoint file and is ready to arm for WP takeoff
  ARM,      //!< ARM state, when vehicle is armed (rotors spinning) but has not yet taken off
  TAKEOFF,  //!< TAKEOFF state, when vehicle is in TO phase
  HOVER,    //!< HOVER state, when vehicle is hovering (holding position until new WPs arrive)
  MISSION,  //!< MISSION state, when vehicle is flying to WP
  HOLD,     //!< HOLD state, when vehicle is holding position until RESUME request arrives
  LAND,     //!< LAND state, when vehicle is in landing phase
  DISARM,   //!< DISARM state, when vehicle is tring to disarm (turn off motors)

  /// \deprecated will be moved to individual enum soon
  RESUME
};

///
/// \brief The TakeoffType enum describes the type of takeoff control in use
///
enum class TakeoffType
{
  POSITION,  //!< POSITION based control takeoff type
  VELOCITY   //!< VELOCITY based control takeoff type
};

inline static std::ostream& operator<<(std::ostream& os, SequencerState state)
{
  switch (state)
  {
    case SequencerState::IDLE:
      return os << "IDLE";
    case SequencerState::PREARM:
      return os << "PREARM";
    case SequencerState::ARM:
      return os << "ARM";
    case SequencerState::TAKEOFF:
      return os << "TAKEOFF";
    case SequencerState::HOVER:
      return os << "HOVER";
    case SequencerState::MISSION:
      return os << "MISSION";
    case SequencerState::HOLD:
      return os << "HOLD";
    case SequencerState::LAND:
      return os << "LAND";
    case SequencerState::DISARM:
      return os << "DISARM";
    case SequencerState::RESUME:
      return os << "RESUME";
      // omit default case to trigger compiler warning for missing cases
  }

  return os;
}

inline static std::ostream& operator<<(std::ostream& os, TakeoffType type)
{
  switch (type)
  {
    case TakeoffType::POSITION:
      return os << "POSITION";
    case TakeoffType::VELOCITY:
      return os << "VELOCITY";
      // omit default case to trigger compiler warning for missing cases
  }

  return os;
}

}  // namespace mission_sequencer

#endif  // MS_SEQUENCER_TYPES_HPP_
