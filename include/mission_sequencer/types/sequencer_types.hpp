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

#include <iostream>

namespace mission_sequencer
{

///
/// \brief The SequencerState enum
///
/// \todo make include for magic_enum
/// \see https://github.com/Neargye/magic_enum
///
enum class SequencerState
{
  IDLE,
  PREARM,
  ARM,
  TAKEOFF,
  HOVER,
  MISSION,
  HOLD,
  LAND,
  DISARM
};

enum class TakeoffType
{
  POSITION,
  VELOCITY
};

static std::ostream& operator<<(std::ostream& os, SequencerState state)
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
      // omit default case to trigger compiler warning for missing cases
  }
}


}  // namespace mission_sequencer

#endif  // MS_SEQUENCER_TYPES_HPP_
