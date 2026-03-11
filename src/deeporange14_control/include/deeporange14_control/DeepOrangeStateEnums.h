/*********************************************************************
Refer to DO13 Raptor DBW state machine for info on states
 *********************************************************************/

#ifndef DEEPORANGE_STATE_ENUMS_H_
#define DEEPORANGE_STATE_ENUMS_H_
#include <stdint.h>

namespace deeporange14 {
#undef BUILD_ASSERT

enum allStates {
  // ROS States
  AU_0_NO_HEARTBEAT        = 0,
  AU_1_WAITING_HEARTBEAT   = 1,
  AU_2_WAITING_HANDOFF     = 2,
  AU_3_READY_FOR_MISSION   = 3,
  AU_4_MISSION_IN_PROGRESS = 4,

  // Raptor States
  DBW_0_AUTO_OFF           = 0,
  DBW_1_WAITING_HEARTBEAT  = 1,
  DBW_2_WAITING_DRIVE_REQ  = 2,
  DBW_3_READY_TO_DRIVE     = 3,
  DBW_4_DRIVING            = 4,
};
}  // namespace deeporange14

#endif  // DEEPORANGE_STATE_ENUMS_H_
