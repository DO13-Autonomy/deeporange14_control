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
  AU_0_DEFAULT                   = 0,
  AU_1_STARTUP                   = 1,
  AU_2_IDLE                      = 2,
  AU_3_ROS_MODE_EN               = 3,
  AU_4_DISENGAGING_BRAKES        = 4,
  AU_5_ROS_CONTROLLED            = 5,

  // Raptor States
  DBW_3_ROS_EN                   = 3,
  DBW_4_ROS_CONTROLLED           = 4,

  SPEED_STATE_Ready2Move         = 2,

  // linear velocity
  VEHICLE_STOPPED                = 0,
  VEHICLE_ACCELERATING           = 1,
  VEHICLE_MOVING                 = 2
};
}  // namespace deeporange14

#endif  // DEEPORANGE_STATE_ENUMS_H_
