/*********************************************************************
 Refer to FORT SRC Manual for description
 Refer to Deep Orange used CAN messages for description
 *********************************************************************/

#ifndef DEEPORANGE_CAN_DISPATCH_H_
#define DEEPORANGE_CAN_DISPATCH_H_
#include <stdint.h>

namespace deeporange14
{
#undef BUILD_ASSERT

enum
{
  // Raptor msgs
  ID_Raptor_Main_Msg              = 0x34,
  ID_VD_Brake_Msg                 = 0x46,

  // ROS msgs
  ID_AU_STATUS_MSG                = 0x385,
  ID_AU_CONTROL_MSG               = 0x386,
};

}  // namespace deeporange14

#endif  // DEEPORANGE_CAN_DISPATCH_H_
