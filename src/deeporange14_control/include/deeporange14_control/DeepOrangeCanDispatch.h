/*********************************************************************
 Refer to FORT SRC Manual for description
 Refer to Deep Orange used CAN messages for description
 *********************************************************************/

#ifndef DEEPORANGE_CAN_DISPATCH_H_
#define DEEPORANGE_CAN_DISPATCH_H_
#include <stdint.h>

namespace deeporange14 {
#undef BUILD_ASSERT

enum {
  ID_AUTONOMY_MEAS  = 0x51,
  ID_AUTONOMY_CMD   = 0x50,
};

}  // namespace deeporange14

#endif  // DEEPORANGE_CAN_DISPATCH_H_
