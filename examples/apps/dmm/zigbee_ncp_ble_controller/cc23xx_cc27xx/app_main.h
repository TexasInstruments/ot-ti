#ifndef BLE_CONTROLLER_APP_H
#define BLE_CONTROLLER_APP_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ti/ble/stack_util/comdef.h"

void BLEController_main(uint32 syncInitTimeoutTicks);

#ifdef __cplusplus
}
#endif
#endif //BLE_CONTROLLER_APP_H
