/******************************************************************************
 @file  dmm_priority_ble_thread_zigbee.h
 @brief DMM Global Priority Table for BLE Peripheral + Thread FTD + Zigbee Router.
 ******************************************************************************/

#ifndef DMM_PRIORITY_BLE_THREAD_ZIGBEE_H
#define DMM_PRIORITY_BLE_THREAD_ZIGBEE_H

#include "dmm_policy.h"

#define ACTIVITY_NUM_BLE     6
#define ACTIVITY_NUM_THREAD  5
#define ACTIVITY_NUM_ZIGBEE  6

/* Zigbee uses ZigbeeRouter as the DMM role (host selects ZC/ZR/ZED at runtime). */
#define DMM_ROLE_ZIGBEE  DMMPolicy_StackRole_ZigbeeRouter

extern GlobalTable globalPriorityTable_bleThreadZigbeeH[DMMPOLICY_NUM_STACKS];

#endif /* DMM_PRIORITY_BLE_THREAD_ZIGBEE_H */
