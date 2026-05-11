/******************************************************************************
 @file  dmm_priority_ble_zigbee.h
 @brief DMM Global Priority Table declarations for Zigbee NCP + BLE Controller.
        Two stacks: BLE Peripheral and Zigbee Router.
 ******************************************************************************/

#ifndef DMM_PRIORITY_BLE_ZIGBEE_H
#define DMM_PRIORITY_BLE_ZIGBEE_H

#include "dmm_policy.h"

#define ACTIVITY_NUM_BLE     6
#define ACTIVITY_NUM_ZIGBEE  6

/* Zigbee uses ZigbeeRouter as its single build-time DMM role (the NCP allows
 * the host to select ZC/ZR/ZED at run time; ZigbeeRouter is the best fit for
 * the platform library's activity codes). */
#define DMM_ROLE_ZIGBEE      DMMPolicy_StackRole_ZigbeeRouter

extern GlobalTable globalPriorityTable_bleZigbeeH[DMMPOLICY_NUM_STACKS];

#endif /* DMM_PRIORITY_BLE_ZIGBEE_H */
