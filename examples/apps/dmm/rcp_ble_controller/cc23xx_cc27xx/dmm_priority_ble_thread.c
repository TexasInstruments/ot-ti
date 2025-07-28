/******************************************************************************

 @file dmm_priority_ble_thread.c

 @brief DMM Global Priority table for OpenThread Stack and BLE Stack

 Group: WCS LPC
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2017 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

 #include <dmm_policy.h>
 #include <dmm_priority_ble_thread.h>
 
 
 /* BLE Activity */
 typedef enum
 {
     DMM_BLE_CONN_INITIATING = 0x3E8,
     DMM_BLE_ACTIVE_CONNECTION = 0x7D0,
     DMM_BLE_ADVERTISING = 0xBB8,
     DMM_BLE_SCANNING = 0xFA0,
     DMM_BLE_PERIODIC_SCANNING = 0x1388,
     DMM_BLE_PERIODIC_ADVERTISING = 0x1770,
 } DMMStackActivityBLE;
 
 /* Thread Activity */
 typedef enum
 {
     DMM_THREAD_TX_DATA    = 0x0001,
     DMM_THREAD_TX_POLL    = 0x0002,
     DMM_THREAD_RX_POLL    = 0x0003,
     DMM_THREAD_RX_SCAN    = 0x0004,
     DMM_THREAD_RX_IDLE    = 0x0005
 } DMMStackActivityThread;
 
 /* Global Priority Table: BLE connection is lower than Thread data */
 StackActivity activityBLE_bleLthreadH[ACTIVITY_NUM_BLE*PRIORITY_NUM] =
 {
      DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION, DMM_StackPNormal, 85),
      DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION, DMM_StackPHigh, 170),
      DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION, DMM_StackPUrgent, 250),
 
      DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING, DMM_StackPNormal, 195),
      DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING, DMM_StackPHigh, 195),
      DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING, DMM_StackPUrgent, 220),
 
      DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING, DMM_StackPNormal, 60),
      DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING, DMM_StackPHigh, 160),
      DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING, DMM_StackPUrgent, 210),
 
      DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING, DMM_StackPNormal, 70),
      DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING, DMM_StackPHigh, 70),
      DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING, DMM_StackPUrgent, 80),

      DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING, DMM_StackPNormal, 80),
      DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING, DMM_StackPHigh, 180),
      DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING, DMM_StackPUrgent, 240),

      DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPNormal, 65),
      DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPHigh, 165),
      DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPUrgent, 215),
 };
 
 
 StackActivity activityThread_bleLthreadH[ACTIVITY_NUM_THREAD*PRIORITY_NUM] =
 {
      DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_DATA, DMM_StackPNormal, 150),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_DATA, DMM_StackPHigh, 200),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_DATA, DMM_StackPUrgent, 240),
 
      DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_POLL, DMM_StackPNormal, 100),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_POLL, DMM_StackPHigh, 200),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_POLL, DMM_StackPUrgent, 230),
 
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_POLL, DMM_StackPNormal, 100),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_POLL, DMM_StackPHigh, 200),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_POLL, DMM_StackPUrgent, 230),
 
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_SCAN, DMM_StackPNormal, 90),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_SCAN, DMM_StackPHigh, 190),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_SCAN, DMM_StackPUrgent, 215),
 
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_IDLE, DMM_StackPNormal, 65),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_IDLE, DMM_StackPHigh, 65),
      DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_IDLE, DMM_StackPUrgent, 75),
 };
 
 /* the order of stacks in policy table and global table must be the same */
 GlobalTable globalPriorityTable_bleLthreadH[DMMPOLICY_NUM_STACKS] =
 {
      {  .globalTableArray =  activityBLE_bleLthreadH,
         .tableSize = (uint8_t)(ACTIVITY_NUM_BLE*PRIORITY_NUM),
         .stackRole = DMMPolicy_StackRole_BlePeripheral,
      },
 
      {  .globalTableArray =  activityThread_bleLthreadH,
         .tableSize = (uint8_t)(ACTIVITY_NUM_THREAD*PRIORITY_NUM),
         .stackRole = DMMPolicy_StackRole_ThreadFtd,
      },
 };