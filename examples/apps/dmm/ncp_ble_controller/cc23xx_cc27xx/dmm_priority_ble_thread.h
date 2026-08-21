/******************************************************************************

 @file dmm_priority_ble_zigee.h

 @brief Global Priority DMM use cases

 Group: WCS LPC
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2017 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/
 /*!****************************************************************************
 *  @file       dmm_priority_ble_zigee.h
 *
 *  @brief      Global Priority Table
 *
 *  DMM enables devices to run multiple wireless protocol stacks concurrently.
 *  The DMMSch is to enable concurrent operation of multiple stacks with minimized conflicts
 *  so that it does not cause significant performance degradation.
 *  The DMMSch uses Application Level, Stack Level information, and Global Priority Table (GPT) when scheduling a command.
 *  Stack Level information (provided by stack) is embedded in each RF command and it includes:
 *  - Start Type, Start Time, AllowDelay, Priority, Activity, etc.
 *  Application Level information (provided by User via the Policy Table) includes:
 *  - Application State Name, Weight, AppliedActivity, Pause, etc.
 *  Global Priority Table (GPT)
 *  - GPT defines relative priorities of the two stacks
 *  - GPT consists of three parameters: Stack Activity, Priority of the activity (Normal, High, Urgent), and Global Priority Number
 *  - GPT input: Stack Activity  + Priority of the activity (Normal, High, Urgent)
 *  - GPT output: Global Priority Number
 *  Final Priority = GPT (Stack Activity  + Priority of the activity) +  .weight (in the Policy Table)
 *
 *  # GPT for BLE Stack + Thread Stack #
 *  ___________________________________________________________________________________________________
 *  | BLE Activity + Priority        | 15.4 Activity + Priority         | GTP value
 *  ____________________________________________________________________________________________________
 *  | Connect             - Normal   |                                  | 75
 *  ____________________________________________________________________________________________________
 *
 *  # GPT APIs are defined in the dmm_policy.c#
 *    - DMMPolicy_getGlobalPriority(): Get the global activity based on stack activity.
 *    - DMMPolicy_getDefaultPriority(): Get the default priority of the stack
 *    - DMMPolicy_getGPTStatus():  check if the global priority table is available
 *
 *
 * ********************************************************************************/
#ifndef dmm_priority_ble_thread__H_
#define dmm_priority_ble_thread__H_

#include "dmm_policy.h"

//! \brief The number of activities
#define ACTIVITY_NUM_BLE                        6
#define ACTIVITY_NUM_THREAD                     5

extern GlobalTable globalPriorityTable_bleLthreadH[DMMPOLICY_NUM_STACKS];

#endif //dmm_priority_ble_thread__H_
