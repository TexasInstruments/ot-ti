/******************************************************************************
 @file  dmm_priority_ble_thread_zigbee.c
 @brief DMM Global Priority Table: BLE Peripheral + Thread FTD + Zigbee Router.

 Activity codes:
   BLE    — DMMStackActivityBLE    (conn=0x7D0, adv=0xBB8, scan=0xFA0, ...)
   Thread — DMMStackActivityThread (TX_DATA=1, TX_POLL=2, RX_POLL=3, RX_SCAN=4, RX_IDLE=5)
   Zigbee — macActivity_t          (TX_LINK_EST=1, TX_BEACON=2, TX_DATA=3, RX_SCAN=4,
                                    RX_POLLING=5, RX_IDLE=6)
 ******************************************************************************/

#include <dmm_policy.h>
#include "dmm_priority_ble_thread_zigbee.h"

typedef enum
{
    DMM_BLE_CONN_INITIATING     = 0x3E8,
    DMM_BLE_ACTIVE_CONNECTION   = 0x7D0,
    DMM_BLE_ADVERTISING         = 0xBB8,
    DMM_BLE_SCANNING            = 0xFA0,
    DMM_BLE_PERIODIC_SCANNING   = 0x1388,
    DMM_BLE_PERIODIC_ADVERTISING = 0x1770,
} DMMStackActivityBLE;

typedef enum
{
    DMM_THREAD_TX_DATA  = 0x0001,
    DMM_THREAD_TX_POLL  = 0x0002,
    DMM_THREAD_RX_POLL  = 0x0003,
    DMM_THREAD_RX_SCAN  = 0x0004,
    DMM_THREAD_RX_IDLE  = 0x0005,
} DMMStackActivityThread;

typedef enum
{
    DMM_ZIGBEE_TX_LINK_EST = 0x0001,
    DMM_ZIGBEE_TX_BEACON   = 0x0002,
    DMM_ZIGBEE_TX_DATA     = 0x0003,
    DMM_ZIGBEE_RX_SCAN     = 0x0004,
    DMM_ZIGBEE_RX_POLLING  = 0x0005,
    DMM_ZIGBEE_RX_IDLE     = 0x0006,
} DMMStackActivityZigbee;

/* BLE global priorities */
static StackActivity activityBLE[ACTIVITY_NUM_BLE * PRIORITY_NUM] =
{
    DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION,    DMM_StackPNormal, 85),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION,    DMM_StackPHigh,  170),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION,    DMM_StackPUrgent,250),

    DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING,      DMM_StackPNormal,195),
    DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING,      DMM_StackPHigh,  195),
    DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING,      DMM_StackPUrgent,220),

    DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING,          DMM_StackPNormal, 60),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING,          DMM_StackPHigh,  160),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING,          DMM_StackPUrgent,210),

    DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING,             DMM_StackPNormal, 70),
    DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING,             DMM_StackPHigh,   70),
    DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING,             DMM_StackPUrgent, 80),

    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING,    DMM_StackPNormal, 80),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING,    DMM_StackPHigh,  180),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING,    DMM_StackPUrgent,240),

    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPNormal, 65),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPHigh,  165),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPUrgent,215),
};

/* Thread global priorities */
static StackActivity activityThread[ACTIVITY_NUM_THREAD * PRIORITY_NUM] =
{
    DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_DATA, DMM_StackPNormal,150),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_DATA, DMM_StackPHigh,  200),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_DATA, DMM_StackPUrgent,240),

    DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_POLL, DMM_StackPNormal,100),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_POLL, DMM_StackPHigh,  200),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_TX_POLL, DMM_StackPUrgent,230),

    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_POLL, DMM_StackPNormal,100),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_POLL, DMM_StackPHigh,  150),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_POLL, DMM_StackPUrgent,160),

    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_SCAN, DMM_StackPNormal, 90),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_SCAN, DMM_StackPHigh,  190),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_SCAN, DMM_StackPUrgent,215),

    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_IDLE, DMM_StackPNormal, 65),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_IDLE, DMM_StackPHigh,   75),
    DMM_GLOBAL_PRIORITY(DMM_THREAD_RX_IDLE, DMM_StackPUrgent, 85),
};

/* Zigbee global priorities (mirrors Thread activities for equal 802.15.4 peer status) */
static StackActivity activityZigbee[ACTIVITY_NUM_ZIGBEE * PRIORITY_NUM] =
{
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_LINK_EST, DMM_StackPNormal,115),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_LINK_EST, DMM_StackPHigh,  215),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_LINK_EST, DMM_StackPUrgent,240),

    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_BEACON,   DMM_StackPNormal,114),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_BEACON,   DMM_StackPHigh,  215),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_BEACON,   DMM_StackPUrgent,240),

    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_DATA,     DMM_StackPNormal,165),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_DATA,     DMM_StackPHigh,  215),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_DATA,     DMM_StackPUrgent,250),

    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_SCAN,     DMM_StackPNormal, 105),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_SCAN,     DMM_StackPHigh,   205),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_SCAN,     DMM_StackPUrgent, 225),

    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_POLLING,  DMM_StackPNormal,100),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_POLLING,  DMM_StackPHigh,  200),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_POLLING,  DMM_StackPUrgent,230),

    /* Zigbee RX_IDLE must beat Thread's escalated RX_IDLE max (75) so the
     * Zigbee router can always preempt Thread's idle to receive ZCL ACKs.
     * Thread/BLE data TX (150+) still outranks this idle priority. */
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_IDLE,     DMM_StackPNormal, 70),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_IDLE,     DMM_StackPHigh,   80),
    DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_IDLE,     DMM_StackPUrgent, 90),
};

/* The order here MUST match the stackRoles order in the SysConfig:
 * blePeripheral (index 0), threadFTD (index 1), zigbeeRouter (index 2).
 *
 * DMMPOLICY_NUM_STACKS is overridden to 3 (from the SDK default of 2) when
 * OT_APP_ZIGBEE_NCP_OT_NCP_BLE_CONTROLLER is set — see src/CMakeLists.txt.
 * This array has exactly 3 entries to match; no padding slot is needed. */
GlobalTable globalPriorityTable_bleThreadZigbeeH[DMMPOLICY_NUM_STACKS] =
{
    {
        .globalTableArray = activityBLE,
        .tableSize        = (uint8_t)(ACTIVITY_NUM_BLE * PRIORITY_NUM),
        .stackRole        = DMMPolicy_StackRole_BlePeripheral,
    },
    {
        .globalTableArray = activityThread,
        .tableSize        = (uint8_t)(ACTIVITY_NUM_THREAD * PRIORITY_NUM),
        .stackRole        = DMMPolicy_StackRole_ThreadFtd,
    },
    {
        .globalTableArray = activityZigbee,
        .tableSize        = (uint8_t)(ACTIVITY_NUM_ZIGBEE * PRIORITY_NUM),
        .stackRole        = DMM_ROLE_ZIGBEE,
    },
};
