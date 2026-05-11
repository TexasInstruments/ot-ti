/******************************************************************************

 @file  dmm_priority_ble_zigbee.c

 @brief DMM Global Priority Table for Zigbee NCP + BLE Controller.

        Two stacks: BLE Peripheral (NLI 1) and Zigbee Router (NLI 2).
        Priority values mirror the three-stack ncp_ble_controller example but
        with Thread removed.  Zigbee activity codes come from macActivity_t in
        ti/zigbee/include/ti_f3_mac_activity.h and must match that header.

 ******************************************************************************/

#include <dmm_policy.h>
#include <dmm_priority_ble_zigbee.h>

/* BLE Activity codes (from the BLE LL scheduler) */
typedef enum
{
    DMM_BLE_CONN_INITIATING    = 0x3E8,
    DMM_BLE_ACTIVE_CONNECTION  = 0x7D0,
    DMM_BLE_ADVERTISING        = 0xBB8,
    DMM_BLE_SCANNING           = 0xFA0,
    DMM_BLE_PERIODIC_SCANNING  = 0x1388,
    DMM_BLE_PERIODIC_ADVERTISING = 0x1770,
} DMMStackActivityBLE;

/*
 * Zigbee Activity.  Values match macActivity_t in ti_f3_mac_activity.h.
 * The Zigbee platform library places these codes in every RCL command's
 * activityInfo field; the DMM scheduler looks them up in this table.
 */
typedef enum
{
    DMM_ZIGBEE_TX_LINK_EST = 0x0001,  /* MAC_ACTIVITY_TX_LINK_EST */
    DMM_ZIGBEE_TX_BEACON   = 0x0002,  /* MAC_ACTIVITY_TX_BEACON   */
    DMM_ZIGBEE_TX_DATA     = 0x0003,  /* MAC_ACTIVITY_TX_DATA     */
    DMM_ZIGBEE_RX_SCAN     = 0x0004,  /* MAC_ACTIVITY_RX_SCAN     */
    DMM_ZIGBEE_RX_POLLING  = 0x0005,  /* MAC_ACTIVITY_RX_POLLING  */
    DMM_ZIGBEE_RX_IDLE     = 0x0006,  /* MAC_ACTIVITY_RX_IDLE     */
} DMMStackActivityZigbee;

/* BLE Global Priority Table: same values as the three-stack example */
StackActivity activityBLE_bleZigbeeH[ACTIVITY_NUM_BLE * PRIORITY_NUM] =
{
    DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION,    DMM_StackPNormal,  85),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION,    DMM_StackPHigh,   170),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ACTIVE_CONNECTION,    DMM_StackPUrgent, 250),

    DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING,      DMM_StackPNormal, 195),
    DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING,      DMM_StackPHigh,   195),
    DMM_GLOBAL_PRIORITY(DMM_BLE_CONN_INITIATING,      DMM_StackPUrgent, 220),

    DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING,          DMM_StackPNormal,  60),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING,          DMM_StackPHigh,   160),
    DMM_GLOBAL_PRIORITY(DMM_BLE_ADVERTISING,          DMM_StackPUrgent, 210),

    DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING,             DMM_StackPNormal,  70),
    DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING,             DMM_StackPHigh,    70),
    DMM_GLOBAL_PRIORITY(DMM_BLE_SCANNING,             DMM_StackPUrgent,  80),

    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING,    DMM_StackPNormal,  80),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING,    DMM_StackPHigh,   180),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_SCANNING,    DMM_StackPUrgent, 240),

    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPNormal,  65),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPHigh,   165),
    DMM_GLOBAL_PRIORITY(DMM_BLE_PERIODIC_ADVERTISING, DMM_StackPUrgent, 215),
};

/*
 * Zigbee Global Priority Table.
 * Initial values are a reasonable starting point; tune on hardware.
 *   TX_LINK_EST / RX_POLLING : time-sensitive link maintenance (high)
 *   TX_BEACON                : short network maintenance frames
 *   TX_DATA                  : data traffic (high)
 *   RX_SCAN                  : channel scan (moderate)
 *   RX_IDLE                  : always-on listen (lowest, yields to BLE advertising & connections)
 */
 
StackActivity activityZigbee_bleZigbeeH[ACTIVITY_NUM_ZIGBEE*PRIORITY_NUM] =
{
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_LINK_EST, DMM_StackPNormal, 100),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_LINK_EST, DMM_StackPHigh, 200),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_LINK_EST, DMM_StackPUrgent, 230),

     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_BEACON, DMM_StackPNormal, 100),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_BEACON, DMM_StackPHigh, 200),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_BEACON, DMM_StackPUrgent, 230),

     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_DATA, DMM_StackPNormal, 150),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_DATA, DMM_StackPHigh, 200),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_TX_DATA, DMM_StackPUrgent, 240),

     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_SCAN, DMM_StackPNormal, 90),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_SCAN, DMM_StackPHigh, 190),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_SCAN, DMM_StackPUrgent, 215),

     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_POLLING, DMM_StackPNormal, 100),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_POLLING, DMM_StackPHigh, 200),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_POLLING, DMM_StackPUrgent, 230),

     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_IDLE, DMM_StackPNormal, 65),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_IDLE, DMM_StackPHigh, 65),
     DMM_GLOBAL_PRIORITY(DMM_ZIGBEE_RX_IDLE, DMM_StackPUrgent, 75),
};

/* The order of stacks here must match the order in the SysConfig policy table
 * (blePeripheral first, zigbeeRouter second — same as dmm.stackRoles in the syscfg). */
GlobalTable globalPriorityTable_bleZigbeeH[DMMPOLICY_NUM_STACKS] =
{
    {
        .globalTableArray = activityBLE_bleZigbeeH,
        .tableSize        = (uint8_t)(ACTIVITY_NUM_BLE * PRIORITY_NUM),
        .stackRole        = DMMPolicy_StackRole_BlePeripheral,
    },
    {
        .globalTableArray = activityZigbee_bleZigbeeH,
        .tableSize        = (uint8_t)(ACTIVITY_NUM_ZIGBEE * PRIORITY_NUM),
        .stackRole        = DMM_ROLE_ZIGBEE,
    },
};
