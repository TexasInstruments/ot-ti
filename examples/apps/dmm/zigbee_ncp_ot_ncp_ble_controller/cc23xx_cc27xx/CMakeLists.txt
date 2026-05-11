set(COMMON_INCLUDES
    ${OT_PUBLIC_INCLUDES}
    ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
    ${PROJECT_SOURCE_DIR}/openthread/src/core
    # ZBOSS headers
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/zigbee/include
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/zigbee/osif/include
    ${TI_SIMPLELINK_SDK_DIR}/source/third_party/zigbee/zboss_r23/include
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/combined_serial
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/dmm/combined_serial/embedded
)

set(BLE_CONTROLLER_OPT_FILES
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_padv.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_pscan.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_cs.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_cs_test.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_initiator.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_connectable.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_scanner.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_cm.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_power_control.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_rssi_monitor.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_connection_handover.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_legacy_cmd.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_health_check.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_adv_conn.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_adv_nconn.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_ae.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_adaptivity.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_ble_health.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_dmm.c
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/stack_util/lib_opt/src/ctrl_opt_dmm_dynamic_priority.c
    dmm_priority_ble_thread_zigbee.c
)

set(ZIGBEE_SDK_DIR ${TI_SIMPLELINK_SDK_DIR}/source/ti/zigbee)
set(ZIGBEE_NCP_SOURCES
    ${ZIGBEE_SDK_DIR}/osif/ti_f3/ti_f3_main.c
    ${ZIGBEE_SDK_DIR}/osif/ti_f3/ti_f3_led_buttons.c
    ${ZIGBEE_SDK_DIR}/mac/ti_f3_mac_activity.c
    ${ZIGBEE_SDK_DIR}/application/ncp_soc/ncp_soc.c
    ti_zigbee_task.c
)

set(COMMON_SOURCES
    main.c
    ncp.c
    thread_mux.c
    ti_zigbee_mux.c
    ${ZIGBEE_NCP_SOURCES}
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/app_util/config/src/ble_user_config.c
    ${BLE_CONTROLLER_OPT_FILES}
)

# The SDK main() is renamed to avoid a collision with freertos_main.c
set_source_files_properties(${ZIGBEE_SDK_DIR}/osif/ti_f3/ti_f3_main.c
    PROPERTIES COMPILE_DEFINITIONS main=ZigbeeSdk_unusedMain)

if(TI_SIMPLELINK_KERNEL STREQUAL "freertos")
    list(APPEND COMMON_SOURCES freertos_main.c)
else()
    message(FATAL_ERROR "Unsupported TI_SIMPLELINK_KERNEL: ${TI_SIMPLELINK_KERNEL}")
endif()

if(OT_APP_ZIGBEE_NCP_OT_NCP_BLE_CONTROLLER)
    include(ble.cmake)
    if(OT_FTD)
        include(zigbee_ncp_ot_ncp_ble_ftd.cmake)
    endif()
    if(OT_MTD)
        include(zigbee_ncp_ot_ncp_ble_mtd.cmake)
    endif()
endif()
