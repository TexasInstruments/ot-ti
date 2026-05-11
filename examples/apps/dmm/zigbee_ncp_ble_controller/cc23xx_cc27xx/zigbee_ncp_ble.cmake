#
# zigbee_ncp_ble.cmake — Builds the Zigbee NCP + BLE Controller firmware image.
#
# No OpenThread NCP or stack libraries are linked.  The binary contains:
#   - Zigbee NCP firmware  (ZBOSS R23 ncp_soc libraries + platform glue)
#   - BLE Controller       (TI BLE ControllerLib + NPI + DMM)
#   - Combined Serial MUX  (mux_task_app, mux_uart, mux_virt_uart)
#   - DMM scheduler/policy (SDK prebuilt dmm_library.a, see dmm.genLibs in the SysConfig file)
#   - TI CC27XX platform   (drivers, SysConfig generated code — from openthread-cc13xx_cc26xx)
#

add_executable(zigbee-ncp-ble-controller ${COMMON_SOURCES})

target_include_directories(zigbee-ncp-ble-controller PUBLIC ${COMMON_INCLUDES})

target_compile_definitions(zigbee-ncp-ble-controller PRIVATE ZB_TI_F3_ZBOSS)
set_source_files_properties(ti_zigbee_mux.c PROPERTIES COMPILE_DEFINITIONS ZB_NCP_SOC_VENDOR)

# zigbeeConfig is defined in the platform library, which is searched before the Zigbee libraries
# that reference it.
target_link_options(zigbee-ncp-ble-controller PRIVATE -Wl,--undefined=zigbeeConfig)

# Platform library (drivers + SysConfig generated code) and BLE stack.
# No openthread-ncp-* or openthread-ftd/mtd: there is no Thread stack in this binary.
target_link_libraries(zigbee-ncp-ble-controller PUBLIC
    openthread-cc13xx_cc26xx
    ble-stack-lib
)

# BLE circular dependency: ble-stack-lib and ControllerLib.a reference each other.
# --start-group/--end-group lets the linker iterate until all symbols are resolved.
target_link_libraries(zigbee-ncp-ble-controller PRIVATE
    -Wl,--start-group
    $<TARGET_FILE:ble-stack-lib>
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/lib/CC27XXX10/ControllerLib/lib/ticlang/m33f/ControllerLib.a
    -Wl,--end-group)

install(TARGETS zigbee-ncp-ble-controller DESTINATION bin)
set_target_properties(zigbee-ncp-ble-controller PROPERTIES SUFFIX .out)
add_dependencies(zigbee-ncp-ble-controller ble-stack-lib)
