if(NOT DEFINED OT_PLATFORM_LIB_MTD)
    set(OT_PLATFORM_LIB_MTD ${OT_PLATFORM_LIB})
endif()

add_executable(ot-zigbee-ncp-ble-controller-mtd ${COMMON_SOURCES})

target_include_directories(ot-zigbee-ncp-ble-controller-mtd PUBLIC ${COMMON_INCLUDES})

target_compile_definitions(ot-zigbee-ncp-ble-controller-mtd PRIVATE ZB_TI_F3_ZBOSS)
set_source_files_properties(ti_zigbee_mux.c PROPERTIES COMPILE_DEFINITIONS ZB_NCP_SOC_VENDOR)

# zigbeeConfig is defined in the platform library, which is searched before the Zigbee libraries
# that reference it.
target_link_options(ot-zigbee-ncp-ble-controller-mtd PRIVATE -Wl,--undefined=zigbeeConfig)

target_link_libraries(ot-zigbee-ncp-ble-controller-mtd PUBLIC
    openthread-ncp-mtd
    ${OT_PLATFORM_LIB_MTD}
    openthread-mtd
    ${OT_PLATFORM_LIB_MTD}
    openthread-ncp-mtd
    ${OT_MBEDTLS}
    ot-config-mtd
    ot-config
    ble-stack-lib
)

target_link_libraries(ot-zigbee-ncp-ble-controller-mtd PRIVATE
    -Wl,--start-group
    $<TARGET_FILE:ble-stack-lib>
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/lib/CC27XXX10/ControllerLib/lib/ticlang/m33f/ControllerLib.a
    -Wl,--end-group
)

install(TARGETS ot-zigbee-ncp-ble-controller-mtd DESTINATION bin)
set_target_properties(ot-zigbee-ncp-ble-controller-mtd PROPERTIES SUFFIX .out)
add_dependencies(ot-zigbee-ncp-ble-controller-mtd ble-stack-lib)
