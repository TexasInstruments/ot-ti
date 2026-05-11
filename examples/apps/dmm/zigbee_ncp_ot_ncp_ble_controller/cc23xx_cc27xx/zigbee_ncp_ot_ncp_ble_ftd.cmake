if(NOT DEFINED OT_PLATFORM_LIB_FTD)
    set(OT_PLATFORM_LIB_FTD ${OT_PLATFORM_LIB})
endif()

add_executable(ot-zigbee-ncp-ble-controller-ftd ${COMMON_SOURCES})

target_include_directories(ot-zigbee-ncp-ble-controller-ftd PUBLIC ${COMMON_INCLUDES})

target_compile_definitions(ot-zigbee-ncp-ble-controller-ftd PRIVATE ZB_TI_F3_ZBOSS)
set_source_files_properties(ti_zigbee_mux.c PROPERTIES COMPILE_DEFINITIONS ZB_NCP_SOC_VENDOR)

# zigbeeConfig is defined in the platform library, which is searched before the Zigbee libraries
# that reference it.
target_link_options(ot-zigbee-ncp-ble-controller-ftd PRIVATE -Wl,--undefined=zigbeeConfig)

target_link_libraries(ot-zigbee-ncp-ble-controller-ftd PUBLIC
    openthread-ncp-ftd
    ${OT_PLATFORM_LIB_FTD}
    openthread-ftd
    ${OT_PLATFORM_LIB_FTD}
    openthread-ncp-ftd
    ${OT_MBEDTLS}
    ot-config-ftd
    ot-config
    ble-stack-lib
)

target_link_libraries(ot-zigbee-ncp-ble-controller-ftd PRIVATE
    -Wl,--start-group
    $<TARGET_FILE:ble-stack-lib>
    ${TI_SIMPLELINK_SDK_DIR}/source/ti/ble/lib/CC27XXX10/ControllerLib/lib/ticlang/m33f/ControllerLib.a
    -Wl,--end-group
)

install(TARGETS ot-zigbee-ncp-ble-controller-ftd DESTINATION bin)
set_target_properties(ot-zigbee-ncp-ble-controller-ftd PROPERTIES SUFFIX .out)
add_dependencies(ot-zigbee-ncp-ble-controller-ftd ble-stack-lib)
