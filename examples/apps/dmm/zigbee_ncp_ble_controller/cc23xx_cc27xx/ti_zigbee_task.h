#ifndef TI_ZIGBEE_TASK_H
#define TI_ZIGBEE_TASK_H
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create the static FreeRTOS task that runs the ZBOSS NCP firmware.
 *
 * Wraps main_task_function() from the SDK's ti_f3_main.c in a FreeRTOS
 * task instead of a pthread, so no FreeRTOS heap is consumed.  Must be
 * called before vTaskStartScheduler().
 */
void ZigbeeTask_create(void);

#ifdef __cplusplus
}
#endif
#endif /* TI_ZIGBEE_TASK_H */
