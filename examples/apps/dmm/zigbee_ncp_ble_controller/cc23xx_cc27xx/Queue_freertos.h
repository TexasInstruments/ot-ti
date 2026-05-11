#ifndef QUEUE_FREERTOS_H
#define QUEUE_FREERTOS_H

#include <FreeRTOS.h>
#include <portmacro.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#define SUCCESS 0x00
#define FAILURE 0x01
#define TRUE  1
#define FALSE 0

QueueHandle_t Queue_create(uint32_t QueueLen, uint32_t msgSize);
uint8_t Queue_enqueue(QueueHandle_t msgQueue, char *pMsg);
uint8_t* Queue_dequeue(QueueHandle_t msgQueue);
uint8_t Queue_empty(QueueHandle_t msgQueue);

#endif /* QUEUE_FREERTOS_H */
