#ifndef _TASKS_H_
#define _TASKS_H_
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern void watchdogTask(void *pvParameters);
extern void readEncodersTask(void *pvParameters);
extern void readIMUTask(void *pvParameters);
extern void sendStatusTask(void *pvParameters);
extern void writeServosTask(void *pvParameters);

extern const TickType_t whatchdog_delay;

#endif