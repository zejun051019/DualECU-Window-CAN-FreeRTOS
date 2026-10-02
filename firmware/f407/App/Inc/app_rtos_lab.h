#ifndef APP_RTOS_LAB_H
#define APP_RTOS_LAB_H

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

/* Single-writer counters for debugger observation during RTOS-01. */
extern volatile uint32_t g_rtos_lab_fast_count;
extern volatile uint32_t g_rtos_lab_slow_count;
extern volatile uint32_t g_rtos_lab_queue_create_ok;
extern volatile uint32_t g_rtos_lab_queue_put_fail_count;
extern volatile uint32_t g_rtos_lab_queue_get_fail_count;
extern volatile uint32_t g_rtos_lab_queue_mismatch_count;

osThreadId_t App_RtosLab_CreateSlowTask(UART_HandleTypeDef *uart);
void App_RtosLab_FastTask(void *argument);

#endif
