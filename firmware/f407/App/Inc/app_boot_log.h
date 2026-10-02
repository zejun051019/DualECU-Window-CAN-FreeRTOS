#ifndef APP_BOOT_LOG_H
#define APP_BOOT_LOG_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef App_BootLog_Send(UART_HandleTypeDef *uart);

#endif /* APP_BOOT_LOG_H */
