#include "app_boot_log.h"

#include <stddef.h>
#include <stdint.h>

#define APP_BOOT_LOG_TIMEOUT_MS 100U

static const uint8_t kBootMessage[] = "BOOT_ID=S2-F407-BOOT-001\r\n";

HAL_StatusTypeDef App_BootLog_Send(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(uart,
                             kBootMessage,
                             (uint16_t)(sizeof(kBootMessage) - 1U),
                             APP_BOOT_LOG_TIMEOUT_MS);
}
