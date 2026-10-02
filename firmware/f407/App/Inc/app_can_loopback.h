#ifndef APP_CAN_LOOPBACK_H
#define APP_CAN_LOOPBACK_H

#include "stm32f4xx_hal.h"

typedef enum {
  APP_CAN_LOOPBACK_NOT_RUN = 0,
  APP_CAN_LOOPBACK_PASS = 1,
  APP_CAN_LOOPBACK_FILTER_FAIL = 2,
  APP_CAN_LOOPBACK_START_FAIL = 3,
  APP_CAN_LOOPBACK_TX_FAIL = 4,
  APP_CAN_LOOPBACK_RX_TIMEOUT = 5,
  APP_CAN_LOOPBACK_RX_MISMATCH = 6,
  APP_CAN_LOOPBACK_REJECT_FAIL = 7
} AppCanLoopbackResult;

extern volatile uint32_t g_can_loopback_result;
extern volatile uint32_t g_can_loopback_tx_requests;
extern volatile uint32_t g_can_loopback_rx_matched;
extern volatile uint32_t g_can_loopback_hal_error;

AppCanLoopbackResult App_CanLoopback_Run(CAN_HandleTypeDef *can);

#endif
