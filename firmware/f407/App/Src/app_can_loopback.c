#include "app_can_loopback.h"

#define LOOPBACK_ID 0x123U
#define LOOPBACK_COUNT 100U
#define LOOPBACK_TIMEOUT_MS 50U

volatile uint32_t g_can_loopback_result;
volatile uint32_t g_can_loopback_tx_requests;
volatile uint32_t g_can_loopback_rx_matched;
volatile uint32_t g_can_loopback_hal_error;

static AppCanLoopbackResult Fail(CAN_HandleTypeDef *can, AppCanLoopbackResult result)
{
  g_can_loopback_hal_error = HAL_CAN_GetError(can);
  g_can_loopback_result = (uint32_t)result;
  return result;
}

static HAL_StatusTypeDef Send(CAN_HandleTypeDef *can, uint32_t id,
                              uint8_t *data)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;

  header.StdId = id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 4U;
  header.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_AddTxMessage(can, &header, data, &mailbox) != HAL_OK) {
    return HAL_ERROR;
  }
  ++g_can_loopback_tx_requests;
  return HAL_OK;
}

static uint8_t WaitForRx(CAN_HandleTypeDef *can)
{
  uint32_t start = HAL_GetTick();
  while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) == 0U) {
    if ((uint32_t)(HAL_GetTick() - start) >= LOOPBACK_TIMEOUT_MS) {
      return 0U;
    }
  }
  return 1U;
}

AppCanLoopbackResult App_CanLoopback_Run(CAN_HandleTypeDef *can)
{
  CAN_FilterTypeDef filter = {0};
  CAN_RxHeaderTypeDef received_header;
  uint8_t sent[8] = {0};
  uint8_t received[8];

  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = LOOPBACK_ID << 5;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0x7FFU << 5;
  filter.FilterMaskIdLow = 0x0006U; /* IDE and RTR must both be zero. */
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK) {
    return Fail(can, APP_CAN_LOOPBACK_FILTER_FAIL);
  }
  if (HAL_CAN_Start(can) != HAL_OK) {
    return Fail(can, APP_CAN_LOOPBACK_START_FAIL);
  }

  for (uint32_t i = 0U; i < LOOPBACK_COUNT; ++i) {
    sent[0] = (uint8_t)i;
    sent[1] = (uint8_t)(i ^ 0xA5U);
    sent[2] = 0x5AU;
    sent[3] = 0xC3U;
    if (Send(can, LOOPBACK_ID, sent) != HAL_OK) {
      return Fail(can, APP_CAN_LOOPBACK_TX_FAIL);
    }
    if (WaitForRx(can) == 0U) {
      return Fail(can, APP_CAN_LOOPBACK_RX_TIMEOUT);
    }
    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &received_header,
                             received) != HAL_OK ||
        received_header.IDE != CAN_ID_STD ||
        received_header.RTR != CAN_RTR_DATA ||
        received_header.StdId != LOOPBACK_ID ||
        received_header.DLC != 4U ||
        received[0] != sent[0] || received[1] != sent[1] ||
        received[2] != sent[2] || received[3] != sent[3]) {
      return Fail(can, APP_CAN_LOOPBACK_RX_MISMATCH);
    }
    ++g_can_loopback_rx_matched;
  }

  if (Send(can, LOOPBACK_ID + 1U, sent) != HAL_OK) {
    return Fail(can, APP_CAN_LOOPBACK_TX_FAIL);
  }
  uint32_t start = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(can) != 3U) {
    if ((uint32_t)(HAL_GetTick() - start) >= LOOPBACK_TIMEOUT_MS) {
      return Fail(can, APP_CAN_LOOPBACK_TX_FAIL);
    }
  }
  HAL_Delay(2U);
  if (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) != 0U) {
    return Fail(can, APP_CAN_LOOPBACK_REJECT_FAIL);
  }

  g_can_loopback_hal_error = HAL_CAN_GetError(can);
  g_can_loopback_result = APP_CAN_LOOPBACK_PASS;
  return APP_CAN_LOOPBACK_PASS;
}
