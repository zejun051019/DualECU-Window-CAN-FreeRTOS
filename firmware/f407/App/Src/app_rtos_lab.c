#include "app_rtos_lab.h"

volatile uint32_t g_rtos_lab_fast_count;
volatile uint32_t g_rtos_lab_slow_count;
volatile uint32_t g_rtos_lab_queue_create_ok;
volatile uint32_t g_rtos_lab_queue_put_fail_count;
volatile uint32_t g_rtos_lab_queue_get_fail_count;
volatile uint32_t g_rtos_lab_queue_mismatch_count;

typedef struct {
  uint32_t sequence;
  uint32_t check;
} LabMessage;

static osMessageQueueId_t s_queue;

static void App_RtosLab_SlowTask(void *argument)
{
  UART_HandleTypeDef *uart = (UART_HandleTypeDef *)argument;
  static const uint8_t marker[] = "RTOS_02_RX\r\n";
  LabMessage received;

  for (;;) {
    if (osMessageQueueGet(s_queue, &received, NULL, osWaitForever) != osOK) {
      ++g_rtos_lab_queue_get_fail_count;
      continue;
    }
    if (received.check != (received.sequence ^ 0xA5A5A5A5U)) {
      ++g_rtos_lab_queue_mismatch_count;
    }
    ++g_rtos_lab_slow_count;
    if ((g_rtos_lab_slow_count % 5U) == 0U) {
      (void)HAL_UART_Transmit(uart, (uint8_t *)marker, sizeof(marker) - 1U, 20U);
    }
  }
}

osThreadId_t App_RtosLab_CreateSlowTask(UART_HandleTypeDef *uart)
{
  static const osThreadAttr_t attributes = {
    .name = "rtosSlow",
    .priority = osPriorityBelowNormal,
    .stack_size = 512U
  };

  s_queue = osMessageQueueNew(8U, sizeof(LabMessage), NULL);
  if (s_queue == NULL) {
    return NULL;
  }
  g_rtos_lab_queue_create_ok = 1U;
  return osThreadNew(App_RtosLab_SlowTask, uart, &attributes);
}

void App_RtosLab_FastTask(void *argument)
{
  (void)argument;
  LabMessage message;

  for (;;) {
    ++g_rtos_lab_fast_count;
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
    message.sequence = g_rtos_lab_fast_count;
    message.check = message.sequence ^ 0xA5A5A5A5U;
    if (osMessageQueuePut(s_queue, &message, 0U, 0U) != osOK) {
      ++g_rtos_lab_queue_put_fail_count;
    }
    /* The queue copied the object; this stack object may now change. */
    message.sequence = 0U;
    message.check = 0U;
    osDelay(100U);
  }
}
