#include "app_can_physical.h"

#include "can_protocol.h"
#include "can_recovery_client.h"
#include "can_test_frame.h"
#include "window_request.h"
#include "app_button.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "task.h"

#define APP_CAN_RX_QUEUE_CAPACITY (8U)
#define APP_CAN_RX_ISR_DRAIN_BUDGET (3U)
#define APP_CAN_TX_CONTROL_PERIOD_MS (10U)
#define APP_CAN_REMOTE_OFFLINE_LIMIT_MS (500U)

typedef struct
{
  uint32_t standard_id;
  uint32_t identifier_type;
  uint32_t frame_type;
  uint32_t data_length;
  uint16_t hardware_timestamp;
  uint16_t reserved;
  uint32_t received_at_ms;
  uint8_t data[8];
} app_can_rx_message_t;

typedef struct
{
  can_protocol_status_t status;
  uint32_t received_at_ms;
  uint32_t generation;
  uint16_t hardware_timestamp;
  uint8_t valid;
} app_can_status_snapshot_t;

#define CAN_PHYSICAL_STARTUP_STOP_SEQUENCE (0xA5U)
#define APP_CAN_T04_CONDITION_RELEASE_DELAY_MS (250U)

typedef enum
{
  APP_CAN_TX_NONE = 0U,
  APP_CAN_TX_PROBE,
  APP_CAN_TX_STARTUP_STOP,
  APP_CAN_TX_CLEAR,
  APP_CAN_TX_FINAL_STOP,
  APP_CAN_TX_MATRIX,
  APP_CAN_TX_T04,
  APP_CAN_TX_T04_INJECT,
  APP_CAN_TX_T04_RELEASE,
  APP_CAN_TX_SAFE_STOP,
  APP_CAN_TX_WINDOW_REQUEST
} app_can_tx_kind_t;

typedef enum
{
  APP_CAN_MATRIX_IDLE = 0U,
  APP_CAN_MATRIX_STEP_UP,
  APP_CAN_MATRIX_STEP_STOP,
  APP_CAN_MATRIX_STEP_REPEAT_STOP,
  APP_CAN_MATRIX_STEP_OLD_UP,
  APP_CAN_MATRIX_STEP_INVALID_DLC,
  APP_CAN_MATRIX_STEP_KEEPALIVE_AFTER_DLC,
  APP_CAN_MATRIX_STEP_INVALID_VERSION,
  APP_CAN_MATRIX_STEP_KEEPALIVE_AFTER_VERSION,
  APP_CAN_MATRIX_STEP_INVALID_COMMAND,
  APP_CAN_MATRIX_STEP_KEEPALIVE_AFTER_COMMAND,
  APP_CAN_MATRIX_STEP_INVALID_RESERVED,
  APP_CAN_MATRIX_STEP_KEEPALIVE_AFTER_RESERVED,
  APP_CAN_MATRIX_STEP_DOWN,
  APP_CAN_MATRIX_STEP_STOP_AFTER_DOWN,
  APP_CAN_MATRIX_STEP_SEQ_FF,
  APP_CAN_MATRIX_STEP_SEQ_00,
  APP_CAN_MATRIX_STEP_SEQ_80,
  APP_CAN_MATRIX_STEP_CONFLICT_UP,
  APP_CAN_MATRIX_STEP_FINAL_STOP,
  APP_CAN_MATRIX_HOLD_STOP,
  APP_CAN_MATRIX_HOLD_UP,
  APP_CAN_MATRIX_FAILED
} app_can_matrix_stage_t;

typedef struct
{
  uint8_t sequence;
  can_protocol_command_id_t command;
  uint8_t expected_sequence;
  can_protocol_state_t expected_state;
  uint8_t invalid_frame;
} app_can_matrix_step_t;

typedef enum
{
  APP_CAN_MATRIX_VALID_FRAME = 0U,
  APP_CAN_MATRIX_INVALID_DLC,
  APP_CAN_MATRIX_INVALID_VERSION,
  APP_CAN_MATRIX_INVALID_COMMAND,
  APP_CAN_MATRIX_INVALID_RESERVED
} app_can_matrix_frame_kind_t;

static const app_can_matrix_step_t s_matrix_steps[] = {
  {0xA8U, CAN_PROTOCOL_CMD_UP, 0xA8U, CAN_PROTOCOL_STATE_UP, 0U},
  {0xA9U, CAN_PROTOCOL_CMD_STOP, 0xA9U, CAN_PROTOCOL_STATE_STOP, 0U},
  {0xA9U, CAN_PROTOCOL_CMD_STOP, 0xA9U, CAN_PROTOCOL_STATE_STOP, 0U},
  {0xA8U, CAN_PROTOCOL_CMD_UP, 0xA9U, CAN_PROTOCOL_STATE_STOP, 0U},
  {0xAAU, CAN_PROTOCOL_CMD_UP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_INVALID_DLC},
  {0xA9U, CAN_PROTOCOL_CMD_STOP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_VALID_FRAME},
  {0xAAU, CAN_PROTOCOL_CMD_UP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_INVALID_VERSION},
  {0xA9U, CAN_PROTOCOL_CMD_STOP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_VALID_FRAME},
  {0xAAU, CAN_PROTOCOL_CMD_UP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_INVALID_COMMAND},
  {0xA9U, CAN_PROTOCOL_CMD_STOP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_VALID_FRAME},
  {0xAAU, CAN_PROTOCOL_CMD_UP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_INVALID_RESERVED},
  {0xA9U, CAN_PROTOCOL_CMD_STOP, 0xA9U, CAN_PROTOCOL_STATE_STOP, APP_CAN_MATRIX_VALID_FRAME},
  {0xABU, CAN_PROTOCOL_CMD_DOWN, 0xABU, CAN_PROTOCOL_STATE_DOWN, 0U},
  {0xACU, CAN_PROTOCOL_CMD_STOP, 0xACU, CAN_PROTOCOL_STATE_STOP, 0U},
  {0xFFU, CAN_PROTOCOL_CMD_STOP, 0xFFU, CAN_PROTOCOL_STATE_STOP, 0U},
  {0x00U, CAN_PROTOCOL_CMD_STOP, 0x00U, CAN_PROTOCOL_STATE_STOP, 0U},
  {0x80U, CAN_PROTOCOL_CMD_STOP, 0x00U, CAN_PROTOCOL_STATE_STOP, 0U},
  {0x00U, CAN_PROTOCOL_CMD_UP, 0x00U, CAN_PROTOCOL_STATE_STOP, 0U},
  {0x01U, CAN_PROTOCOL_CMD_STOP, 0x01U, CAN_PROTOCOL_STATE_STOP, 0U}
};

#define APP_CAN_MATRIX_FIRST_STEP (APP_CAN_MATRIX_STEP_UP)
#define APP_CAN_MATRIX_FINAL_STEP (APP_CAN_MATRIX_STEP_FINAL_STOP)

volatile uint32_t g_can_physical_start_result;
volatile uint32_t g_can_physical_status_rx_count;
volatile uint32_t g_can_physical_invalid_rx_count;
volatile uint32_t g_can_physical_fifo_overrun_count;
volatile uint32_t g_can_physical_last_fault;
volatile uint32_t g_can_physical_last_status_sequence;
volatile uint32_t g_can_physical_last_status_state;
volatile uint32_t g_can_physical_last_status_flags;
volatile uint32_t g_can_physical_stop_tx_request_count;
volatile uint32_t g_can_physical_stop_tx_success_count;
volatile uint32_t g_can_physical_stop_tx_error_count;
volatile uint32_t g_can_physical_stop_status_match_count;
volatile uint32_t g_can_physical_clear_tx_request_count;
volatile uint32_t g_can_physical_clear_tx_success_count;
volatile uint32_t g_can_physical_clear_tx_error_count;
volatile uint32_t g_can_physical_clear_status_match_count;
volatile uint32_t g_can_physical_final_stop_tx_request_count;
volatile uint32_t g_can_physical_final_stop_tx_success_count;
volatile uint32_t g_can_physical_final_stop_tx_error_count;
volatile uint32_t g_can_physical_final_stop_status_match_count;
volatile uint32_t g_can_physical_stop_ack_timeout_count;
volatile uint32_t g_can_physical_clear_ack_timeout_count;
volatile uint32_t g_can_physical_final_stop_ack_timeout_count;
volatile uint32_t g_can_physical_clear_rejected_count;
volatile uint32_t g_can_physical_freshness_blocked_count;
volatile uint32_t g_can_physical_recovery_client_stage;
volatile uint32_t g_can_physical_recovery_test_result;
volatile uint32_t g_can_physical_recovery_sequence_error_site;
volatile uint32_t g_can_physical_matrix_stage;
volatile uint32_t g_can_physical_matrix_result;
volatile uint32_t g_can_physical_matrix_enable;
volatile uint32_t g_can_physical_matrix_hold_up;
volatile uint32_t g_can_physical_matrix_tx_count;
volatile uint32_t g_can_physical_matrix_unexpected_status_count;
volatile uint32_t g_can_physical_t04_arm;
volatile uint32_t g_can_physical_t04_start;
volatile uint32_t g_can_physical_t04_stage;
volatile uint32_t g_can_physical_t04_result;
volatile uint32_t g_can_physical_t04_tx_count;
volatile uint32_t g_can_physical_t04_status_match_count;
volatile uint32_t g_can_physical_t04_unexpected_status_count;
volatile uint32_t g_can_physical_t04_fixture_tx_count;
volatile uint32_t g_can_physical_last_hal_error;
volatile uint32_t g_can_physical_tx_request_count;
volatile uint32_t g_can_physical_tx_success_count;
volatile uint32_t g_can_physical_tx_error_count;
volatile uint32_t g_can_physical_tx_last_esr;
volatile uint32_t g_can_physical_probe_frame_count;
volatile uint32_t g_can_physical_probe_valid_count;
volatile uint32_t g_can_physical_probe_first_counter;
volatile uint32_t g_can_physical_probe_last_counter;
volatile uint32_t g_can_physical_probe_missing_count;
volatile uint32_t g_can_physical_probe_duplicate_count;
volatile uint32_t g_can_physical_probe_stale_count;
volatile uint32_t g_can_physical_probe_invalid_count;
volatile uint32_t g_can_physical_rx_queue_create_ok;
volatile uint32_t g_can_physical_task_create_failure;
volatile uint32_t g_can_physical_stack_overflow_fault;
volatile uint32_t g_can_physical_malloc_failed_fault;
volatile uint32_t g_can_physical_rx_queue_put_count;
volatile uint32_t g_can_physical_rx_queue_get_count;
volatile uint32_t g_can_physical_rx_queue_drop_count;
volatile uint32_t g_can_physical_rx_queue_current;
volatile uint32_t g_can_physical_rx_queue_high_water;
volatile uint32_t g_can_physical_rx_isr_count;
volatile uint32_t g_can_physical_rx_isr_max_batch;
volatile uint32_t g_can_physical_rx_notification_pause_count;
volatile uint32_t g_can_physical_rx_notification_resume_count;
volatile uint32_t g_can_physical_rx_status_snapshot_count;
volatile uint32_t g_can_physical_rx_status_stale_count;
volatile uint32_t g_can_physical_rx_remote_status_age_ms;
volatile uint32_t g_can_physical_rx_remote_offline;
volatile uint32_t g_can_physical_rx_overflow_first_tick_ms;
volatile uint32_t g_can_physical_rx_status_last_received_tick_ms;
volatile uint32_t g_can_physical_rx_status_last_processed_tick_ms;
volatile uint32_t g_can_physical_tx_task_cycle_count;
volatile uint32_t g_can_physical_tx_task_late_count;
volatile uint32_t g_can_physical_tx_task_max_interval_ms;
volatile uint32_t g_can_physical_tx_task_stack_space_bytes;
volatile uint32_t g_can_physical_rx_task_stack_space_bytes;
volatile uint32_t g_can_physical_heap_free_bytes;
volatile uint32_t g_can_physical_heap_min_free_bytes;
volatile uint32_t g_can_physical_rx_overflow_latched;
volatile uint32_t g_can_physical_rx_overflow_active;
volatile uint32_t g_can_physical_local_stop_latched;
volatile uint32_t g_can_physical_local_stop_reason;
volatile uint32_t g_can_physical_local_stop_latched_tick_ms;
volatile uint32_t g_can_physical_safe_stop_tx_request_count;
volatile uint32_t g_can_physical_safe_stop_tx_request_tick_ms;
volatile uint32_t g_can_physical_safe_stop_tx_success_count;
volatile uint32_t g_can_physical_safe_stop_tx_success_tick_ms;
volatile uint32_t g_can_physical_safe_stop_tx_error_count;
volatile uint32_t g_can_physical_safe_stop_status_match_count;
volatile uint32_t g_can_physical_safe_stop_status_match_tick_ms;
volatile uint32_t g_can_physical_window_request_tx_count;
volatile uint32_t g_can_physical_window_request_tx_error_count;
volatile uint32_t g_can_physical_window_request_up_inject;
/* Debug requests are consumed by the existing TX owner, never raw queue writes. */
volatile uint32_t g_can_physical_window_request_inject;
volatile uint32_t g_can_physical_window_request_inject_result;
volatile uint32_t g_can_physical_window_request_inject_sequence;
volatile uint32_t g_can_physical_window_request_up_result;
volatile uint32_t g_can_physical_window_request_up_sequence;
volatile uint32_t g_can_physical_window_request_rearm;
volatile uint32_t g_can_physical_window_request_rearm_result;
volatile uint32_t g_can_physical_rx_task_test_pause;
volatile uint32_t g_can_physical_button_raw;
volatile uint32_t g_can_physical_button_last_event;
volatile uint32_t g_can_physical_button_event_count;
volatile uint32_t g_can_physical_button_rejected_count;

static CAN_HandleTypeDef *s_can;
static osMessageQueueId_t s_rx_queue;
static osMutexId_t s_status_mutex;
static osThreadId_t s_rx_task;
static window_request_queue_t s_window_requests;
static app_button_t s_button;
static app_button_deferred_toggle_t s_button_deferred_toggle;
static window_request_command_t s_button_pending_command;
static uint8_t s_button_pending_sequence;
static uint8_t s_button_demo_motion_seen;
static uint32_t s_button_pending_since_ms;
static app_can_status_snapshot_t s_latest_status;
static uint32_t s_consumed_status_generation;
static uint32_t s_last_tx_task_cycle_ms;
static uint8_t s_have_tx_task_cycle;
static volatile uint32_t s_rx_notification_paused;
static volatile uint32_t s_rx_overflow_latched;
static volatile uint32_t s_rx_overflow_cause_active;
static volatile uint32_t s_rx_fifo_overrun_latched;
static uint32_t s_can_started_at_ms;
static uint32_t s_window_last_tx_attempt_ms;
static uint16_t s_window_last_tx_timestamp;
static uint8_t s_window_has_tx_attempt;
static uint8_t s_window_has_tx_timestamp;
static uint32_t s_tx_mailbox;
static uint32_t s_tx_requested_at_ms;
static uint8_t s_tx_pending;
static uint8_t s_tx_stopped;
static uint8_t s_tx_abort_requested;
static app_can_tx_kind_t s_tx_kind;
static can_recovery_client_t s_recovery_client;
static uint8_t s_matrix_waiting;
static uint8_t s_matrix_last_sequence;
static uint32_t s_matrix_last_tx_ms;
static uint32_t s_matrix_step_started_ms;
static uint16_t s_matrix_last_tx_timestamp;
static uint32_t s_t04_last_tx_ms;
static uint32_t s_t04_wait_started_ms;
static uint16_t s_t04_last_tx_timestamp;
static uint8_t s_t04_has_tx_timestamp;
static uint8_t s_t04_injection_pending;
static uint8_t s_t04_release_sent;

static void App_CanPhysical_RxTask(void *argument);
static void App_CanPhysical_ServiceLatestStatus(void);
static void App_CanPhysical_RequestLocalStop(
    app_can_local_stop_reason_t reason);
static void App_CanPhysical_ProcessDebugRequests(void);
static uint8_t App_CanPhysical_NextSequence(void);
static void App_CanPhysical_ServiceButton(void);

static void App_CanPhysical_UpdateRecoveryStage(void)
{
  g_can_physical_recovery_client_stage =
      (uint32_t)s_recovery_client.stage;
}

static bool App_CanPhysical_MatrixIsActive(void)
{
  return (g_can_physical_matrix_stage >= APP_CAN_MATRIX_FIRST_STEP) &&
         (g_can_physical_matrix_stage <= APP_CAN_MATRIX_HOLD_UP);
}

static bool App_CanPhysical_T04IsActive(void)
{
  return (g_can_physical_t04_stage >= APP_CAN_T04_SEND_BASELINE_STOP) &&
         (g_can_physical_t04_stage <= APP_CAN_T04_WAIT_FINAL_STOP);
}

static const app_can_matrix_step_t *App_CanPhysical_GetMatrixStep(void)
{
  if (g_can_physical_matrix_stage == APP_CAN_MATRIX_HOLD_UP) {
    return &s_matrix_steps[0];
  }
  const uint32_t index =
      g_can_physical_matrix_stage - APP_CAN_MATRIX_FIRST_STEP;
  return (index < (sizeof(s_matrix_steps) / sizeof(s_matrix_steps[0]))) ?
      &s_matrix_steps[index] : NULL;
}

static void App_CanPhysical_RestartRecovery(uint8_t stop_sequence,
                                            bool matrix_interrupted)
{
  app_button_deferred_toggle_cancel(&s_button_deferred_toggle);
  s_button_pending_command = WINDOW_REQUEST_NONE;
  s_button_demo_motion_seen = 0U;
  if (matrix_interrupted) {
    g_can_physical_matrix_result = APP_CAN_MATRIX_FAIL;
    g_can_physical_matrix_stage = APP_CAN_MATRIX_FAILED;
  }
  s_matrix_waiting = 0U;
  s_tx_stopped = 0U;
  can_recovery_client_init(&s_recovery_client, stop_sequence);
  g_can_physical_recovery_test_result = APP_CAN_RECOVERY_RESULT_RUNNING;
  App_CanPhysical_UpdateRecoveryStage();
}

static void App_CanPhysical_BeginSafeRecovery(void)
{
  App_CanPhysical_RestartRecovery(
      (uint8_t)(s_matrix_last_sequence + 1U), true);
}

static void App_CanPhysical_ObserveMatrixStatus(
    const can_protocol_status_t *status,
    uint16_t rx_timestamp)
{
  const uint16_t elapsed_ticks =
      (uint16_t)(rx_timestamp - s_matrix_last_tx_timestamp);

  if (!App_CanPhysical_MatrixIsActive() || (status == NULL) ||
      (elapsed_ticks == 0U) || (elapsed_ticks >= 0x8000U)) {
    return;
  }

  if (g_can_physical_matrix_stage == APP_CAN_MATRIX_HOLD_STOP) {
    if (!status->last_sequence_valid ||
        (status->last_sequence != 0x01U) ||
        (status->state != CAN_PROTOCOL_STATE_STOP) ||
        (status->fault != CAN_PROTOCOL_FAULT_NONE)) {
      ++g_can_physical_matrix_unexpected_status_count;
      App_CanPhysical_BeginSafeRecovery();
    }
    return;
  }

  if (g_can_physical_matrix_stage == APP_CAN_MATRIX_HOLD_UP) {
    if (!status->last_sequence_valid ||
        (status->last_sequence != s_matrix_steps[0].expected_sequence) ||
        (status->state != CAN_PROTOCOL_STATE_UP) ||
        (status->fault != CAN_PROTOCOL_FAULT_NONE)) {
      ++g_can_physical_matrix_unexpected_status_count;
      App_CanPhysical_BeginSafeRecovery();
    } else if (g_can_physical_matrix_hold_up == 0U) {
      s_matrix_waiting = 0U;
      g_can_physical_matrix_stage = APP_CAN_MATRIX_STEP_DOWN;
    }
    return;
  }

  if (s_matrix_waiting == 0U) {
    return;
  }

  const app_can_matrix_step_t *step = App_CanPhysical_GetMatrixStep();
  if ((step != NULL) && status->last_sequence_valid &&
      (status->last_sequence == step->expected_sequence) &&
      (status->state == step->expected_state) &&
      (status->fault == CAN_PROTOCOL_FAULT_NONE)) {
    s_matrix_waiting = 0U;
    if (g_can_physical_matrix_stage == APP_CAN_MATRIX_STEP_UP &&
        (g_can_physical_matrix_hold_up != 0U)) {
      g_can_physical_matrix_stage = APP_CAN_MATRIX_HOLD_UP;
    } else {
      ++g_can_physical_matrix_stage;
    }
    if (g_can_physical_matrix_stage == APP_CAN_MATRIX_HOLD_STOP) {
      g_can_physical_matrix_result = APP_CAN_MATRIX_PASS;
    }
  } else {
    ++g_can_physical_matrix_unexpected_status_count;
    if ((status->fault != CAN_PROTOCOL_FAULT_NONE) ||
        !status->last_sequence_valid) {
      App_CanPhysical_BeginSafeRecovery();
    }
  }
}

static void App_CanPhysical_FailT04(void)
{
  g_can_physical_t04_stage = APP_CAN_T04_FAILED;
  g_can_physical_t04_result = APP_CAN_T04_FAIL;
  g_can_physical_t04_arm = 0U;
  g_can_physical_t04_start = 0U;
  s_t04_has_tx_timestamp = 0U;
}

static bool App_CanPhysical_T04CommandForStage(
    app_can_t04_stage_t stage,
    can_protocol_command_t *command_out)
{
  if (command_out == NULL) {
    return false;
  }

  switch (stage) {
  case APP_CAN_T04_SEND_BASELINE_STOP:
  case APP_CAN_T04_WAIT_BASELINE_STOP:
    command_out->sequence = 0xD0U;
    command_out->command = CAN_PROTOCOL_CMD_STOP;
    return true;
  case APP_CAN_T04_SEND_FIRST_CLEAR:
  case APP_CAN_T04_WAIT_FIRST_CLEAR:
  case APP_CAN_T04_SEND_OLD_CLEAR:
  case APP_CAN_T04_WAIT_OLD_CLEAR:
    command_out->sequence = 0xD1U;
    command_out->command = CAN_PROTOCOL_CMD_CLEAR_FAULT;
    return true;
  case APP_CAN_T04_SEND_INSERT_STOP:
  case APP_CAN_T04_WAIT_INSERT_STOP:
  case APP_CAN_T04_WAIT_CONDITION_RELEASE:
    command_out->sequence = 0xD2U;
    command_out->command = CAN_PROTOCOL_CMD_STOP;
    return true;
  case APP_CAN_T04_SEND_NEW_CLEAR:
  case APP_CAN_T04_WAIT_NEW_CLEAR:
    command_out->sequence = 0xD3U;
    command_out->command = CAN_PROTOCOL_CMD_CLEAR_FAULT;
    return true;
  case APP_CAN_T04_SEND_FINAL_STOP:
  case APP_CAN_T04_WAIT_FINAL_STOP:
    command_out->sequence = 0xD4U;
    command_out->command = CAN_PROTOCOL_CMD_STOP;
    return true;
  default:
    return false;
  }
}

static bool App_CanPhysical_T04CommandDue(uint32_t now_ms)
{
  const app_can_t04_stage_t stage =
      (app_can_t04_stage_t)g_can_physical_t04_stage;

  if ((stage == APP_CAN_T04_SEND_BASELINE_STOP) ||
      (stage == APP_CAN_T04_SEND_FIRST_CLEAR) ||
      (stage == APP_CAN_T04_SEND_INSERT_STOP) ||
      (stage == APP_CAN_T04_SEND_OLD_CLEAR) ||
      (stage == APP_CAN_T04_SEND_NEW_CLEAR) ||
      (stage == APP_CAN_T04_SEND_FINAL_STOP)) {
    return true;
  }
  if ((stage == APP_CAN_T04_WAIT_BASELINE_STOP) ||
      (stage == APP_CAN_T04_WAIT_FIRST_CLEAR) ||
      (stage == APP_CAN_T04_WAIT_INSERT_STOP) ||
      (stage == APP_CAN_T04_WAIT_CONDITION_RELEASE) ||
      (stage == APP_CAN_T04_WAIT_OLD_CLEAR) ||
      (stage == APP_CAN_T04_WAIT_NEW_CLEAR) ||
      (stage == APP_CAN_T04_WAIT_FINAL_STOP)) {
    const uint32_t limit_ms = (stage == APP_CAN_T04_WAIT_CONDITION_RELEASE) ?
                                  20000U : 1000U;
    if ((uint32_t)(now_ms - s_t04_wait_started_ms) > limit_ms) {
      App_CanPhysical_FailT04();
      return false;
    }
    return (uint32_t)(now_ms - s_t04_last_tx_ms) >=
           CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS;
  }
  return false;
}

static void App_CanPhysical_RecordT04TxSuccess(
    uint16_t tx_timestamp,
    uint32_t now_ms)
{
  const app_can_t04_stage_t stage =
      (app_can_t04_stage_t)g_can_physical_t04_stage;

  ++g_can_physical_t04_tx_count;
  s_t04_last_tx_ms = now_ms;
  s_t04_last_tx_timestamp = tx_timestamp;
  s_t04_has_tx_timestamp = 1U;

  switch (stage) {
  case APP_CAN_T04_SEND_BASELINE_STOP:
    g_can_physical_t04_stage = APP_CAN_T04_WAIT_BASELINE_STOP;
    break;
  case APP_CAN_T04_SEND_FIRST_CLEAR:
    g_can_physical_t04_stage = APP_CAN_T04_WAIT_FIRST_CLEAR;
    break;
  case APP_CAN_T04_SEND_INSERT_STOP:
    g_can_physical_t04_stage = APP_CAN_T04_WAIT_INSERT_STOP;
    break;
  case APP_CAN_T04_SEND_OLD_CLEAR:
    g_can_physical_t04_stage = APP_CAN_T04_WAIT_OLD_CLEAR;
    break;
  case APP_CAN_T04_SEND_NEW_CLEAR:
    g_can_physical_t04_stage = APP_CAN_T04_WAIT_NEW_CLEAR;
    break;
  case APP_CAN_T04_SEND_FINAL_STOP:
    g_can_physical_t04_stage = APP_CAN_T04_WAIT_FINAL_STOP;
    break;
  default:
    break;
  }
  if ((stage == APP_CAN_T04_SEND_BASELINE_STOP) ||
      (stage == APP_CAN_T04_SEND_FIRST_CLEAR) ||
      (stage == APP_CAN_T04_SEND_INSERT_STOP) ||
      (stage == APP_CAN_T04_SEND_OLD_CLEAR) ||
      (stage == APP_CAN_T04_SEND_NEW_CLEAR) ||
      (stage == APP_CAN_T04_SEND_FINAL_STOP)) {
    s_t04_wait_started_ms = now_ms;
  }
}

static void App_CanPhysical_ObserveT04Status(
    const can_protocol_status_t *status,
    uint16_t rx_timestamp)
{
  app_can_t04_stage_t stage;
  uint8_t expected_sequence;
  can_protocol_fault_t expected_fault;
  bool expected_overflow_active;
  uint16_t elapsed_ticks;
  bool matches;

  if ((status == NULL) || !App_CanPhysical_T04IsActive() ||
      (s_t04_has_tx_timestamp == 0U)) {
    return;
  }
  elapsed_ticks = (uint16_t)(rx_timestamp - s_t04_last_tx_timestamp);
  if ((elapsed_ticks == 0U) || (elapsed_ticks >= 0x8000U)) {
    return;
  }

  stage = (app_can_t04_stage_t)g_can_physical_t04_stage;
  if (stage == APP_CAN_T04_WAIT_CONDITION_RELEASE) {
    if ((status->last_sequence != 0xD2U) ||
        !status->last_sequence_valid ||
        (status->state != CAN_PROTOCOL_STATE_STOP) ||
        (status->fault != CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW)) {
      if (status->last_sequence == 0xD2U) {
        ++g_can_physical_t04_unexpected_status_count;
        App_CanPhysical_FailT04();
      }
      return;
    }
    if (status->rx_overflow_active) {
      return;
    }
    ++g_can_physical_t04_status_match_count;
    g_can_physical_t04_stage = APP_CAN_T04_SEND_OLD_CLEAR;
    s_t04_has_tx_timestamp = 0U;
    return;
  }
  if ((stage == APP_CAN_T04_WAIT_CONDITION_RELEASE) ||
      (stage == APP_CAN_T04_WAIT_BASELINE_STOP) ||
      (stage == APP_CAN_T04_WAIT_INSERT_STOP)) {
    expected_fault = CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW;
    expected_overflow_active = true;
  } else if (stage == APP_CAN_T04_WAIT_FIRST_CLEAR) {
    expected_fault = CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW;
    expected_overflow_active = true;
  } else if (stage == APP_CAN_T04_WAIT_OLD_CLEAR) {
    expected_fault = CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW;
    expected_overflow_active = false;
  } else if ((stage == APP_CAN_T04_WAIT_NEW_CLEAR) ||
             (stage == APP_CAN_T04_WAIT_FINAL_STOP)) {
    expected_fault = CAN_PROTOCOL_FAULT_NONE;
    expected_overflow_active = false;
  } else {
    return;
  }

  switch (stage) {
  case APP_CAN_T04_WAIT_BASELINE_STOP:
    expected_sequence = 0xD0U;
    break;
  case APP_CAN_T04_WAIT_FIRST_CLEAR:
    expected_sequence = 0xD1U;
    break;
  case APP_CAN_T04_WAIT_INSERT_STOP:
  case APP_CAN_T04_WAIT_CONDITION_RELEASE:
    expected_sequence = 0xD2U;
    break;
  case APP_CAN_T04_WAIT_OLD_CLEAR:
    expected_sequence = 0xD2U;
    if (status->last_sequence == 0xD1U) {
      ++g_can_physical_t04_unexpected_status_count;
      App_CanPhysical_FailT04();
      return;
    }
    break;
  case APP_CAN_T04_WAIT_NEW_CLEAR:
    expected_sequence = 0xD3U;
    break;
  case APP_CAN_T04_WAIT_FINAL_STOP:
    expected_sequence = 0xD4U;
    break;
  default:
    return;
  }

  if (status->last_sequence != expected_sequence) {
    return;
  }
  matches = status->last_sequence_valid &&
            (status->state == CAN_PROTOCOL_STATE_STOP) &&
            (status->fault == expected_fault) &&
            (status->rx_overflow_active == expected_overflow_active);
  if (!matches) {
    ++g_can_physical_t04_unexpected_status_count;
    App_CanPhysical_FailT04();
    return;
  }

  ++g_can_physical_t04_status_match_count;
  s_t04_has_tx_timestamp = 0U;
  switch (stage) {
  case APP_CAN_T04_WAIT_BASELINE_STOP:
    g_can_physical_t04_stage = APP_CAN_T04_SEND_FIRST_CLEAR;
    break;
  case APP_CAN_T04_WAIT_FIRST_CLEAR:
    g_can_physical_t04_stage = APP_CAN_T04_SEND_INSERT_STOP;
    break;
  case APP_CAN_T04_WAIT_INSERT_STOP:
    g_can_physical_t04_stage = APP_CAN_T04_WAIT_CONDITION_RELEASE;
    s_t04_wait_started_ms = HAL_GetTick();
    s_t04_last_tx_ms = HAL_GetTick();
    s_t04_has_tx_timestamp = 1U;
    s_t04_last_tx_timestamp = rx_timestamp;
    break;
  case APP_CAN_T04_WAIT_OLD_CLEAR:
    g_can_physical_t04_stage = APP_CAN_T04_SEND_NEW_CLEAR;
    break;
  case APP_CAN_T04_WAIT_NEW_CLEAR:
    g_can_physical_t04_stage = APP_CAN_T04_SEND_FINAL_STOP;
    break;
  case APP_CAN_T04_WAIT_FINAL_STOP:
    g_can_physical_t04_stage = APP_CAN_T04_COMPLETE;
    g_can_physical_t04_result = APP_CAN_T04_PASS;
    g_can_physical_t04_arm = 0U;
    g_can_physical_t04_start = 0U;
    break;
  default:
    break;
  }
}

static void App_CanPhysical_RecordTxFailure(app_can_tx_kind_t tx_kind)
{
  g_can_physical_tx_last_esr = s_can->Instance->ESR;
  g_can_physical_last_hal_error = HAL_CAN_GetError(s_can);
  ++g_can_physical_tx_error_count;
  if (tx_kind == APP_CAN_TX_SAFE_STOP) {
    ++g_can_physical_safe_stop_tx_error_count;
    s_window_last_tx_attempt_ms = HAL_GetTick();
    s_window_has_tx_attempt = 1U;
    return;
  }
  if (tx_kind == APP_CAN_TX_WINDOW_REQUEST) {
    ++g_can_physical_window_request_tx_error_count;
    App_CanPhysical_RequestLocalStop(APP_CAN_LOCAL_STOP_TX_FAILURE);
    return;
  }
  if (tx_kind == APP_CAN_TX_T04) {
    App_CanPhysical_FailT04();
    return;
  }
  if ((tx_kind == APP_CAN_TX_T04_INJECT) ||
      (tx_kind == APP_CAN_TX_T04_RELEASE)) {
    App_CanPhysical_FailT04();
    return;
  }
  if (tx_kind == APP_CAN_TX_MATRIX) {
    App_CanPhysical_BeginSafeRecovery();
    return;
  }
  if (tx_kind == APP_CAN_TX_STARTUP_STOP) {
    ++g_can_physical_stop_tx_error_count;
  } else if (tx_kind == APP_CAN_TX_CLEAR) {
    ++g_can_physical_clear_tx_error_count;
  } else if (tx_kind == APP_CAN_TX_FINAL_STOP) {
    ++g_can_physical_final_stop_tx_error_count;
  }
  if ((tx_kind == APP_CAN_TX_STARTUP_STOP) ||
      (tx_kind == APP_CAN_TX_CLEAR) ||
      (tx_kind == APP_CAN_TX_FINAL_STOP)) {
    if (!can_recovery_client_note_tx_failure_at(
            &s_recovery_client, HAL_GetTick())) {
      g_can_physical_recovery_sequence_error_site = 1U;
      g_can_physical_recovery_test_result =
          APP_CAN_RECOVERY_RESULT_SEQUENCE_ERROR;
      s_tx_stopped = 1U;
    }
  } else if (tx_kind == APP_CAN_TX_PROBE) {
    g_can_physical_recovery_test_result =
        APP_CAN_RECOVERY_RESULT_TX_FAILED;
    s_tx_stopped = 1U;
  }
}

static void App_CanPhysical_RecordTxSuccess(app_can_tx_kind_t tx_kind,
                                            uint16_t tx_timestamp)
{
  if (tx_kind == APP_CAN_TX_SAFE_STOP) {
    ++g_can_physical_safe_stop_tx_success_count;
    g_can_physical_safe_stop_tx_success_tick_ms = HAL_GetTick();
    s_window_last_tx_timestamp = tx_timestamp;
    s_window_has_tx_timestamp = 1U;
    return;
  }
  if (tx_kind == APP_CAN_TX_WINDOW_REQUEST) {
    ++g_can_physical_window_request_tx_count;
    s_window_last_tx_timestamp = tx_timestamp;
    s_window_has_tx_timestamp = 1U;
    return;
  }
  if (tx_kind == APP_CAN_TX_T04) {
    App_CanPhysical_RecordT04TxSuccess(tx_timestamp, HAL_GetTick());
    return;
  }
  if (tx_kind == APP_CAN_TX_T04_INJECT) {
    ++g_can_physical_t04_fixture_tx_count;
    s_t04_injection_pending = 0U;
    return;
  }
  if (tx_kind == APP_CAN_TX_T04_RELEASE) {
    ++g_can_physical_t04_fixture_tx_count;
    s_t04_release_sent = 1U;
    s_t04_last_tx_ms = HAL_GetTick();
    s_t04_last_tx_timestamp = tx_timestamp;
    s_t04_has_tx_timestamp = 1U;
    return;
  }
  if (tx_kind == APP_CAN_TX_PROBE) {
    ++g_can_physical_tx_success_count;
    return;
  }

  if (tx_kind == APP_CAN_TX_MATRIX) {
    ++g_can_physical_matrix_tx_count;
    if ((g_can_physical_matrix_stage != APP_CAN_MATRIX_HOLD_STOP) &&
        (s_matrix_waiting == 0U)) {
      s_matrix_step_started_ms = HAL_GetTick();
    }
    s_matrix_last_tx_ms = HAL_GetTick();
    s_matrix_last_tx_timestamp = tx_timestamp;
    if (g_can_physical_matrix_stage != APP_CAN_MATRIX_HOLD_STOP) {
      s_matrix_waiting = 1U;
    }
    return;
  }

  if (tx_kind == APP_CAN_TX_STARTUP_STOP) {
    ++g_can_physical_stop_tx_success_count;
  } else if (tx_kind == APP_CAN_TX_CLEAR) {
    ++g_can_physical_clear_tx_success_count;
  } else if (tx_kind == APP_CAN_TX_FINAL_STOP) {
    ++g_can_physical_final_stop_tx_success_count;
  } else {
    return;
  }

  if (!can_recovery_client_note_tx_success_at(
          &s_recovery_client, HAL_GetTick(), tx_timestamp)) {
    g_can_physical_recovery_sequence_error_site = 2U;
    g_can_physical_recovery_test_result =
        APP_CAN_RECOVERY_RESULT_SEQUENCE_ERROR;
    s_tx_stopped = 1U;
    return;
  }
  App_CanPhysical_UpdateRecoveryStage();
}

static void App_CanPhysical_ServiceTx(void)
{
  static const uint32_t rqcp_flags[3] = {
    CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2
  };
  static const uint32_t txok_flags[3] = {
    CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2
  };
  const uint32_t now_ms = HAL_GetTick();

  if ((s_window_requests.stop_latched == false) &&
      can_recovery_client_take_ack_timeout(&s_recovery_client, now_ms)) {
    if (s_recovery_client.stage ==
        CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS) {
      ++g_can_physical_stop_ack_timeout_count;
    } else if (s_recovery_client.stage ==
               CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS) {
      ++g_can_physical_clear_ack_timeout_count;
    } else if (s_recovery_client.stage ==
               CAN_RECOVERY_CLIENT_WAIT_FINAL_STOP_STATUS) {
      ++g_can_physical_final_stop_ack_timeout_count;
    }
  }

  if (s_tx_pending != 0U) {
    if (HAL_CAN_IsTxMessagePending(s_can, s_tx_mailbox) != 0U) {
      const bool local_stop_preempts =
          (s_window_requests.stop_latched != false) &&
          (s_tx_kind != APP_CAN_TX_SAFE_STOP);
      const bool transmit_timed_out =
          (uint32_t)(now_ms - s_tx_requested_at_ms) > 200U;
      if ((s_tx_abort_requested == 0U) &&
          (local_stop_preempts || transmit_timed_out)) {
        (void)HAL_CAN_AbortTxRequest(s_can, s_tx_mailbox);
        s_tx_abort_requested = 1U;
      }
      return;
    }
    uint32_t mailbox_index = (s_tx_mailbox == CAN_TX_MAILBOX0) ? 0U :
                             (s_tx_mailbox == CAN_TX_MAILBOX1) ? 1U : 2U;
    uint32_t tsr = s_can->Instance->TSR;
    if ((tsr & rqcp_flags[mailbox_index]) == 0U ||
        (tsr & txok_flags[mailbox_index]) == 0U) {
      App_CanPhysical_RecordTxFailure(s_tx_kind);
    } else {
      App_CanPhysical_RecordTxSuccess(
          s_tx_kind, (uint16_t)HAL_CAN_GetTxTimestamp(s_can, s_tx_mailbox));
    }
    s_can->Instance->TSR = rqcp_flags[mailbox_index];
    s_tx_pending = 0U;
    s_tx_abort_requested = 0U;
    s_tx_kind = APP_CAN_TX_NONE;
  }
  if (((s_tx_stopped != 0U) &&
       (s_window_requests.stop_latched == false)) ||
      (s_tx_pending != 0U)) {
    return;
  }

  if ((g_can_physical_t04_arm != 0U) &&
      (g_can_physical_t04_start != 0U) &&
      (g_can_physical_t04_result == APP_CAN_T04_NOT_STARTED) &&
      (g_can_physical_recovery_test_result == APP_CAN_RECOVERY_RESULT_PASS)) {
    g_can_physical_t04_stage = APP_CAN_T04_SEND_BASELINE_STOP;
    g_can_physical_t04_result = APP_CAN_T04_RUNNING;
    g_can_physical_t04_tx_count = 0U;
    g_can_physical_t04_status_match_count = 0U;
    g_can_physical_t04_unexpected_status_count = 0U;
    g_can_physical_t04_fixture_tx_count = 0U;
    s_t04_injection_pending = 1U;
    s_t04_release_sent = 0U;
    s_t04_has_tx_timestamp = 0U;
  }

  if ((g_can_physical_recovery_test_result == APP_CAN_RECOVERY_RESULT_PASS) &&
      (g_can_physical_matrix_result == APP_CAN_MATRIX_NOT_STARTED) &&
      (g_can_physical_matrix_enable != 0U)) {
    g_can_physical_matrix_enable = 0U;
    g_can_physical_matrix_result = APP_CAN_MATRIX_RUNNING;
    g_can_physical_matrix_stage = APP_CAN_MATRIX_STEP_UP;
    s_matrix_waiting = 0U;
    s_matrix_last_tx_ms = now_ms;
  }

  can_protocol_frame_t frame = {0};
  CAN_TxHeaderTypeDef header = {0};
  app_can_tx_kind_t tx_kind;
  window_request_t request = {0};
  if (s_window_requests.stop_latched != false) {
    can_protocol_command_t command = {0};
    if (!window_request_peek(&s_window_requests, &request)) {
      return;
    }
    if ((s_window_has_tx_attempt != 0U) &&
        ((uint32_t)(now_ms - s_window_last_tx_attempt_ms) <
         CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS)) {
      return;
    }
    command.sequence = request.sequence;
    command.command = CAN_PROTOCOL_CMD_STOP;
    if (!can_protocol_encode_command(&command, &frame)) {
      return;
    }
    tx_kind = APP_CAN_TX_SAFE_STOP;
    s_window_last_tx_attempt_ms = now_ms;
    s_window_has_tx_attempt = 1U;
  } else if ((g_can_physical_recovery_test_result ==
              APP_CAN_RECOVERY_RESULT_PASS) &&
             window_request_peek(&s_window_requests, &request)) {
    can_protocol_command_t command = {0};
    if ((s_window_has_tx_attempt != 0U) &&
        ((uint32_t)(now_ms - s_window_last_tx_attempt_ms) <
         CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS)) {
      return;
    }
    command.sequence = request.sequence;
    command.command = (request.command == WINDOW_REQUEST_UP) ?
        CAN_PROTOCOL_CMD_UP :
        (request.command == WINDOW_REQUEST_DOWN) ?
        CAN_PROTOCOL_CMD_DOWN :
        (request.command == WINDOW_REQUEST_CLEAR) ?
        CAN_PROTOCOL_CMD_CLEAR_FAULT :
        (request.command == WINDOW_REQUEST_DEMO_SET_ZERO) ?
        CAN_PROTOCOL_CMD_DEMO_SET_ZERO :
        (request.command == WINDOW_REQUEST_DEMO_TOGGLE) ?
        CAN_PROTOCOL_CMD_DEMO_TOGGLE : CAN_PROTOCOL_CMD_STOP;
    if (!can_protocol_encode_command(&command, &frame)) {
      return;
    }
    tx_kind = APP_CAN_TX_WINDOW_REQUEST;
    s_window_last_tx_attempt_ms = now_ms;
    s_window_has_tx_attempt = 1U;
  } else if (App_CanPhysical_T04IsActive()) {
    const app_can_t04_stage_t stage =
        (app_can_t04_stage_t)g_can_physical_t04_stage;

    if (s_t04_injection_pending != 0U) {
      if (!can_test_frame_encode_t04_control(
              CAN_TEST_T04_ACTION_INJECT_OVERFLOW, &frame)) {
        App_CanPhysical_FailT04();
        return;
      }
      tx_kind = APP_CAN_TX_T04_INJECT;
    } else if ((stage == APP_CAN_T04_WAIT_CONDITION_RELEASE) &&
               (s_t04_release_sent == 0U) &&
               ((uint32_t)(now_ms - s_t04_wait_started_ms) >=
                APP_CAN_T04_CONDITION_RELEASE_DELAY_MS) &&
               ((uint32_t)(now_ms - s_t04_last_tx_ms) >=
                CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS)) {
      if (!can_test_frame_encode_t04_control(
              CAN_TEST_T04_ACTION_RELEASE_OVERFLOW, &frame)) {
        App_CanPhysical_FailT04();
        return;
      }
      tx_kind = APP_CAN_TX_T04_RELEASE;
    } else {
      can_protocol_command_t command = {0};
      if (!App_CanPhysical_T04CommandDue(now_ms)) {
        return;
      }
      if (!App_CanPhysical_T04CommandForStage(stage, &command) ||
          !can_protocol_encode_command(&command, &frame)) {
        App_CanPhysical_FailT04();
        return;
      }
      tx_kind = APP_CAN_TX_T04;
    }
  } else if ((g_can_physical_t04_arm != 0U) &&
             (g_can_physical_t04_start != 0U) &&
             (g_can_physical_t04_result == APP_CAN_T04_NOT_STARTED)) {
    return;
  } else if (App_CanPhysical_MatrixIsActive()) {
    const app_can_matrix_step_t *step = App_CanPhysical_GetMatrixStep();
    can_protocol_command_t command = {0};

    if (g_can_physical_matrix_stage == APP_CAN_MATRIX_HOLD_STOP) {
      if ((uint32_t)(now_ms - s_matrix_last_tx_ms) <
          CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS) {
        return;
      }
      command.sequence = 0x01U;
      command.command = CAN_PROTOCOL_CMD_STOP;
    } else {
      if (step == NULL) {
        g_can_physical_matrix_result = APP_CAN_MATRIX_FAIL;
        g_can_physical_matrix_stage = APP_CAN_MATRIX_FAILED;
        s_tx_stopped = 1U;
        return;
      }
      if (s_matrix_waiting != 0U) {
        if ((g_can_physical_matrix_stage != APP_CAN_MATRIX_HOLD_UP) &&
            ((uint32_t)(now_ms - s_matrix_step_started_ms) >=
             CAN_RECOVERY_CLIENT_ACK_DEADLINE_MS)) {
          App_CanPhysical_BeginSafeRecovery();
          return;
        }
        if ((uint32_t)(now_ms - s_matrix_last_tx_ms) <
            CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS) {
          return;
        }
      }
      command.sequence = step->sequence;
      command.command = step->command;
      s_matrix_last_sequence = step->sequence;
    }

    if (!can_protocol_encode_command(&command, &frame)) {
      g_can_physical_recovery_sequence_error_site = 3U;
      g_can_physical_matrix_result = APP_CAN_MATRIX_FAIL;
      g_can_physical_matrix_stage = APP_CAN_MATRIX_FAILED;
      s_tx_stopped = 1U;
      return;
    }
    if ((step != NULL) &&
        (g_can_physical_matrix_stage != APP_CAN_MATRIX_HOLD_STOP)) {
      switch ((app_can_matrix_frame_kind_t)step->invalid_frame) {
      case APP_CAN_MATRIX_INVALID_DLC:
        frame.dlc = 3U;
        break;
      case APP_CAN_MATRIX_INVALID_VERSION:
        frame.data[0] = (uint8_t)(CAN_PROTOCOL_VERSION + 1U);
        break;
      case APP_CAN_MATRIX_INVALID_COMMAND:
        frame.data[2] = 0xFFU;
        break;
      case APP_CAN_MATRIX_INVALID_RESERVED:
        frame.data[3] = 1U;
        break;
      default:
        break;
      }
    }
    tx_kind = APP_CAN_TX_MATRIX;
  } else if (g_can_physical_tx_success_count < CAN_TEST_FRAME_COUNT) {
    tx_kind = APP_CAN_TX_PROBE;
    can_test_frame_encode(CAN_TEST_FRAME_F407_TO_G3507_ID,
                          g_can_physical_tx_success_count + 1U, &frame);
  } else {
    can_protocol_command_t command = {0};
    if (!can_recovery_client_get_command_at(
            &s_recovery_client, now_ms, &command)) {
      return;
    }
    if (!can_protocol_encode_command(&command, &frame)) {
      ++g_can_physical_tx_error_count;
      g_can_physical_recovery_test_result =
          APP_CAN_RECOVERY_RESULT_SEQUENCE_ERROR;
      s_tx_stopped = 1U;
      return;
    }
    switch (s_recovery_client.stage) {
    case CAN_RECOVERY_CLIENT_SEND_STARTUP_STOP:
    case CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS:
      tx_kind = APP_CAN_TX_STARTUP_STOP;
      break;
    case CAN_RECOVERY_CLIENT_SEND_CLEAR:
    case CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS:
      tx_kind = APP_CAN_TX_CLEAR;
      break;
    case CAN_RECOVERY_CLIENT_SEND_FINAL_STOP:
    case CAN_RECOVERY_CLIENT_WAIT_FINAL_STOP_STATUS:
    case CAN_RECOVERY_CLIENT_COMPLETE:
      tx_kind = APP_CAN_TX_FINAL_STOP;
      break;
    default:
      g_can_physical_recovery_sequence_error_site = 4U;
      g_can_physical_recovery_test_result =
          APP_CAN_RECOVERY_RESULT_SEQUENCE_ERROR;
      s_tx_stopped = 1U;
      return;
    }
  }
  header.StdId = frame.id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = frame.dlc;
  header.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_AddTxMessage(s_can, &header, frame.data,
                           &s_tx_mailbox) != HAL_OK) {
    App_CanPhysical_RecordTxFailure(tx_kind);
    return;
  }
  if (tx_kind == APP_CAN_TX_STARTUP_STOP) {
    ++g_can_physical_stop_tx_request_count;
    g_can_physical_recovery_test_result = APP_CAN_RECOVERY_RESULT_RUNNING;
  } else if (tx_kind == APP_CAN_TX_CLEAR) {
    ++g_can_physical_clear_tx_request_count;
  } else if (tx_kind == APP_CAN_TX_FINAL_STOP) {
    ++g_can_physical_final_stop_tx_request_count;
  } else if (tx_kind == APP_CAN_TX_T04) {
    /* T04 commands have their own per-event accounting. */
  } else if (tx_kind == APP_CAN_TX_SAFE_STOP) {
    ++g_can_physical_safe_stop_tx_request_count;
    g_can_physical_safe_stop_tx_request_tick_ms = HAL_GetTick();
  } else {
    ++g_can_physical_tx_request_count;
  }
  s_tx_requested_at_ms = now_ms;
  s_tx_pending = 1U;
  s_tx_kind = tx_kind;
}

uint8_t App_CanPhysical_Configure(CAN_HandleTypeDef *can)
{
  CAN_FilterTypeDef filter = {0};

  if ((can == NULL) || (can->Init.TimeTriggeredMode != ENABLE)) {
    return 0U;
  }
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterMaskIdHigh = 0x7FFU << 5;
  filter.FilterMaskIdLow = 0x0006U; /* Standard data frame only. */
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  filter.FilterBank = 0U;
  filter.FilterIdHigh = CAN_PROTOCOL_STATUS_ID << 5;
  if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK) {
    g_can_physical_last_hal_error = HAL_CAN_GetError(can);
    return 0U;
  }
  filter.FilterBank = 1U;
  filter.FilterIdHigh = CAN_TEST_FRAME_G3507_TO_F407_ID << 5;
  if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK) {
    g_can_physical_last_hal_error = HAL_CAN_GetError(can);
    return 0U;
  }

  s_can = can;
  can_recovery_client_init(&s_recovery_client,
                           CAN_PHYSICAL_STARTUP_STOP_SEQUENCE);
  s_latest_status = (app_can_status_snapshot_t){0};
  s_rx_queue = NULL;
  s_status_mutex = NULL;
  s_rx_notification_paused = 0U;
  s_rx_overflow_latched = 0U;
  s_rx_overflow_cause_active = 0U;
  s_rx_fifo_overrun_latched = 0U;
  window_request_init(&s_window_requests);
  app_button_init(&s_button,
                  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET,
                  HAL_GetTick());
  s_button_pending_command = WINDOW_REQUEST_NONE;
  s_button_pending_sequence = 0U;
  s_button_demo_motion_seen = 0U;
  s_button_pending_since_ms = 0U;
  app_button_deferred_toggle_cancel(&s_button_deferred_toggle);
  s_window_has_tx_attempt = 0U;
  s_window_has_tx_timestamp = 0U;
  g_can_physical_rx_overflow_latched = 0U;
  g_can_physical_rx_overflow_active = 0U;
  g_can_physical_rx_overflow_first_tick_ms = 0U;
  g_can_physical_rx_status_last_received_tick_ms = 0U;
  g_can_physical_rx_status_last_processed_tick_ms = 0U;
  g_can_physical_local_stop_latched = 0U;
  g_can_physical_local_stop_reason = APP_CAN_LOCAL_STOP_NONE;
  g_can_physical_local_stop_latched_tick_ms = 0U;
  g_can_physical_safe_stop_tx_request_tick_ms = 0U;
  g_can_physical_safe_stop_tx_success_tick_ms = 0U;
  g_can_physical_safe_stop_status_match_tick_ms = 0U;
  g_can_physical_rx_queue_current = 0U;
  App_CanPhysical_UpdateRecoveryStage();
  g_can_physical_recovery_test_result =
      APP_CAN_RECOVERY_RESULT_NOT_STARTED;
  return 1U;
}

uint8_t App_CanPhysical_Start(void)
{
  if ((s_can == NULL) || (s_rx_queue == NULL) || (s_status_mutex == NULL)) {
    return 0U;
  }
  if (HAL_CAN_Start(s_can) != HAL_OK) {
    g_can_physical_last_hal_error = HAL_CAN_GetError(s_can);
    return 0U;
  }
  if (HAL_CAN_ActivateNotification(
          s_can, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN) !=
      HAL_OK) {
    g_can_physical_last_hal_error = HAL_CAN_GetError(s_can);
    (void)HAL_CAN_Stop(s_can);
    return 0U;
  }
  s_can_started_at_ms = HAL_GetTick();
  g_can_physical_start_result = 1U;
  return 1U;
}

osThreadId_t App_CanPhysical_CreateRxTask(void)
{
  static const osThreadAttr_t attributes = {
    .name = "canPhyRx",
    .priority = osPriorityBelowNormal,
    .stack_size = 512U
  };

  if ((s_can == NULL) || (s_rx_queue != NULL) || (s_status_mutex != NULL)) {
    return NULL;
  }
  s_rx_queue = osMessageQueueNew(APP_CAN_RX_QUEUE_CAPACITY,
                                 sizeof(app_can_rx_message_t), NULL);
  s_status_mutex = osMutexNew(NULL);
  if ((s_rx_queue == NULL) || (s_status_mutex == NULL)) {
    g_can_physical_task_create_failure = 1U;
    return NULL;
  }
  g_can_physical_rx_queue_create_ok = 1U;
  s_rx_task = osThreadNew(App_CanPhysical_RxTask, NULL, &attributes);
  if (s_rx_task == NULL) {
    g_can_physical_task_create_failure = 2U;
  }
  return s_rx_task;
}

static uint8_t App_CanPhysical_NextSequence(void)
{
  app_can_status_snapshot_t snapshot = {0};
  uint8_t next_sequence;

  if ((s_status_mutex != NULL) &&
      (osMutexAcquire(s_status_mutex, osWaitForever) == osOK)) {
    snapshot = s_latest_status;
    (void)osMutexRelease(s_status_mutex);
  }
  if ((snapshot.valid != 0U) && snapshot.status.last_sequence_valid) {
    next_sequence = (uint8_t)(snapshot.status.last_sequence + 1U);
  } else {
    next_sequence = (uint8_t)(s_recovery_client.next_sequence + 1U);
  }
  /* An unacknowledged motion may already be on the wire. A STOP with the
   * same sequence would be rejected as a conflicting event by the peer. */
  if (s_window_requests.latest_valid &&
      (s_window_requests.latest.sequence == next_sequence)) {
    ++next_sequence;
  }
  return next_sequence;
}

static void App_CanPhysical_RequestLocalStop(
    app_can_local_stop_reason_t reason)
{
  app_button_deferred_toggle_cancel(&s_button_deferred_toggle);
  s_button_pending_command = WINDOW_REQUEST_NONE;
  s_button_demo_motion_seen = 0U;
  if (s_window_requests.stop_latched != false) {
    if ((g_can_physical_local_stop_reason == APP_CAN_LOCAL_STOP_USER_BUTTON) &&
        (reason != APP_CAN_LOCAL_STOP_USER_BUTTON)) {
      g_can_physical_local_stop_reason = (uint32_t)reason;
    }
    return;
  }

  const uint8_t stop_sequence = App_CanPhysical_NextSequence();
  if (!window_request_submit(&s_window_requests, (window_request_t){
          .command = WINDOW_REQUEST_STOP,
          .sequence = stop_sequence
      })) {
    return;
  }
  g_can_physical_local_stop_latched = 1U;
  g_can_physical_local_stop_reason = (uint32_t)reason;
  g_can_physical_local_stop_latched_tick_ms = HAL_GetTick();
  g_can_physical_safe_stop_tx_request_tick_ms = 0U;
  g_can_physical_safe_stop_tx_success_tick_ms = 0U;
  g_can_physical_safe_stop_status_match_tick_ms = 0U;
  s_window_has_tx_attempt = 0U;
  s_window_has_tx_timestamp = 0U;
  s_tx_stopped = 0U;
  if (App_CanPhysical_MatrixIsActive()) {
    g_can_physical_matrix_result = APP_CAN_MATRIX_FAIL;
    g_can_physical_matrix_stage = APP_CAN_MATRIX_FAILED;
    s_matrix_waiting = 0U;
  }
  if (App_CanPhysical_T04IsActive()) {
    App_CanPhysical_FailT04();
  }
}

static void App_CanPhysical_ProcessDebugRequests(void)
{
  if (g_can_physical_window_request_inject != 0U) {
    const uint32_t request = g_can_physical_window_request_inject;
    g_can_physical_window_request_inject = 0U;
    const uint8_t sequence = App_CanPhysical_NextSequence();
    const bool accepted = (request <= WINDOW_REQUEST_DEMO_TOGGLE) &&
        window_request_command_is_valid((window_request_command_t)request) &&
        window_request_submit(&s_window_requests,
          (window_request_t){.command = (window_request_command_t)request,
                             .sequence = sequence});
    g_can_physical_window_request_inject_result = accepted ? 1U : 2U;
    g_can_physical_window_request_inject_sequence = sequence;
    if (accepted) {
      app_button_deferred_toggle_cancel(&s_button_deferred_toggle);
      s_button_pending_command = WINDOW_REQUEST_NONE;
      s_button_demo_motion_seen = 0U;
    }
    s_window_has_tx_attempt = 0U;
  }
  if (g_can_physical_rx_overflow_latched != 0U) {
    const app_can_local_stop_reason_t reason =
        (s_rx_fifo_overrun_latched != 0U) ?
        APP_CAN_LOCAL_STOP_RX_FIFO_OVERRUN :
        APP_CAN_LOCAL_STOP_RX_QUEUE_OVERFLOW;
    App_CanPhysical_RequestLocalStop(reason);
  }
  if (g_can_physical_rx_remote_offline != 0U) {
    App_CanPhysical_RequestLocalStop(APP_CAN_LOCAL_STOP_STATUS_OFFLINE);
  }

  if (g_can_physical_window_request_up_inject != 0U) {
    g_can_physical_window_request_up_inject = 0U;
    const uint8_t sequence = App_CanPhysical_NextSequence();
    const bool accepted = window_request_submit(&s_window_requests,
        (window_request_t){
            .command = WINDOW_REQUEST_UP,
            .sequence = sequence
        });
    g_can_physical_window_request_up_sequence = sequence;
    g_can_physical_window_request_up_result = accepted ? 1U : 2U;
    s_window_has_tx_attempt = 0U;
  }

  if (g_can_physical_window_request_rearm != 0U) {
    g_can_physical_window_request_rearm = 0U;
    const uint8_t recovery_sequence = App_CanPhysical_NextSequence();
    const bool cause_cleared =
        (g_can_physical_rx_overflow_active == 0U) &&
        (g_can_physical_rx_remote_offline == 0U);
    if (cause_cleared && window_request_rearm(&s_window_requests)) {
      g_can_physical_local_stop_latched = 0U;
      g_can_physical_local_stop_reason = APP_CAN_LOCAL_STOP_NONE;
      g_can_physical_window_request_rearm_result = 1U;
      /* Explicit rearm starts a fresh handshake, never an old final STOP seq. */
      App_CanPhysical_RestartRecovery(recovery_sequence, false);
      s_window_has_tx_attempt = 0U;
      s_window_has_tx_timestamp = 0U;
      if (s_rx_overflow_latched != 0U) {
        taskENTER_CRITICAL();
        s_rx_overflow_latched = 0U;
        s_rx_fifo_overrun_latched = 0U;
        g_can_physical_rx_overflow_latched = 0U;
        taskEXIT_CRITICAL();
      }
    } else {
      g_can_physical_window_request_rearm_result = 2U;
    }
  }
}

static void App_CanPhysical_ServiceButton(void)
{
  app_can_status_snapshot_t snapshot = {0};
  const uint32_t now_ms = HAL_GetTick();
  const bool pressed = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
  app_button_event_t event;
  bool status_ready;
  bool button_safe;
  bool motion_pending;

  g_can_physical_button_raw = pressed ? 1U : 0U;
  if ((s_status_mutex != NULL) &&
      (osMutexAcquire(s_status_mutex, osWaitForever) == osOK)) {
    snapshot = s_latest_status;
    (void)osMutexRelease(s_status_mutex);
  }
  status_ready = (snapshot.valid != 0U) &&
      ((uint32_t)(now_ms - snapshot.received_at_ms) <=
       APP_CAN_REMOTE_OFFLINE_LIMIT_MS) &&
      snapshot.status.last_sequence_valid &&
      (snapshot.status.fault == CAN_PROTOCOL_FAULT_NONE) &&
      (s_recovery_client.stage == CAN_RECOVERY_CLIENT_COMPLETE);
  if (!status_ready || s_window_requests.stop_latched) {
    /* A fault or local STOP must discard the previous button operation. */
    s_button_pending_command = WINDOW_REQUEST_NONE;
    s_button_demo_motion_seen = 0U;
    app_button_deferred_toggle_cancel(&s_button_deferred_toggle);
  }
  if (status_ready && (s_button_pending_command != WINDOW_REQUEST_NONE) &&
      (snapshot.status.last_sequence == s_button_pending_sequence)) {
    if (s_button_pending_command == WINDOW_REQUEST_DEMO_SET_ZERO) {
      if (snapshot.status.is_calibrated) {
        s_button_pending_command = WINDOW_REQUEST_NONE;
      }
    } else if (snapshot.status.state != CAN_PROTOCOL_STATE_STOP) {
      s_button_demo_motion_seen = 1U;
    } else if (s_button_demo_motion_seen != 0U) {
      s_button_pending_command = WINDOW_REQUEST_NONE;
    }
  }
  if ((s_button_pending_command == WINDOW_REQUEST_DEMO_SET_ZERO) &&
      ((uint32_t)(now_ms - s_button_pending_since_ms) >
       APP_BUTTON_ZERO_CONFIRM_TIMEOUT_MS)) {
    s_button_pending_command = WINDOW_REQUEST_NONE;
    app_button_deferred_toggle_cancel(&s_button_deferred_toggle);
  }
  button_safe = status_ready &&
      (snapshot.status.state == CAN_PROTOCOL_STATE_STOP) &&
      (s_window_requests.stop_latched == false) &&
      (g_can_physical_rx_remote_offline == 0U) &&
      (g_can_physical_rx_overflow_active == 0U) &&
      !App_CanPhysical_MatrixIsActive() && !App_CanPhysical_T04IsActive();
  if (app_button_deferred_toggle_take(
          &s_button_deferred_toggle, button_safe,
          snapshot.status.is_calibrated,
          snapshot.status.last_sequence, now_ms)) {
    const uint8_t sequence = App_CanPhysical_NextSequence();
    if (window_request_submit(&s_window_requests,
                              (window_request_t){WINDOW_REQUEST_DEMO_TOGGLE,
                                                 sequence})) {
      s_button_pending_command = WINDOW_REQUEST_DEMO_TOGGLE;
      s_button_pending_sequence = sequence;
      s_button_demo_motion_seen = 0U;
      s_button_pending_since_ms = now_ms;
      s_window_has_tx_attempt = 0U;
    } else {
      ++g_can_physical_button_rejected_count;
    }
  }
  motion_pending =
      s_button_deferred_toggle.armed ||
      (s_button_pending_command == WINDOW_REQUEST_DEMO_TOGGLE) ||
      (status_ready && (snapshot.status.state != CAN_PROTOCOL_STATE_STOP));
  event = app_button_update(&s_button, pressed, motion_pending, now_ms);
  if (event == APP_BUTTON_NONE) {
    return;
  }
  g_can_physical_button_last_event = (uint32_t)event;
  ++g_can_physical_button_event_count;
  {
    const app_button_recovery_context_t recovery_context = {
      .status_fresh = (snapshot.valid != 0U) &&
          ((uint32_t)(now_ms - snapshot.received_at_ms) <=
           APP_CAN_REMOTE_OFFLINE_LIMIT_MS) &&
          snapshot.status.last_sequence_valid,
      .remote_motor_fault =
          snapshot.status.fault == CAN_PROTOCOL_FAULT_MOTOR_LOCAL,
      .remote_stopped = snapshot.status.state == CAN_PROTOCOL_STATE_STOP,
      .stop_confirmed = s_window_requests.stop_acknowledged,
      .remote_motor_stop_latched =
          (s_window_requests.stop_latched != false) &&
          (g_can_physical_local_stop_reason ==
           APP_CAN_LOCAL_STOP_REMOTE_MOTOR),
      .rx_overflow = (g_can_physical_rx_overflow_active != 0U) ||
          (s_rx_overflow_latched != 0U),
      .remote_offline = g_can_physical_rx_remote_offline != 0U
    };
    if (app_button_requests_motor_recovery(event, &recovery_context)) {
      const uint8_t recovery_sequence = App_CanPhysical_NextSequence();
      if (window_request_rearm(&s_window_requests)) {
        g_can_physical_local_stop_latched = 0U;
        g_can_physical_local_stop_reason = APP_CAN_LOCAL_STOP_NONE;
        g_can_physical_window_request_rearm_result = 1U;
        App_CanPhysical_RestartRecovery(recovery_sequence, false);
        s_window_has_tx_attempt = 0U;
        s_window_has_tx_timestamp = 0U;
      } else {
        g_can_physical_window_request_rearm_result = 2U;
        ++g_can_physical_button_rejected_count;
      }
      return;
    }
  }
  if (event == APP_BUTTON_STOP) {
    App_CanPhysical_RequestLocalStop(APP_CAN_LOCAL_STOP_USER_BUTTON);
    return;
  }
  if ((event == APP_BUTTON_TOGGLE) &&
      (s_button_pending_command == WINDOW_REQUEST_DEMO_SET_ZERO) &&
      button_safe) {
    app_button_deferred_toggle_arm(&s_button_deferred_toggle,
                                   s_button_pending_sequence, now_ms);
    return;
  }
  if ((s_window_requests.stop_latched != false) &&
      (g_can_physical_local_stop_reason == APP_CAN_LOCAL_STOP_USER_BUTTON) &&
      s_window_requests.stop_acknowledged && status_ready &&
      (snapshot.status.state == CAN_PROTOCOL_STATE_STOP) &&
      (g_can_physical_rx_overflow_active == 0U) &&
      (g_can_physical_rx_remote_offline == 0U)) {
    (void)window_request_rearm(&s_window_requests);
    g_can_physical_local_stop_latched = 0U;
    g_can_physical_local_stop_reason = APP_CAN_LOCAL_STOP_NONE;
    s_window_has_tx_attempt = 0U;
  }
  if (!status_ready ||
      (snapshot.status.state != CAN_PROTOCOL_STATE_STOP) ||
      (s_window_requests.stop_latched != false) ||
      (s_button_pending_command != WINDOW_REQUEST_NONE) ||
      App_CanPhysical_MatrixIsActive() || App_CanPhysical_T04IsActive() ||
      ((event == APP_BUTTON_TOGGLE) && !snapshot.status.is_calibrated) ||
      ((event == APP_BUTTON_SET_ZERO) && snapshot.status.is_calibrated)) {
    ++g_can_physical_button_rejected_count;
    return;
  }
  const window_request_command_t command =
      (event == APP_BUTTON_SET_ZERO) ? WINDOW_REQUEST_DEMO_SET_ZERO :
                                      WINDOW_REQUEST_DEMO_TOGGLE;
  const uint8_t sequence = App_CanPhysical_NextSequence();
  if (!window_request_submit(&s_window_requests,
                             (window_request_t){command, sequence})) {
    ++g_can_physical_button_rejected_count;
    return;
  }
  s_button_pending_command = command;
  s_button_pending_sequence = sequence;
  s_button_demo_motion_seen = 0U;
  s_button_pending_since_ms = now_ms;
  s_window_has_tx_attempt = 0U;
}

static void App_CanPhysical_UpdateRxOverflowCause(void)
{
  taskENTER_CRITICAL();
  if ((osMessageQueueGetCount(s_rx_queue) == 0U) &&
      (HAL_CAN_GetRxFifoFillLevel(s_can, CAN_RX_FIFO0) == 0U) &&
      (__HAL_CAN_GET_FLAG(s_can, CAN_FLAG_FOV0) == RESET)) {
    s_rx_overflow_cause_active = 0U;
    g_can_physical_rx_overflow_active = 0U;
  }
  taskEXIT_CRITICAL();
}

static void App_CanPhysical_RxTask(void *argument)
{
  (void)argument;
  for (;;) {
    app_can_rx_message_t message = {0};
    can_protocol_frame_t frame = {0};
    can_protocol_status_t status = {0};

    if (g_can_physical_rx_task_test_pause != 0U) {
      osDelay(1U);
      continue;
    }
    if (osMessageQueueGet(s_rx_queue, &message, NULL, osWaitForever) != osOK) {
      ++g_can_physical_rx_queue_drop_count;
      continue;
    }
    ++g_can_physical_rx_queue_get_count;
    g_can_physical_rx_queue_current = osMessageQueueGetCount(s_rx_queue);
    App_CanPhysical_UpdateRxOverflowCause();

    if (s_rx_notification_paused != 0U) {
      taskENTER_CRITICAL();
      if ((s_rx_notification_paused != 0U) &&
          (HAL_CAN_ActivateNotification(
              s_can, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)) {
        s_rx_notification_paused = 0U;
        ++g_can_physical_rx_notification_resume_count;
      }
      taskEXIT_CRITICAL();
    }

    if ((message.standard_id > 0x7FFU) || (message.data_length > 8U)) {
      ++g_can_physical_invalid_rx_count;
      continue;
    }
    frame.id = (uint16_t)message.standard_id;
    frame.dlc = (uint8_t)message.data_length;
    frame.is_extended = (message.identifier_type != CAN_ID_STD);
    frame.is_remote = (message.frame_type != CAN_RTR_DATA);
    for (uint32_t i = 0U; i < sizeof(frame.data); ++i) {
      frame.data[i] = message.data[i];
    }

    if (frame.id == CAN_TEST_FRAME_G3507_TO_F407_ID) {
      uint32_t counter = 0U;
      ++g_can_physical_probe_frame_count;
      if (!can_test_frame_decode(
              &frame, CAN_TEST_FRAME_G3507_TO_F407_ID, &counter)) {
        ++g_can_physical_probe_invalid_count;
        continue;
      }
      ++g_can_physical_probe_valid_count;
      if (g_can_physical_probe_valid_count == 1U) {
        g_can_physical_probe_first_counter = counter;
        g_can_physical_probe_last_counter = counter;
        g_can_physical_probe_missing_count = counter - 1U;
      } else if (counter == g_can_physical_probe_last_counter) {
        ++g_can_physical_probe_duplicate_count;
      } else if (counter < g_can_physical_probe_last_counter) {
        ++g_can_physical_probe_stale_count;
      } else {
        g_can_physical_probe_missing_count +=
            counter - g_can_physical_probe_last_counter - 1U;
        g_can_physical_probe_last_counter = counter;
      }
    } else if (can_protocol_decode_status(&frame, &status)) {
      app_can_status_snapshot_t snapshot = {0};
      ++g_can_physical_status_rx_count;
      g_can_physical_last_fault = (uint32_t)status.fault;
      g_can_physical_last_status_sequence = status.last_sequence;
      g_can_physical_last_status_state = (uint32_t)status.state;
      g_can_physical_last_status_flags =
          (status.is_calibrated ? 1U : 0U) |
          (status.last_sequence_valid ? 2U : 0U) |
          (status.rx_overflow_active ? 4U : 0U);
      snapshot.status = status;
      snapshot.received_at_ms = message.received_at_ms;
      snapshot.hardware_timestamp = message.hardware_timestamp;
      snapshot.valid = 1U;
      g_can_physical_rx_status_last_received_tick_ms =
          message.received_at_ms;
      g_can_physical_rx_status_last_processed_tick_ms = HAL_GetTick();
      if (osMutexAcquire(s_status_mutex, osWaitForever) == osOK) {
        snapshot.generation = s_latest_status.generation + 1U;
        s_latest_status = snapshot;
        (void)osMutexRelease(s_status_mutex);
        ++g_can_physical_rx_status_snapshot_count;
      } else {
        ++g_can_physical_rx_queue_drop_count;
      }
    } else {
      ++g_can_physical_invalid_rx_count;
    }
  }
}

static void App_CanPhysical_ServiceLatestStatus(void)
{
  app_can_status_snapshot_t snapshot = {0};
  can_recovery_client_event_t recovery_event =
      CAN_RECOVERY_CLIENT_EVENT_NONE;
  const uint32_t now_ms = HAL_GetTick();

  if ((s_status_mutex == NULL) ||
      (osMutexAcquire(s_status_mutex, osWaitForever) != osOK)) {
    return;
  }
  snapshot = s_latest_status;
  (void)osMutexRelease(s_status_mutex);

  if (snapshot.valid == 0U) {
    const uint32_t age_since_start_ms =
        (uint32_t)(now_ms - s_can_started_at_ms);
    g_can_physical_rx_remote_status_age_ms = age_since_start_ms;
    g_can_physical_rx_remote_offline =
        (age_since_start_ms > APP_CAN_REMOTE_OFFLINE_LIMIT_MS) ? 1U : 0U;
    if (g_can_physical_rx_remote_offline != 0U) {
      App_CanPhysical_RequestLocalStop(APP_CAN_LOCAL_STOP_STATUS_OFFLINE);
    }
    return;
  }
  const uint32_t age_ms = (uint32_t)(now_ms - snapshot.received_at_ms);
  g_can_physical_rx_remote_status_age_ms = age_ms;
  g_can_physical_rx_remote_offline =
      (age_ms > APP_CAN_REMOTE_OFFLINE_LIMIT_MS) ? 1U : 0U;
  if (g_can_physical_rx_remote_offline != 0U) {
    App_CanPhysical_RequestLocalStop(APP_CAN_LOCAL_STOP_STATUS_OFFLINE);
  }
  if (s_rx_overflow_latched != 0U) {
    const app_can_local_stop_reason_t reason =
        (s_rx_fifo_overrun_latched != 0U) ?
        APP_CAN_LOCAL_STOP_RX_FIFO_OVERRUN :
        APP_CAN_LOCAL_STOP_RX_QUEUE_OVERFLOW;
    App_CanPhysical_RequestLocalStop(reason);
  }
  if (snapshot.generation == s_consumed_status_generation) {
    return;
  }
  s_consumed_status_generation = snapshot.generation;
  if (age_ms > APP_CAN_REMOTE_OFFLINE_LIMIT_MS) {
    ++g_can_physical_rx_status_stale_count;
    ++g_can_physical_freshness_blocked_count;
    return;
  }

  if ((snapshot.status.fault == CAN_PROTOCOL_FAULT_MOTOR_LOCAL) &&
      (s_recovery_client.stage == CAN_RECOVERY_CLIENT_COMPLETE)) {
    /* Motor feedback returning cannot automatically authorize CLEAR/restart. */
    App_CanPhysical_RequestLocalStop(APP_CAN_LOCAL_STOP_REMOTE_MOTOR);
  }

  if (s_window_requests.stop_latched != false) {
    if ((s_window_requests.stop_acknowledged != false) &&
        !snapshot.status.last_sequence_valid &&
        window_request_revoke_stop_confirmation(&s_window_requests)) {
      /* The peer lost its STOP sequence baseline; require a fresh STOP TXOK
       * and status match before considering the latched request confirmed. */
      s_window_has_tx_timestamp = 0U;
    }
    const uint16_t elapsed_ticks =
        (uint16_t)(snapshot.hardware_timestamp -
                   s_window_last_tx_timestamp);
    if ((s_window_requests.stop_acknowledged == false) &&
        (s_window_has_tx_timestamp != 0U) &&
        (elapsed_ticks != 0U) && (elapsed_ticks < 0x8000U) &&
        snapshot.status.last_sequence_valid &&
        (snapshot.status.last_sequence ==
         s_window_requests.latched_stop.sequence) &&
        (snapshot.status.state == CAN_PROTOCOL_STATE_STOP)) {
      if (window_request_confirm_stop(
              &s_window_requests,
              s_window_requests.latched_stop.sequence)) {
        ++g_can_physical_safe_stop_status_match_count;
        g_can_physical_safe_stop_status_match_tick_ms =
            snapshot.received_at_ms;
      }
    }
    return;
  }

  App_CanPhysical_ObserveT04Status(
      &snapshot.status, snapshot.hardware_timestamp);
  if ((s_recovery_client.stage == CAN_RECOVERY_CLIENT_COMPLETE) &&
      !App_CanPhysical_MatrixIsActive() &&
      (g_can_physical_t04_arm == 0U) &&
      ((snapshot.status.fault != CAN_PROTOCOL_FAULT_NONE) ||
       !snapshot.status.last_sequence_valid)) {
    App_CanPhysical_RestartRecovery(
        (uint8_t)(s_recovery_client.next_sequence + 1U), false);
  }
  recovery_event = can_recovery_client_observe_status_at(
      &s_recovery_client, &snapshot.status,
      snapshot.hardware_timestamp);
  if (recovery_event == CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED) {
    ++g_can_physical_stop_status_match_count;
  } else if (recovery_event == CAN_RECOVERY_CLIENT_EVENT_CLEAR_CONFIRMED) {
    ++g_can_physical_clear_status_match_count;
  } else if (recovery_event ==
             CAN_RECOVERY_CLIENT_EVENT_FINAL_STOP_CONFIRMED) {
    ++g_can_physical_final_stop_status_match_count;
    g_can_physical_recovery_test_result = APP_CAN_RECOVERY_RESULT_PASS;
  } else if (recovery_event == CAN_RECOVERY_CLIENT_EVENT_CLEAR_REJECTED) {
    ++g_can_physical_clear_rejected_count;
  }
  if (g_can_physical_t04_arm == 0U) {
    App_CanPhysical_ObserveMatrixStatus(
        &snapshot.status, snapshot.hardware_timestamp);
  }
  App_CanPhysical_UpdateRecoveryStage();
}

void App_CanPhysical_TxTask(void *argument)
{
  uint32_t next_wake_tick = osKernelGetTickCount();
  (void)argument;

  for (;;) {
    const uint32_t now_ms = HAL_GetTick();
    if (s_have_tx_task_cycle != 0U) {
      const uint32_t interval_ms =
          (uint32_t)(now_ms - s_last_tx_task_cycle_ms);
      if (interval_ms > g_can_physical_tx_task_max_interval_ms) {
        g_can_physical_tx_task_max_interval_ms = interval_ms;
      }
      if (interval_ms > APP_CAN_TX_CONTROL_PERIOD_MS) {
        ++g_can_physical_tx_task_late_count;
      }
    }
    s_have_tx_task_cycle = 1U;
    s_last_tx_task_cycle_ms = now_ms;
    ++g_can_physical_tx_task_cycle_count;

    App_CanPhysical_ServiceLatestStatus();
    App_CanPhysical_ProcessDebugRequests();
    App_CanPhysical_ServiceButton();
    App_CanPhysical_ServiceTx();

    g_can_physical_tx_task_stack_space_bytes =
        osThreadGetStackSpace(osThreadGetId());
    g_can_physical_rx_task_stack_space_bytes =
        osThreadGetStackSpace(s_rx_task);
    g_can_physical_rx_queue_current = osMessageQueueGetCount(s_rx_queue);
    g_can_physical_heap_free_bytes = (uint32_t)xPortGetFreeHeapSize();
    g_can_physical_heap_min_free_bytes =
        (uint32_t)xPortGetMinimumEverFreeHeapSize();

    next_wake_tick += APP_CAN_TX_CONTROL_PERIOD_MS;
    if (osDelayUntil(next_wake_tick) != osOK) {
      ++g_can_physical_tx_task_late_count;
      next_wake_tick = osKernelGetTickCount();
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t batch_count = 0U;
  const uint32_t interrupt_entry_ms = HAL_GetTick();

  if ((hcan != s_can) || (s_rx_queue == NULL)) {
    return;
  }
  ++g_can_physical_rx_isr_count;
  while ((batch_count < APP_CAN_RX_ISR_DRAIN_BUDGET) &&
         (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U)) {
    CAN_RxHeaderTypeDef header = {0};
    app_can_rx_message_t message = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header,
                             message.data) != HAL_OK) {
      g_can_physical_last_hal_error = HAL_CAN_GetError(hcan);
      break;
    }
    ++batch_count;
    message.standard_id = header.StdId;
    message.identifier_type = header.IDE;
    message.frame_type = header.RTR;
    message.data_length = header.DLC;
    message.hardware_timestamp = (uint16_t)header.Timestamp;
    message.received_at_ms = interrupt_entry_ms;

    if (osMessageQueuePut(s_rx_queue, &message, 0U, 0U) != osOK) {
      ++g_can_physical_rx_queue_drop_count;
      if (s_rx_overflow_latched == 0U) {
        g_can_physical_rx_overflow_first_tick_ms = HAL_GetTick();
      }
      s_rx_overflow_latched = 1U;
      s_rx_overflow_cause_active = 1U;
      g_can_physical_rx_overflow_latched = 1U;
      g_can_physical_rx_overflow_active = 1U;
      __DMB();
      if (s_rx_notification_paused == 0U) {
        s_rx_notification_paused = 1U;
        ++g_can_physical_rx_notification_pause_count;
        (void)HAL_CAN_DeactivateNotification(
            hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
      }
      break;
    }

    ++g_can_physical_rx_queue_put_count;
    g_can_physical_rx_queue_current = osMessageQueueGetCount(s_rx_queue);
    if (g_can_physical_rx_queue_current >
        g_can_physical_rx_queue_high_water) {
      g_can_physical_rx_queue_high_water =
          g_can_physical_rx_queue_current;
    }
  }
  if (batch_count > g_can_physical_rx_isr_max_batch) {
    g_can_physical_rx_isr_max_batch = batch_count;
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if ((hcan == s_can) &&
      ((HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_RX_FOV0) != 0U)) {
    ++g_can_physical_fifo_overrun_count;
    if (s_rx_overflow_latched == 0U) {
      g_can_physical_rx_overflow_first_tick_ms = HAL_GetTick();
    }
    s_rx_overflow_latched = 1U;
    s_rx_overflow_cause_active = 1U;
    s_rx_fifo_overrun_latched = 1U;
    g_can_physical_rx_overflow_latched = 1U;
    g_can_physical_rx_overflow_active = 1U;
    __DMB();
  }
}
