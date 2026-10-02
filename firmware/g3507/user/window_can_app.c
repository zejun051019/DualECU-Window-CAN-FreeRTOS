#include "window_can_app.h"

#include "can_port_mspm0.h"
#include "can_protocol.h"
#include "can_test_frame.h"
#include "mcan_timestamp.h"
#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
#include "zdt_motor_can.h"
#endif
#if defined(DUALECU_ENABLE_MOTOR_STEP_TEST)
#include "motor_port_mspm0.h"
#endif
#if defined(DUALECU_ENABLE_ZDT_UART_BENCH_TEST) || \
    defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
#include "zdt_uart_mspm0.h"
#include "zdt_uart_protocol.h"
#endif
#include "ti_msp_dl_config.h"
#include "window_state.h"
#include "window_demo.h"

#include <stdint.h>

#define WINDOW_CAN_APP_SAFETY_PERIOD_MS        (10U)
#define WINDOW_CAN_APP_COMMAND_PERIOD_MS       (50U)
#define WINDOW_CAN_APP_RX_DEADLINE_MS          (100U)
#define WINDOW_CAN_APP_TEST_TIMEOUT_LIMIT_MS   (350U)
#define WINDOW_CAN_APP_TEST_COMMAND_COUNT      (4U)
#define WINDOW_CAN_APP_RX_BUDGET               (3U)
#define WINDOW_CAN_APP_STATUS_PERIOD_MS        (100U)
#define WINDOW_CAN_APP_PHYSICAL_TEST_PERIOD_MS (10U)
#define WINDOW_CAN_APP_STATUS_TEST_MIN_RX      (5U)
#define WINDOW_CAN_APP_STATUS_TEST_DEADLINE_MS (650U)
#define WINDOW_CAN_APP_STATUS_CADENCE_MAX_MS   (110U)
#define WINDOW_CAN_APP_ZDT_POSITION_PERIOD_MS  (100U)

enum
{
    WINDOW_CAN_APP_INIT_NOT_RUN = 0U,
    WINDOW_CAN_APP_INIT_OK,
    WINDOW_CAN_APP_INIT_FAIL_SYSTICK,
    WINDOW_CAN_APP_INIT_FAIL_MCAN,
    WINDOW_CAN_APP_INIT_FAIL_ZDT_UART
};

enum
{
    WINDOW_CAN_APP_TEST_NOT_RUN = 0U,
    WINDOW_CAN_APP_TEST_RUNNING,
    WINDOW_CAN_APP_TEST_PASS,
    WINDOW_CAN_APP_TEST_FAIL
};

enum
{
    WINDOW_CAN_APP_FAIL_NONE = 0U,
    WINDOW_CAN_APP_FAIL_TX,
    WINDOW_CAN_APP_FAIL_RX_ACK,
    WINDOW_CAN_APP_FAIL_RX_TIMEOUT,
    WINDOW_CAN_APP_FAIL_COMMAND_SEQUENCE,
    WINDOW_CAN_APP_FAIL_TIMEOUT
};

enum
{
    WINDOW_CAN_APP_STATUS_TEST_NOT_RUN = 0U,
    WINDOW_CAN_APP_STATUS_TEST_RUNNING,
    WINDOW_CAN_APP_STATUS_TEST_PASS,
    WINDOW_CAN_APP_STATUS_TEST_FAIL
};

enum
{
    WINDOW_CAN_APP_STATUS_FAIL_NONE = 0U,
    WINDOW_CAN_APP_STATUS_FAIL_INVALID_FRAME,
    WINDOW_CAN_APP_STATUS_FAIL_MISSING_FRAMES,
    WINDOW_CAN_APP_STATUS_FAIL_FINAL_STATE,
    WINDOW_CAN_APP_STATUS_FAIL_TX,
    WINDOW_CAN_APP_STATUS_FAIL_LOOPBACK_PENDING,
    WINDOW_CAN_APP_STATUS_FAIL_CADENCE
};

volatile uint32_t g_window_can_uptime_ms = 0U;
volatile uint32_t g_window_can_init_result = WINDOW_CAN_APP_INIT_NOT_RUN;
volatile uint32_t g_window_can_test_result = WINDOW_CAN_APP_TEST_NOT_RUN;
volatile uint32_t g_window_can_test_failure = WINDOW_CAN_APP_FAIL_NONE;
volatile uint32_t g_window_can_test_tx_count = 0U;
volatile uint32_t g_window_can_test_rx_count = 0U;
volatile uint32_t g_window_can_test_timeout_age_ms = 0U;
volatile uint32_t g_window_can_test_complete_ms = 0U;
volatile uint32_t g_window_can_rx_frame_count = 0U;
volatile uint32_t g_window_can_last_rx_result = 0U;
volatile uint32_t g_window_can_last_rx_ms = 0U;
volatile uint32_t g_window_can_rx_timestamp = 0U;
volatile uint32_t g_window_can_timestamp_now = 0U;
volatile uint32_t g_window_can_timestamp_age_ticks = 0U;
volatile uint32_t g_window_can_last_accepted_ms = 0U;
volatile uint32_t g_window_can_state = CAN_PROTOCOL_STATE_STOP;
/* 1=feedback unsafe, 2=timeout, 3=UART overflow, 4=range, 5=TX/reject, 6=device error. */
volatile uint32_t g_window_can_motor_fault_reason = 0U;
volatile uint32_t g_window_can_motor_fault_code = 0U;
volatile uint32_t g_window_can_motor_fault_detected_ms = 0U;
volatile uint32_t g_window_can_demo_active = 0U;
volatile uint32_t g_window_can_demo_step_count = 0U;
volatile uint32_t g_window_can_demo_complete_count = 0U;
volatile uint32_t g_window_can_demo_fault_count = 0U;
volatile uint32_t g_window_can_demo_zero_count = 0U;
volatile uint32_t g_window_can_motor_restart_rezero_pending = 0U;
volatile uint32_t g_window_can_motor_restart_rezero_samples = 0U;
volatile uint32_t g_window_can_motor_restart_rezero_count = 0U;
volatile uint32_t g_window_can_demo_rejected_count = 0U;
volatile uint32_t g_window_can_fault = CAN_PROTOCOL_FAULT_STARTUP_LOCKED;
volatile uint32_t g_window_can_last_sequence = 0U;
volatile uint32_t g_window_can_last_sequence_valid = 0U;
volatile uint32_t g_window_can_rx_overflow_active = 0U;
volatile uint32_t g_window_can_overflow_observations = 0U;
volatile uint32_t g_window_can_status_test_result =
    WINDOW_CAN_APP_STATUS_TEST_NOT_RUN;
volatile uint32_t g_window_can_status_test_failure =
    WINDOW_CAN_APP_STATUS_FAIL_NONE;
volatile uint32_t g_window_can_status_tx_count = 0U;
volatile uint32_t g_window_can_status_rx_count = 0U;
volatile uint32_t g_window_can_status_first_tx_ms = 0U;
volatile uint32_t g_window_can_status_last_tx_ms = 0U;
volatile uint32_t g_window_can_status_min_interval_ms = 0U;
volatile uint32_t g_window_can_status_max_interval_ms = 0U;
volatile uint32_t g_window_can_status_test_complete_ms = 0U;
volatile uint32_t g_window_can_status_last_sequence = 0U;
volatile uint32_t g_window_can_status_last_state = 0U;
volatile uint32_t g_window_can_status_last_fault = 0U;
volatile uint32_t g_window_can_status_last_position = 0U;
volatile uint32_t g_window_can_status_last_flags = 0U;
volatile uint32_t g_can_phy_test_rx_count = 0U;
volatile uint32_t g_can_phy_test_first_counter = 0U;
volatile uint32_t g_can_phy_test_last_counter = 0U;
volatile uint32_t g_can_phy_test_missing_count = 0U;
volatile uint32_t g_can_phy_test_duplicate_count = 0U;
volatile uint32_t g_can_phy_test_invalid_count = 0U;
volatile uint32_t g_can_phy_test_tx_request_count = 0U;
volatile uint32_t g_can_phy_test_tx_deferred_count = 0U;
#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
volatile uint32_t g_zdt_motor_can_position_tx_count = 0U;
volatile uint32_t g_zdt_motor_can_position_rx_count = 0U;
volatile uint32_t g_zdt_motor_can_invalid_rx_count = 0U;
volatile uint32_t g_zdt_motor_can_position_negative = 0U;
volatile uint32_t g_zdt_motor_can_position_raw = 0U;
volatile uint32_t g_zdt_motor_can_position_received_ms = 0U;
#endif
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
volatile uint32_t g_window_can_test_overflow_injection_mode = 0U;
volatile uint32_t g_window_can_test_last_command_sequence = 0U;
volatile uint32_t g_window_can_test_last_command_type = 0U;
volatile uint32_t g_window_can_test_last_command_received_ms = 0U;
volatile uint32_t g_window_can_test_last_command_processed_ms = 0U;
volatile uint32_t g_window_can_test_last_command_recovery_entry_ms = 0U;
volatile uint32_t g_window_can_test_last_command_result = 0U;
volatile uint32_t g_window_can_test_motion_sequence = 0U;
volatile uint32_t g_window_can_test_motion_received_ms = 0U;
volatile uint32_t g_window_can_test_motion_processed_ms = 0U;
volatile uint32_t g_window_can_test_motor_restart_request = 0U;
volatile uint32_t g_window_can_test_motor_restart_result = 0U;
static bool s_testOverflowInjected;
#endif

static window_state_t s_windowState;
static uint32_t s_lastSafetyCheckMs;
static uint32_t s_lastTestTxMs;
static uint32_t s_lastAcceptedCommandMs;
static uint32_t s_statusTestStartMs;
static uint32_t s_lastStatusTxMs;
static uint32_t s_lastPhysicalTestTxMs;
#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
static uint32_t s_lastZdtPositionQueryMs;
#endif
static bool s_overflowLatched;
static bool s_testUpWasAccepted;
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
static uint32_t s_handledMotorDeviceErrorCount;
static window_demo_t s_demo;
static uint32_t s_motorRestartLastPositionRxMs;
static uint32_t s_motorRestartFirstStableMs;
static int32_t s_motorRestartCandidatePositionTenths;
#endif
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
static bool s_statusFramePending;
static can_protocol_frame_t s_expectedStatusFrame;
#endif

static const can_protocol_command_t s_loopbackCommands[
    WINDOW_CAN_APP_TEST_COMMAND_COUNT] = {
    {0U, CAN_PROTOCOL_CMD_STOP},
    {1U, CAN_PROTOCOL_CMD_CLEAR_FAULT},
    {2U, CAN_PROTOCOL_CMD_STOP},
    {3U, CAN_PROTOCOL_CMD_UP}
};

static const window_state_rx_result_t s_expectedRxResults[
    WINDOW_CAN_APP_TEST_COMMAND_COUNT] = {
    WINDOW_STATE_RX_BASELINE_ACCEPTED,
    WINDOW_STATE_RX_CLEAR_SUCCEEDED,
    WINDOW_STATE_RX_ACCEPTED,
    WINDOW_STATE_RX_ACCEPTED
};

void SysTick_Handler(void)
{
    g_window_can_uptime_ms++;
#if defined(DUALECU_ENABLE_MOTOR_STEP_TEST)
    motor_port_mspm0_tick_1ms();
#endif
}

static uint32_t windowCanAppNowMs(void)
{
    return g_window_can_uptime_ms;
}

static bool windowCanAppGetStatus(can_protocol_status_t *status)
{
    if (!window_state_get_status(&s_windowState, status)) { return false; }
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    zdt_uart_mspm0_apply_range_status(status);
#endif
    return true;
}

static void windowCanAppPublishState(void)
{
    can_protocol_status_t status = {0};

    if (windowCanAppGetStatus(&status))
    {
        g_window_can_state = (uint32_t)status.state;
        g_window_can_fault = (uint32_t)status.fault;
        g_window_can_last_sequence = status.last_sequence;
        g_window_can_last_sequence_valid = status.last_sequence_valid ? 1U : 0U;
        g_window_can_rx_overflow_active = status.rx_overflow_active ? 1U : 0U;
        g_window_can_overflow_observations =
            window_state_get_rx_overflow_observation_count(&s_windowState);
    }
}

static void windowCanAppFail(uint32_t reason)
{
    if (g_window_can_test_result != WINDOW_CAN_APP_TEST_FAIL)
    {
        g_window_can_test_failure = reason;
        g_window_can_test_result = WINDOW_CAN_APP_TEST_FAIL;
    }
}

static void windowCanAppObserveOverflow(uint32_t nowMs)
{
    if (can_port_mspm0_rx_overflow_pending() && !s_overflowLatched)
    {
        window_state_note_rx_overflow(&s_windowState, nowMs);
        s_overflowLatched = true;
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
        (void)zdt_uart_mspm0_request_stop();
        window_demo_cancel(&s_demo);
        g_window_can_demo_active = 0U;
#endif
    }
}

#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
static void windowCanAppRunTestOverflowInjection(void)
{
    if ((g_window_can_test_overflow_injection_mode == 1U) &&
        !s_testOverflowInjected)
    {
        window_state_note_rx_overflow(
            &s_windowState, windowCanAppNowMs());
        s_testOverflowInjected = true;
        windowCanAppPublishState();
    }
    else if ((g_window_can_test_overflow_injection_mode == 2U) &&
             s_testOverflowInjected &&
             can_port_mspm0_clear_rx_overflow_if_drained())
    {
        window_state_confirm_rx_overflow_cleared(&s_windowState);
        s_testOverflowInjected = false;
        g_window_can_test_overflow_injection_mode = 0U;
        windowCanAppPublishState();
    }
}
#endif

static bool windowCanAppIsAcceptedResult(window_state_rx_result_t result)
{
    return (result == WINDOW_STATE_RX_BASELINE_ACCEPTED) ||
           (result == WINDOW_STATE_RX_REPEAT_ACCEPTED) ||
           (result == WINDOW_STATE_RX_ACCEPTED) ||
           (result == WINDOW_STATE_RX_CLEAR_FAILED) ||
           (result == WINDOW_STATE_RX_CLEAR_SUCCEEDED);
}

static void windowCanAppProcessRxFrame(void)
{
    can_protocol_frame_t frame = {0};
    can_port_mspm0_rx_result_t portResult;
    window_state_rx_result_t stateResult;
    uint16_t frameTimestamp = 0U;
    uint16_t timestampNow;
    uint32_t processedAtMs;
    uint32_t receivedAtMs;
    can_protocol_status_t status = {0};
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    can_protocol_command_t testCommand = {0};
#endif
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    can_protocol_command_t controlCommand = {0};
    bool controlCommandValid = false;
    bool wasMoving = false;
#endif

    portResult = can_port_mspm0_receive(&frame, &frameTimestamp);
    if (portResult == CAN_PORT_MSPM0_RX_ACK_ERROR)
    {
        windowCanAppFail(WINDOW_CAN_APP_FAIL_RX_ACK);
        return;
    }
    if (portResult != CAN_PORT_MSPM0_RX_FRAME)
    {
        return;
    }

    g_window_can_rx_frame_count++;
    processedAtMs = windowCanAppNowMs();
    if (frame.is_extended)
    {
#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
        zdt_motor_position_t motorPosition = {0};

        timestampNow = can_port_mspm0_timestamp_now();
        receivedAtMs = mcan_timestamp_frame_received_at_ms(
            processedAtMs, timestampNow, frameTimestamp);
        if (zdt_motor_can_decode_position_response(
                ZDT_MOTOR_CAN_DEFAULT_ADDRESS, &frame, &motorPosition))
        {
            g_zdt_motor_can_position_negative =
                motorPosition.negative ? 1U : 0U;
            g_zdt_motor_can_position_raw = motorPosition.raw_position;
            g_zdt_motor_can_position_received_ms = receivedAtMs;
            g_zdt_motor_can_position_rx_count++;
        }
        else
        {
            g_zdt_motor_can_invalid_rx_count++;
        }
#endif
        return; /* Extended vendor frames cannot refresh the window watchdog. */
    }

    if (frame.id == CAN_TEST_FRAME_F407_TO_G3507_ID)
    {
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
        can_test_t04_action_t t04Action;

        if (can_test_frame_decode_t04_control(&frame, &t04Action))
        {
            if ((t04Action == CAN_TEST_T04_ACTION_INJECT_OVERFLOW) &&
                !s_testOverflowInjected &&
                (g_window_can_test_overflow_injection_mode == 0U))
            {
                g_window_can_test_overflow_injection_mode = 1U;
            }
            else if ((t04Action == CAN_TEST_T04_ACTION_RELEASE_OVERFLOW) &&
                     s_testOverflowInjected &&
                     (g_window_can_test_overflow_injection_mode == 1U))
            {
                g_window_can_test_overflow_injection_mode = 2U;
            }
            return; /* Test control frames never refresh the command watchdog. */
        }
#endif
        uint32_t counter = 0U;
        if (!can_test_frame_decode(
                &frame, CAN_TEST_FRAME_F407_TO_G3507_ID, &counter))
        {
            g_can_phy_test_invalid_count++;
        }
        else if (g_can_phy_test_rx_count == 0U)
        {
            g_can_phy_test_first_counter = counter;
            g_can_phy_test_last_counter = counter;
            g_can_phy_test_missing_count = counter - 1U;
            g_can_phy_test_rx_count++;
        }
        else if (counter <= g_can_phy_test_last_counter)
        {
            g_can_phy_test_duplicate_count++;
        }
        else
        {
            g_can_phy_test_missing_count +=
                counter - g_can_phy_test_last_counter - 1U;
            g_can_phy_test_last_counter = counter;
            g_can_phy_test_rx_count++;
        }
        return; /* A link probe must not touch command age or motion state. */
    }
    if (frame.id == CAN_PROTOCOL_STATUS_ID)
    {
        if (!can_protocol_decode_status(&frame, &status))
        {
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
            g_window_can_status_test_failure =
                WINDOW_CAN_APP_STATUS_FAIL_INVALID_FRAME;
            g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_FAIL;
#endif
            return;
        }

        g_window_can_status_last_sequence = status.last_sequence;
        g_window_can_status_last_state = (uint32_t)status.state;
        g_window_can_status_last_fault = (uint32_t)status.fault;
        g_window_can_status_last_position = status.position;
        g_window_can_status_last_flags =
            (status.is_calibrated ? 1U : 0U) |
            (status.last_sequence_valid ? 2U : 0U) |
            (status.rx_overflow_active ? 4U : 0U);
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
        if (!s_statusFramePending ||
            (frame.id != s_expectedStatusFrame.id) ||
            (frame.dlc != s_expectedStatusFrame.dlc) ||
            frame.is_extended || frame.is_remote)
        {
            g_window_can_status_test_failure =
                WINDOW_CAN_APP_STATUS_FAIL_INVALID_FRAME;
            g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_FAIL;
            return;
        }

        for (uint32_t byteIndex = 0U; byteIndex < frame.dlc; byteIndex++)
        {
            if (frame.data[byteIndex] != s_expectedStatusFrame.data[byteIndex])
            {
                g_window_can_status_test_failure =
                    WINDOW_CAN_APP_STATUS_FAIL_INVALID_FRAME;
                g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_FAIL;
                return;
            }
        }

        s_statusFramePending = false;
        g_window_can_status_rx_count++;
#endif
        return;
    }

    timestampNow = can_port_mspm0_timestamp_now();
    g_window_can_rx_timestamp = frameTimestamp;
    g_window_can_timestamp_now = timestampNow;
    g_window_can_timestamp_age_ticks =
        (uint16_t)(timestampNow - frameTimestamp);
    receivedAtMs = mcan_timestamp_frame_received_at_ms(
        processedAtMs, timestampNow, frameTimestamp);
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    controlCommandValid = can_protocol_decode_command(&frame, &controlCommand);
#endif
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    if ((frame.id == CAN_PROTOCOL_COMMAND_ID) &&
        can_protocol_decode_command(&frame, &testCommand))
    {
        g_window_can_test_last_command_sequence = testCommand.sequence;
        g_window_can_test_last_command_type =
            (uint32_t)testCommand.command;
        g_window_can_test_last_command_received_ms = receivedAtMs;
        g_window_can_test_last_command_processed_ms = processedAtMs;
        g_window_can_test_last_command_recovery_entry_ms =
            s_windowState.recovery_entry_ms;
    }
#endif
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    wasMoving = (s_windowState.state != CAN_PROTOCOL_STATE_STOP);
#endif
    stateResult = window_state_receive_frame_at(
        &s_windowState, &frame, receivedAtMs, processedAtMs);
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    if (s_demo.active &&
        ((s_windowState.state == CAN_PROTOCOL_STATE_STOP) ||
         (s_windowState.last_command != CAN_PROTOCOL_CMD_DEMO_TOGGLE)))
    {
        window_demo_cancel(&s_demo);
        g_window_can_demo_active = 0U;
    }
    if (wasMoving && (s_windowState.state == CAN_PROTOCOL_STATE_STOP))
    {
        const bool isNewCanStop = controlCommandValid &&
            (controlCommand.command == CAN_PROTOCOL_CMD_STOP) &&
            ((stateResult == WINDOW_STATE_RX_BASELINE_ACCEPTED) ||
             (stateResult == WINDOW_STATE_RX_ACCEPTED));

        if (!isNewCanStop)
        {
            (void)zdt_uart_mspm0_request_stop();
        }
    }
#endif
    g_window_can_last_rx_result = (uint32_t)stateResult;
    g_window_can_last_rx_ms = receivedAtMs;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    if (frame.id == CAN_PROTOCOL_COMMAND_ID)
    {
        g_window_can_test_last_command_result = (uint32_t)stateResult;
    }
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    if (controlCommandValid &&
        (stateResult == WINDOW_STATE_RX_ACCEPTED) &&
        ((controlCommand.command == CAN_PROTOCOL_CMD_UP) ||
         (controlCommand.command == CAN_PROTOCOL_CMD_DOWN)))
    {
        g_window_can_test_motion_sequence = controlCommand.sequence;
        g_window_can_test_motion_received_ms = receivedAtMs;
        g_window_can_test_motion_processed_ms = processedAtMs;
    }
#endif
#endif

    if (windowCanAppIsAcceptedResult(stateResult))
    {
        s_lastAcceptedCommandMs = receivedAtMs;
        g_window_can_last_accepted_ms = receivedAtMs;
    }
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    if (controlCommandValid &&
        ((controlCommand.command == CAN_PROTOCOL_CMD_UP) ||
         (controlCommand.command == CAN_PROTOCOL_CMD_DOWN)) &&
        windowCanAppIsAcceptedResult(stateResult))
    {
        can_protocol_status_t outputStatus = {0};

        if (windowCanAppGetStatus(&outputStatus))
        {
            const zdt_uart_window_output_t output =
                zdt_uart_select_window_output(
                    &outputStatus,
                    zdt_uart_mspm0_motion_ready(processedAtMs),
                    g_zdt_uart_rx_overflow_count != 0U);

            if (output == ZDT_UART_WINDOW_OUTPUT_STOP)
            {
                if (outputStatus.state != CAN_PROTOCOL_STATE_STOP)
                {
                    window_state_note_local_output_stop(&s_windowState);
                    (void)zdt_uart_mspm0_request_stop();
                }
            }
            else if (stateResult == WINDOW_STATE_RX_ACCEPTED)
            {
                (void)zdt_uart_mspm0_request_window_output(
                    output, processedAtMs);
            }
        }
        else
        {
            (void)zdt_uart_mspm0_request_stop();
        }
    }
    if (controlCommandValid &&
        (controlCommand.command == CAN_PROTOCOL_CMD_STOP) &&
        ((stateResult == WINDOW_STATE_RX_BASELINE_ACCEPTED) ||
         (stateResult == WINDOW_STATE_RX_ACCEPTED)))
    {
        (void)zdt_uart_mspm0_request_stop();
    }
    if (controlCommandValid &&
        (stateResult == WINDOW_STATE_RX_ACCEPTED) &&
        ((controlCommand.command == CAN_PROTOCOL_CMD_DEMO_SET_ZERO) ||
         (controlCommand.command == CAN_PROTOCOL_CMD_DEMO_TOGGLE)))
    {
        if (wasMoving || s_demo.active ||
            (s_windowState.fault != CAN_PROTOCOL_FAULT_NONE) ||
            (s_windowState.recovery_phase != WINDOW_RECOVERY_READY))
        {
            ++g_window_can_demo_rejected_count;
        }
        else if (controlCommand.command == CAN_PROTOCOL_CMD_DEMO_SET_ZERO)
        {
            if (zdt_uart_mspm0_set_demo_range(processedAtMs))
            {
                ++g_window_can_demo_zero_count;
            }
            else
            {
                ++g_window_can_demo_rejected_count;
            }
        }
        else
        {
            motor_range_t range;
            if (zdt_uart_mspm0_motion_ready(processedAtMs) &&
                zdt_uart_mspm0_get_range(&range) &&
                window_demo_begin(&s_demo, &range,
                                  g_zdt_uart_position_tenths, processedAtMs))
            {
                g_window_can_demo_active = 1U;
            }
            else
            {
                ++g_window_can_demo_rejected_count;
            }
        }
    }
#endif

#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
    if (g_window_can_test_result == WINDOW_CAN_APP_TEST_RUNNING)
    {
        const uint32_t expectedIndex = g_window_can_test_rx_count;

        if ((expectedIndex >= g_window_can_test_tx_count) ||
            (expectedIndex >= WINDOW_CAN_APP_TEST_COMMAND_COUNT) ||
            (stateResult != s_expectedRxResults[expectedIndex]))
        {
            windowCanAppFail(WINDOW_CAN_APP_FAIL_COMMAND_SEQUENCE);
        }
        else
        {
            g_window_can_test_rx_count++;
            (void)windowCanAppGetStatus(&status);
            if ((expectedIndex == 3U) &&
                (status.state == CAN_PROTOCOL_STATE_UP) &&
                (status.fault == CAN_PROTOCOL_FAULT_NONE))
            {
                s_testUpWasAccepted = true;
            }
            else if ((expectedIndex == 3U) && !s_testUpWasAccepted)
            {
                windowCanAppFail(WINDOW_CAN_APP_FAIL_COMMAND_SEQUENCE);
            }
        }
    }
#endif

    windowCanAppPublishState();
}

static void windowCanAppCheckStatusSelfTest(uint32_t nowMs)
{
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
    if (g_window_can_status_test_result != WINDOW_CAN_APP_STATUS_TEST_RUNNING)
    {
        return;
    }

    if ((g_window_can_status_rx_count >= WINDOW_CAN_APP_STATUS_TEST_MIN_RX) &&
        !s_statusFramePending &&
        (g_window_can_status_tx_count >= WINDOW_CAN_APP_STATUS_TEST_MIN_RX) &&
        (g_window_can_status_last_state == CAN_PROTOCOL_STATE_STOP) &&
        (g_window_can_status_last_fault == CAN_PROTOCOL_FAULT_COMM_TIMEOUT) &&
        (g_window_can_status_last_sequence == 3U) &&
        (g_window_can_status_last_position == 0xFFFFU) &&
        (g_window_can_status_last_flags == 0U) &&
        (g_window_can_status_min_interval_ms >=
         WINDOW_CAN_APP_STATUS_PERIOD_MS) &&
        (g_window_can_status_max_interval_ms <=
         WINDOW_CAN_APP_STATUS_CADENCE_MAX_MS))
    {
        g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_PASS;
        g_window_can_status_test_complete_ms = nowMs;
        return;
    }

    if ((uint32_t)(nowMs - s_statusTestStartMs) >
        WINDOW_CAN_APP_STATUS_TEST_DEADLINE_MS)
    {
        if (g_window_can_status_rx_count < WINDOW_CAN_APP_STATUS_TEST_MIN_RX)
        {
            g_window_can_status_test_failure =
                WINDOW_CAN_APP_STATUS_FAIL_MISSING_FRAMES;
        }
        else if (s_statusFramePending)
        {
            g_window_can_status_test_failure =
                WINDOW_CAN_APP_STATUS_FAIL_LOOPBACK_PENDING;
        }
        else if ((g_window_can_status_min_interval_ms <
                  WINDOW_CAN_APP_STATUS_PERIOD_MS) ||
                 (g_window_can_status_max_interval_ms >
                  WINDOW_CAN_APP_STATUS_CADENCE_MAX_MS))
        {
            g_window_can_status_test_failure = WINDOW_CAN_APP_STATUS_FAIL_CADENCE;
        }
        else
        {
            g_window_can_status_test_failure =
                WINDOW_CAN_APP_STATUS_FAIL_FINAL_STATE;
        }
        g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_FAIL;
    }
#else
    (void)nowMs;
#endif
}

static void windowCanAppRunStatusTransmit(uint32_t nowMs)
{
    can_protocol_status_t status = {0};
    can_protocol_frame_t frame = {0};
    const uint32_t elapsedMs = (uint32_t)(nowMs - s_lastStatusTxMs);

    if (elapsedMs < WINDOW_CAN_APP_STATUS_PERIOD_MS)
    {
        return;
    }

#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
    if (s_statusFramePending)
    {
        g_window_can_status_test_failure =
            WINDOW_CAN_APP_STATUS_FAIL_LOOPBACK_PENDING;
        g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_FAIL;
        return;
    }
#endif

    if (!windowCanAppGetStatus(&status) ||
        !can_protocol_encode_status(&status, &frame) ||
        !can_port_mspm0_send_frame(&frame))
    {
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
        g_window_can_status_test_failure = WINDOW_CAN_APP_STATUS_FAIL_TX;
        g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_FAIL;
#endif
        return;
    }

    if (g_window_can_status_tx_count == 0U)
    {
        g_window_can_status_first_tx_ms = nowMs;
    }
    else
    {
        const uint32_t intervalMs =
            (uint32_t)(nowMs - g_window_can_status_last_tx_ms);

        if ((g_window_can_status_tx_count == 1U) ||
            (intervalMs < g_window_can_status_min_interval_ms))
        {
            g_window_can_status_min_interval_ms = intervalMs;
        }
        if (intervalMs > g_window_can_status_max_interval_ms)
        {
            g_window_can_status_max_interval_ms = intervalMs;
        }
    }

    g_window_can_status_tx_count++;
    g_window_can_status_last_tx_ms = nowMs;
    s_lastStatusTxMs = nowMs;
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
    s_expectedStatusFrame = frame;
    s_statusFramePending = true;
#endif
}

static void windowCanAppRunPhysicalTestTransmit(uint32_t nowMs)
{
    can_protocol_frame_t frame = {0};

    /* Start the reverse direction only after the forward RX run is complete. */
    if ((g_can_phy_test_rx_count < CAN_TEST_FRAME_COUNT) ||
        (g_can_phy_test_tx_request_count >= CAN_TEST_FRAME_COUNT) ||
        ((uint32_t)(nowMs - s_lastPhysicalTestTxMs) <
         WINDOW_CAN_APP_PHYSICAL_TEST_PERIOD_MS))
    {
        return;
    }

    can_test_frame_encode(CAN_TEST_FRAME_G3507_TO_F407_ID,
                          g_can_phy_test_tx_request_count + 1U, &frame);
    if (can_port_mspm0_send_frame(&frame))
    {
        g_can_phy_test_tx_request_count++;
        s_lastPhysicalTestTxMs = nowMs;
    }
    else
    {
        g_can_phy_test_tx_deferred_count++;
    }
}

#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
static void windowCanAppRunZdtPositionQuery(uint32_t nowMs)
{
    can_protocol_frame_t frame = {0};

    if ((uint32_t)(nowMs - s_lastZdtPositionQueryMs) <
        WINDOW_CAN_APP_ZDT_POSITION_PERIOD_MS)
    {
        return;
    }

    s_lastZdtPositionQueryMs = nowMs;
    if (!zdt_motor_can_encode_read_position(
            ZDT_MOTOR_CAN_DEFAULT_ADDRESS, &frame) ||
        !can_port_mspm0_send_frame(&frame))
    {
        return;
    }

    g_zdt_motor_can_position_tx_count++;
}
#endif

static void windowCanAppProcessRx(uint32_t nowMs)
{
    uint32_t processedCount;

    windowCanAppObserveOverflow(nowMs);

    for (processedCount = 0U;
         processedCount < WINDOW_CAN_APP_RX_BUDGET;
         processedCount++)
    {
        const uint32_t beforeCount = g_window_can_rx_frame_count;

        windowCanAppProcessRxFrame();
        if ((beforeCount == g_window_can_rx_frame_count) ||
            (g_window_can_test_result == WINDOW_CAN_APP_TEST_FAIL))
        {
            break;
        }
    }

    if (s_overflowLatched && can_port_mspm0_clear_rx_overflow_if_drained())
    {
        window_state_confirm_rx_overflow_cleared(&s_windowState);
        s_overflowLatched = false;
        windowCanAppPublishState();
    }
}

static void windowCanAppRunLoopbackTest(uint32_t nowMs)
{
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
    if (g_window_can_test_result == WINDOW_CAN_APP_TEST_RUNNING)
    {
        const uint32_t txCount = g_window_can_test_tx_count;

        if ((txCount < WINDOW_CAN_APP_TEST_COMMAND_COUNT) &&
            (g_window_can_test_rx_count == txCount) &&
            ((txCount == 0U && nowMs >= WINDOW_CAN_APP_COMMAND_PERIOD_MS) ||
             (txCount > 0U &&
              (uint32_t)(nowMs - s_lastTestTxMs) >=
                  WINDOW_CAN_APP_COMMAND_PERIOD_MS)))
        {
            can_protocol_frame_t frame = {0};

            if (!can_protocol_encode_command(&s_loopbackCommands[txCount], &frame) ||
                !can_port_mspm0_send_frame(&frame))
            {
                windowCanAppFail(WINDOW_CAN_APP_FAIL_TX);
            }
            else
            {
                g_window_can_test_tx_count++;
                s_lastTestTxMs = nowMs;
            }
        }

        if ((g_window_can_test_tx_count > g_window_can_test_rx_count) &&
            ((uint32_t)(nowMs - s_lastTestTxMs) >
             WINDOW_CAN_APP_RX_DEADLINE_MS))
        {
            windowCanAppFail(WINDOW_CAN_APP_FAIL_RX_TIMEOUT);
        }

        if ((g_window_can_test_rx_count == WINDOW_CAN_APP_TEST_COMMAND_COUNT) &&
            s_testUpWasAccepted)
        {
            can_protocol_status_t status = {0};
            const uint32_t silenceMs = (uint32_t)(nowMs - s_lastAcceptedCommandMs);

            (void)windowCanAppGetStatus(&status);
            if ((status.state == CAN_PROTOCOL_STATE_STOP) &&
                (status.fault == CAN_PROTOCOL_FAULT_COMM_TIMEOUT) &&
                !status.last_sequence_valid)
            {
                g_window_can_test_timeout_age_ms = silenceMs;
                g_window_can_test_complete_ms = nowMs;
                if ((silenceMs > WINDOW_STATE_COMMAND_TIMEOUT_MS) &&
                    (silenceMs <= WINDOW_CAN_APP_TEST_TIMEOUT_LIMIT_MS))
                {
                    g_window_can_test_result = WINDOW_CAN_APP_TEST_PASS;
                }
                else
                {
                    windowCanAppFail(WINDOW_CAN_APP_FAIL_TIMEOUT);
                }
            }
            else if (silenceMs > WINDOW_CAN_APP_TEST_TIMEOUT_LIMIT_MS)
            {
                windowCanAppFail(WINDOW_CAN_APP_FAIL_TIMEOUT);
            }
        }
    }
#else
    (void)nowMs;
#endif
}

#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
static void windowCanAppServiceDemo(uint32_t nowMs, bool motorReady)
{
    motor_range_t range = {0};
    window_demo_step_t step = WINDOW_DEMO_STEP_IDLE;
    window_demo_action_t action;
    zdt_uart_window_output_t output;

    if (!s_demo.active)
    {
        return;
    }
    if ((s_windowState.fault != CAN_PROTOCOL_FAULT_NONE) ||
        (s_windowState.recovery_phase != WINDOW_RECOVERY_READY) ||
        (s_windowState.last_command != CAN_PROTOCOL_CMD_DEMO_TOGGLE) ||
        !zdt_uart_mspm0_get_range(&range))
    {
        window_demo_cancel(&s_demo);
        g_window_can_demo_active = 0U;
        (void)zdt_uart_mspm0_request_stop();
        return;
    }
    if (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_RUNNING)
    {
        step = WINDOW_DEMO_STEP_RUNNING;
    }
    else if (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_POSITION_CHANGED)
    {
        step = WINDOW_DEMO_STEP_COMPLETE;
    }
    else if ((g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_REJECTED) ||
             (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_TIMEOUT_STOPPED) ||
             (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_RX_OVERFLOW_STOPPED) ||
             (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_FEEDBACK_STOPPED))
    {
        step = WINDOW_DEMO_STEP_FAILED;
    }
    action = window_demo_update(&s_demo, &range,
                                g_zdt_uart_position_tenths, step,
                                motorReady, nowMs);
    g_window_can_demo_active = s_demo.active ? 1U : 0U;
    if (action == WINDOW_DEMO_ACTION_NONE)
    {
        return;
    }
    if (action == WINDOW_DEMO_ACTION_COMPLETE)
    {
        ++g_window_can_demo_complete_count;
        window_state_note_local_output_stop(&s_windowState);
        windowCanAppPublishState();
        return;
    }
    if ((action == WINDOW_DEMO_ACTION_STEP_UP) ||
        (action == WINDOW_DEMO_ACTION_STEP_DOWN))
    {
        output = (action == WINDOW_DEMO_ACTION_STEP_UP) ?
                     ZDT_UART_WINDOW_OUTPUT_UP : ZDT_UART_WINDOW_OUTPUT_DOWN;
        if (zdt_uart_mspm0_request_window_output_at_speed(
                output, nowMs, s_demo.speed_tenths_rpm) &&
            window_state_note_demo_output(&s_windowState,
                (action == WINDOW_DEMO_ACTION_STEP_UP) ?
                    CAN_PROTOCOL_STATE_UP : CAN_PROTOCOL_STATE_DOWN))
        {
            ++g_window_can_demo_step_count;
            windowCanAppPublishState();
            return;
        }
    }
    window_demo_cancel(&s_demo);
    g_window_can_demo_active = 0U;
    ++g_window_can_demo_fault_count;
    window_state_note_motor_fault(&s_windowState, nowMs);
    zdt_uart_mspm0_invalidate_range();
    (void)zdt_uart_mspm0_request_stop();
    windowCanAppPublishState();
}

static void windowCanAppServiceMotorRestartRezero(
    uint32_t nowMs, bool motorReady)
{
    const uint8_t stationaryMask =
        (uint8_t)(ZDT_UART_STATUS_ENABLED | ZDT_UART_STATUS_POSITION_REACHED);
    const uint32_t positionRxMs = g_zdt_uart_position_rx_ms;
    const int32_t positionTenths = g_zdt_uart_position_tenths;

    if (g_window_can_motor_restart_rezero_pending == 0U)
    {
        return;
    }
    if (!motorReady || (s_windowState.state != CAN_PROTOCOL_STATE_STOP) ||
        (g_window_can_demo_active != 0U) ||
        ((g_zdt_uart_status_flags & stationaryMask) != stationaryMask))
    {
        s_motorRestartLastPositionRxMs = 0U;
        g_window_can_motor_restart_rezero_samples = 0U;
        return;
    }
    if ((positionRxMs == 0U) || (positionRxMs == s_motorRestartLastPositionRxMs))
    {
        return;
    }
    s_motorRestartLastPositionRxMs = positionRxMs;

    if ((g_window_can_motor_restart_rezero_samples == 0U) ||
        (positionTenths != s_motorRestartCandidatePositionTenths))
    {
        s_motorRestartCandidatePositionTenths = positionTenths;
        s_motorRestartFirstStableMs = positionRxMs;
        g_window_can_motor_restart_rezero_samples = 1U;
        return;
    }
    if (g_window_can_motor_restart_rezero_samples < UINT32_MAX)
    {
        ++g_window_can_motor_restart_rezero_samples;
    }

    if ((g_window_can_motor_restart_rezero_samples < 3U) ||
        ((uint32_t)(positionRxMs - s_motorRestartFirstStableMs) < 200U) ||
        !window_state_can_rezero_after_motor_recovery(
            &s_windowState, true, motorReady, true))
    {
        return;
    }

    if (zdt_uart_mspm0_set_demo_range(nowMs))
    {
        g_window_can_motor_restart_rezero_pending = 0U;
        ++g_window_can_motor_restart_rezero_count;
        ++g_window_can_demo_zero_count;
        windowCanAppPublishState();
    }
}
#endif

bool window_can_app_init(void)
{
    const uint32_t systickReload = CPUCLK_FREQ / 1000U;

    if ((systickReload == 0U) || (SysTick_Config(systickReload) != 0U))
    {
        g_window_can_init_result = WINDOW_CAN_APP_INIT_FAIL_SYSTICK;
        return false;
    }

#if defined(DUALECU_ENABLE_MOTOR_STEP_TEST)
    motor_port_mspm0_init();
#endif

    /* Normal application startup remains in its existing locked STOP state. */
    window_state_init(&s_windowState);
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    s_handledMotorDeviceErrorCount = 0U;
    s_motorRestartLastPositionRxMs = 0U;
    s_motorRestartFirstStableMs = 0U;
    s_motorRestartCandidatePositionTenths = 0;
    g_window_can_motor_restart_rezero_pending = 0U;
    g_window_can_motor_restart_rezero_samples = 0U;
    g_window_can_motor_restart_rezero_count = 0U;
    window_demo_cancel(&s_demo);
    g_window_can_demo_active = 0U;
#endif
    if (!can_port_mspm0_init())
    {
        g_window_can_init_result = WINDOW_CAN_APP_INIT_FAIL_MCAN;
        return false;
    }

#if defined(DUALECU_ENABLE_ZDT_UART_BENCH_TEST) || \
    defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    if (!zdt_uart_mspm0_init(windowCanAppNowMs()))
    {
        g_window_can_init_result = WINDOW_CAN_APP_INIT_FAIL_ZDT_UART;
        return false;
    }
#endif

    s_lastSafetyCheckMs = windowCanAppNowMs();
    s_statusTestStartMs = s_lastSafetyCheckMs;
    s_lastStatusTxMs = s_lastSafetyCheckMs;
    s_lastPhysicalTestTxMs = s_lastSafetyCheckMs;
#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
    s_lastZdtPositionQueryMs = s_lastSafetyCheckMs;
#endif
#if defined(APP_INTERNAL_LOOPBACK_SELF_TEST)
    g_window_can_test_result = WINDOW_CAN_APP_TEST_RUNNING;
    g_window_can_status_test_result = WINDOW_CAN_APP_STATUS_TEST_RUNNING;
#endif
    g_window_can_init_result = WINDOW_CAN_APP_INIT_OK;
    windowCanAppPublishState();
    return true;
}

void window_can_app_process(void)
{
    const uint32_t nowMs = windowCanAppNowMs();

    if (g_window_can_init_result != WINDOW_CAN_APP_INIT_OK)
    {
        return;
    }

    if ((uint32_t)(nowMs - s_lastSafetyCheckMs) >=
        WINDOW_CAN_APP_SAFETY_PERIOD_MS)
    {
        const bool wasMoving = s_windowState.state != CAN_PROTOCOL_STATE_STOP;
        s_lastSafetyCheckMs = nowMs;
        window_state_tick(&s_windowState, nowMs);
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
        if (wasMoving &&
            (s_windowState.state == CAN_PROTOCOL_STATE_STOP))
        {
            window_demo_cancel(&s_demo);
            g_window_can_demo_active = 0U;
            (void)zdt_uart_mspm0_request_stop();
        }
#else
        (void)wasMoving;
#endif
        windowCanAppPublishState();
    }

#if defined(DUALECU_ENABLE_ZDT_UART_BENCH_TEST) || \
    defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    zdt_uart_mspm0_process(nowMs);
#endif
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    const bool motorReady = zdt_uart_mspm0_motion_ready(nowMs);
    can_protocol_status_t rangeStatus = {0};
    uint8_t motorDeviceErrorCode = 0U;
    const bool newMotorDeviceError = zdt_uart_device_error_event_take(
        (uint32_t)g_zdt_uart_device_error_count,
        (uint8_t)g_zdt_uart_last_device_error,
        &s_handledMotorDeviceErrorCount,
        &motorDeviceErrorCode);
    zdt_uart_mspm0_apply_range_status(&rangeStatus);
    const bool wasMotorMoving = s_windowState.state != CAN_PROTOCOL_STATE_STOP;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    if (g_window_can_test_motor_restart_request != 0U)
    {
        g_window_can_test_motor_restart_request = 0U;
        if ((s_windowState.state == CAN_PROTOCOL_STATE_STOP) &&
            (s_windowState.fault == CAN_PROTOCOL_FAULT_NONE) &&
            (s_windowState.recovery_phase == WINDOW_RECOVERY_READY) &&
            (g_window_can_demo_active == 0U) &&
            (g_zdt_uart_range_valid != 0U) && motorReady &&
            zdt_uart_mspm0_test_restart_controller(nowMs))
        {
            g_window_can_test_motor_restart_result = 1U;
        }
        else
        {
            g_window_can_test_motor_restart_result = 2U;
        }
    }
#endif
    const bool motorFailed = newMotorDeviceError || (wasMotorMoving &&
        ((g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_TIMEOUT_STOPPED) ||
         (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_RX_OVERFLOW_STOPPED) ||
         (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_FEEDBACK_STOPPED) ||
         (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_REJECTED)));
    if (motorFailed || ((g_zdt_uart_range_valid != 0U) &&
                       (!motorReady || !rangeStatus.is_calibrated)))
    {
        g_window_can_motor_fault_reason = newMotorDeviceError ? 6U :
            !motorReady ? 1U :
            !rangeStatus.is_calibrated ? 4U :
            (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_TIMEOUT_STOPPED) ? 2U :
            (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_RX_OVERFLOW_STOPPED) ? 3U : 5U;
        if (newMotorDeviceError)
        {
            g_window_can_motor_fault_code = motorDeviceErrorCode;
        }
        g_window_can_motor_fault_detected_ms = nowMs;
        window_state_note_motor_fault(&s_windowState, nowMs);
        if (g_zdt_uart_range_valid != 0U)
        {
            g_window_can_motor_restart_rezero_pending = 1U;
            g_window_can_motor_restart_rezero_samples = 0U;
            s_motorRestartLastPositionRxMs = 0U;
        }
        window_demo_cancel(&s_demo);
        g_window_can_demo_active = 0U;
        zdt_uart_mspm0_invalidate_range();
        (void)zdt_uart_mspm0_request_stop();
        windowCanAppPublishState();
    }
    else if (motorReady)
    {
        window_state_set_motor_fault_active(&s_windowState, false);
    }
    windowCanAppServiceMotorRestartRezero(nowMs, motorReady);
    windowCanAppServiceDemo(nowMs, motorReady);
    if (wasMotorMoving && !s_demo.active &&
        ((g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_POSITION_CHANGED) ||
         (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_STOP_SENT)))
    {
        window_state_note_local_output_stop(&s_windowState);
        windowCanAppPublishState();
    }
    if (g_zdt_uart_range_request != 0U)
    {
        const uint32_t request = g_zdt_uart_range_request;
        g_zdt_uart_range_request = 0U;
        if (request == 2U)
        {
            window_demo_cancel(&s_demo);
            g_window_can_demo_active = 0U;
            (void)zdt_uart_mspm0_request_stop();
            window_state_note_local_output_stop(&s_windowState);
            zdt_uart_mspm0_invalidate_range();
            g_zdt_uart_range_result = 3U;
        }
        else if ((request == 1U) &&
                 (s_windowState.state == CAN_PROTOCOL_STATE_STOP) &&
                 (s_windowState.fault == CAN_PROTOCOL_FAULT_NONE) &&
                 (s_windowState.recovery_phase == WINDOW_RECOVERY_READY) &&
                 zdt_uart_mspm0_set_bench_range(nowMs))
        {
            g_zdt_uart_range_result = 1U;
        }
        else { g_zdt_uart_range_result = 2U; }
    }
#endif

#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    windowCanAppRunTestOverflowInjection();
#endif
    windowCanAppProcessRx(nowMs);
    windowCanAppRunLoopbackTest(nowMs);
    windowCanAppRunStatusTransmit(nowMs);
    windowCanAppRunPhysicalTestTransmit(nowMs);
#if defined(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
    windowCanAppRunZdtPositionQuery(nowMs);
#endif
    windowCanAppCheckStatusSelfTest(nowMs);
}
