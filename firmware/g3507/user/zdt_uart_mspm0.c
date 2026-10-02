#if defined(DUALECU_ENABLE_ZDT_UART_BENCH_TEST) || \
    defined(DUALECU_ENABLE_ZDT_UART_CONTROL)

#include "zdt_uart_mspm0.h"

#include "ti_msp_dl_config.h"
#include "zdt_uart_protocol.h"
#include "motor_range.h"

#include <stddef.h>
#include <stdint.h>

#define ZDT_UART_RX_RING_SIZE          (64U)
#define ZDT_UART_RX_RING_MASK          (ZDT_UART_RX_RING_SIZE - 1U)
#define ZDT_UART_RX_TRACE_SIZE          (64U)
#define ZDT_UART_RX_TRACE_MASK          (ZDT_UART_RX_TRACE_SIZE - 1U)
#define ZDT_UART_DEVICE_ERROR_FRAME_LEN (4U)
#define ZDT_UART_RX_SERVICE_BUDGET     (32U)
#define ZDT_UART_INTERFRAME_DELAY_MS   (5U)
#define ZDT_UART_OPTIONS_RETRY_MS      (250U)
#define ZDT_UART_MOVE_TIMEOUT_MS       (2000U)
#define ZDT_UART_MOVE_DELTA_TENTHS     (5)
#define ZDT_UART_MOVE_MIN_DELTA_TENTHS (4)
#define ZDT_UART_MOVE_ACCEL_RPM_S      (1000U)
#define ZDT_UART_MOVE_DECEL_RPM_S      (1000U)
#define ZDT_UART_MOVE_SPEED_TENTHS_RPM (100U)
#define ZDT_UART_MOVE_CURRENT_MA       (500U)

volatile uint32_t g_zdt_uart_init_result = 0U;
volatile uint32_t g_zdt_uart_tx_frame_count = 0U;
volatile uint32_t g_zdt_uart_tx_byte_count = 0U;
volatile uint8_t g_zdt_uart_tx_trace[ZDT_UART_RX_TRACE_SIZE] = {0U};
volatile uint32_t g_zdt_uart_rx_byte_count = 0U;
volatile uint8_t g_zdt_uart_rx_trace[ZDT_UART_RX_TRACE_SIZE] = {0U};
volatile uint32_t g_zdt_uart_rx_frame_count = 0U;
volatile uint32_t g_zdt_uart_rx_rejected_count = 0U;
volatile uint32_t g_zdt_uart_rx_overflow_count = 0U;
volatile uint32_t g_zdt_uart_device_error_count = 0U;
volatile uint32_t g_zdt_uart_last_device_error = 0U;
volatile uint32_t g_zdt_uart_last_device_error_processed_ms = 0U;
volatile uint32_t g_zdt_uart_last_device_error_tx_frame_count = 0U;
volatile uint32_t g_zdt_uart_last_device_error_tx_frame_length = 0U;
volatile uint8_t g_zdt_uart_last_device_error_frame[ZDT_UART_DEVICE_ERROR_FRAME_LEN] = {0U};
volatile uint8_t g_zdt_uart_last_device_error_tx_frame[ZDT_UART_FRAME_MAX_LEN] = {0U};
volatile uint32_t g_zdt_uart_last_tx_frame_length = 0U;
volatile uint8_t g_zdt_uart_last_tx_frame[ZDT_UART_FRAME_MAX_LEN] = {0U};
volatile uint32_t g_zdt_uart_first_rx_byte_count = 0U;
volatile uint8_t g_zdt_uart_first_rx_bytes[8] = {0U};
volatile uint32_t g_zdt_uart_last_function = 0U;
volatile uint32_t g_zdt_uart_last_reply = 0U;
volatile uint32_t g_zdt_uart_option_flags = 0U;
volatile uint32_t g_zdt_uart_status_flags = 0U;
volatile uint32_t g_zdt_uart_bus_voltage_mv = 0U;
volatile uint32_t g_zdt_uart_position_raw = 0U;
volatile int32_t g_zdt_uart_position_tenths = 0;
volatile uint32_t g_zdt_uart_options_valid = 0U;
volatile uint32_t g_zdt_uart_status_valid = 0U;
volatile uint32_t g_zdt_uart_position_valid = 0U;
volatile uint32_t g_zdt_uart_bus_voltage_valid = 0U;
volatile uint32_t g_zdt_uart_last_frame_ms = 0U;
volatile uint32_t g_zdt_uart_position_rx_ms = 0U;
volatile uint32_t g_zdt_uart_last_status_ms = 0U;
volatile uint32_t g_zdt_uart_last_bus_voltage_ms = 0U;
volatile uint32_t g_zdt_uart_move_request = ZDT_UART_MOVE_REQUEST_NONE;
volatile uint32_t g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_IDLE;
volatile uint32_t g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NONE;
volatile int32_t g_zdt_uart_move_baseline_tenths = 0;
volatile int32_t g_zdt_uart_move_observed_delta_tenths = 0;
volatile uint32_t g_zdt_uart_move_complete_ms = 0U;
volatile uint32_t g_zdt_uart_range_valid = 0U;
volatile int32_t g_zdt_uart_range_min_tenths = 0;
volatile int32_t g_zdt_uart_range_max_tenths = 0;
volatile uint32_t g_zdt_uart_range_request = 0U;
volatile uint32_t g_zdt_uart_range_result = 0U;
static motor_range_t s_range;
static bool s_moveActive;
static int32_t s_targetDeltaTenths;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
/* 1: discard feedback; 2: arm discard after the next bounded motion TX. */
volatile uint32_t g_zdt_uart_test_feedback_pause = 0U;
/* 1: parse fresh replies but hold the app-visible position; 2: arm on next motion TX. */
volatile uint32_t g_zdt_uart_test_position_freeze = 0U;
/* Result of an X42S controller restart while safely stopped. */
volatile uint32_t g_zdt_uart_test_restart_result = 0U;
#endif
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
volatile uint32_t g_zdt_uart_test_first_position_query_ms = 0U;
volatile uint32_t g_zdt_uart_test_first_status_query_ms = 0U;
volatile uint32_t g_zdt_uart_test_first_position_rx_ms = 0U;
volatile uint32_t g_zdt_uart_test_first_status_rx_ms = 0U;
volatile uint32_t g_zdt_uart_test_first_position_progress_ms = 0U;
#endif

void zdt_uart_mspm0_invalidate_range(void)
{
    motor_range_invalidate(&s_range);
    g_zdt_uart_range_valid = 0U;
}

bool zdt_uart_mspm0_set_bench_range(uint32_t now_ms)
{
    const int32_t current = g_zdt_uart_position_tenths;
    /* Only the confirmed free-shaft bench: +/-5 degrees, never auto-homing. */
    if (!zdt_uart_mspm0_motion_ready(now_ms) ||
        (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_RUNNING) ||
        ((int64_t)current - 50 < INT32_MIN) ||
        ((int64_t)current + 50 > INT32_MAX) ||
        !motor_range_set(&s_range, current - 50, current + 50, current))
    {
        return false;
    }
    g_zdt_uart_range_min_tenths = s_range.minimum;
    g_zdt_uart_range_max_tenths = s_range.maximum;
    g_zdt_uart_range_valid = 1U;
    return true;
}

bool zdt_uart_mspm0_set_demo_range(uint32_t now_ms)
{
    const int32_t current = g_zdt_uart_position_tenths;
    /* A NEW explicit long press may rebase an existing stationary RAM range. */
    if (!zdt_uart_mspm0_motion_ready(now_ms) ||
        ((g_zdt_uart_status_flags & ZDT_UART_STATUS_POSITION_REACHED) == 0U) ||
        s_moveActive || (g_zdt_uart_move_request != ZDT_UART_MOVE_REQUEST_NONE) ||
        (g_zdt_uart_move_result == ZDT_UART_MOVE_RESULT_RUNNING) ||
        ((int64_t)current + 900 > INT32_MAX) ||
        !motor_range_set(&s_range, current, current + 900, current))
    {
        return false;
    }
    g_zdt_uart_range_min_tenths = s_range.minimum;
    g_zdt_uart_range_max_tenths = s_range.maximum;
    g_zdt_uart_range_valid = 1U;
    return true;
}

bool zdt_uart_mspm0_get_range(motor_range_t *range_out)
{
    if ((range_out == NULL) || !s_range.valid)
    {
        return false;
    }
    *range_out = s_range;
    return true;
}

void zdt_uart_mspm0_apply_range_status(can_protocol_status_t *status)
{
    uint16_t position;
    if ((status != NULL) &&
        motor_range_position(&s_range, g_zdt_uart_position_tenths, &position))
    {
        status->is_calibrated = true;
        status->position = position;
    }
}

static uint8_t s_rxRing[ZDT_UART_RX_RING_SIZE];
static volatile uint8_t s_rxHead;
static volatile uint8_t s_rxTail;
static zdt_uart_parser_t s_parser;
static zdt_uart_feedback_t s_feedback;
static uint32_t s_lastOptionsQueryMs;
static uint32_t s_lastPositionQueryMs;
static uint32_t s_lastStatusQueryMs;
static uint32_t s_lastBusVoltageQueryMs;
static uint32_t s_lastPositionRxMs;
static uint32_t s_lastStatusRxMs;
static uint32_t s_moveStartMs;
static zdt_uart_window_output_t s_currentOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
static zdt_uart_window_output_t s_requestedOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
static uint16_t s_requestedSpeedTenthsRpm = ZDT_UART_MOVE_SPEED_TENTHS_RPM;

static bool zdtUartMspm0Send(const uint8_t *frame, uint8_t length)
{
    uint8_t index;

    if ((frame == NULL) || (length == 0U))
    {
        return false;
    }

    for (index = 0U; index < length; ++index)
    {
        DL_UART_Main_transmitDataBlocking(UART_1_INST, frame[index]);
        ++g_zdt_uart_tx_byte_count;
        g_zdt_uart_tx_trace[(g_zdt_uart_tx_byte_count - 1U) &
                            ZDT_UART_RX_TRACE_MASK] = frame[index];
        g_zdt_uart_last_tx_frame[index] = frame[index];
    }

    g_zdt_uart_last_tx_frame_length = length;
    ++g_zdt_uart_tx_frame_count;

    /*
     * Keep a quiet interval between complete commands. The X42S manual says
     * back-to-back serial commands need a few milliseconds to avoid merging.
     */
    DL_Common_delayCycles(
        (CPUCLK_FREQ / 1000U) * ZDT_UART_INTERFRAME_DELAY_MS);

    return true;
}

static bool zdtUartMspm0SendSimpleRequest(uint8_t (*builder)(uint8_t, uint8_t[3]))
{
    uint8_t frame[3];
    const uint8_t length = builder(1U, frame);

    return (length != 0U) && zdtUartMspm0Send(frame, length);
}

static bool zdtUartMspm0SendStop(void)
{
    uint8_t frame[5];
    const uint8_t length = zdt_uart_build_stop(1U, frame);

    return (length != 0U) && zdtUartMspm0Send(frame, length);
}

bool zdt_uart_mspm0_request_stop(void)
{
    if (!zdtUartMspm0SendStop())
    {
        return false;
    }

    g_zdt_uart_move_request = ZDT_UART_MOVE_REQUEST_NONE;
    s_moveActive = false;
    s_currentOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
    s_requestedOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
    g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_STOP_SENT;
    g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NONE;
    return true;
}

#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
bool zdt_uart_mspm0_test_restart_controller(uint32_t now_ms)
{
    uint8_t frame[4U];
    const uint8_t length = zdt_uart_build_restart(1U, frame);

    if ((length == 0U) || s_moveActive ||
        (g_zdt_uart_move_request != ZDT_UART_MOVE_REQUEST_NONE) ||
        (s_currentOutput != ZDT_UART_WINDOW_OUTPUT_STOP) ||
        (s_requestedOutput != ZDT_UART_WINDOW_OUTPUT_STOP) ||
        (g_zdt_uart_range_valid == 0U) ||
        !zdt_uart_mspm0_motion_ready(now_ms))
    {
        g_zdt_uart_test_restart_result = 2U;
        return false;
    }

    if (!zdtUartMspm0Send(frame, length))
    {
        g_zdt_uart_test_restart_result = 3U;
        return false;
    }

    g_zdt_uart_test_restart_result = 1U;
    return true;
}
#endif

bool zdt_uart_mspm0_request_window_output(
    zdt_uart_window_output_t output, uint32_t now_ms)
{
    return zdt_uart_mspm0_request_window_output_at_speed(
        output, now_ms, ZDT_UART_MOVE_SPEED_TENTHS_RPM);
}

bool zdt_uart_mspm0_request_window_output_at_speed(
    zdt_uart_window_output_t output, uint32_t now_ms,
    uint16_t speed_tenths_rpm)
{
    if (output == ZDT_UART_WINDOW_OUTPUT_STOP)
    {
        return zdt_uart_mspm0_request_stop();
    }

    if ((output != ZDT_UART_WINDOW_OUTPUT_UP) &&
        (output != ZDT_UART_WINDOW_OUTPUT_DOWN))
    {
        return false;
    }
    if (speed_tenths_rpm == 0U)
    {
        return false;
    }

    if (!zdt_uart_mspm0_motion_ready(now_ms))
    {
        (void)zdt_uart_mspm0_request_stop();
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NOT_READY;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        return false;
    }

    if (output == s_requestedOutput)
    {
        return true;
    }

    if (s_moveActive && (output != s_currentOutput))
    {
        if (!zdt_uart_mspm0_request_stop())
        {
            g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_TX;
            g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
            return false;
        }
    }

    s_requestedSpeedTenthsRpm = speed_tenths_rpm;
    s_requestedOutput = output;
    g_zdt_uart_move_request =
        (output == ZDT_UART_WINDOW_OUTPUT_UP)
            ? ZDT_UART_MOVE_REQUEST_POSITIVE
            : ZDT_UART_MOVE_REQUEST_NEGATIVE;
    return true;
}

bool zdt_uart_mspm0_motion_ready(uint32_t now_ms)
{
    return (g_zdt_uart_rx_overflow_count == 0U) &&
           zdt_uart_motion_feedback_is_safe(
               &s_feedback, now_ms, s_lastPositionRxMs, s_lastStatusRxMs);
}

static void zdtUartMspm0PublishFeedback(uint32_t now_ms)
{
    int32_t position_tenths;

    g_zdt_uart_rx_frame_count = s_feedback.frames_received;
    g_zdt_uart_rx_rejected_count = s_feedback.frames_rejected;
    g_zdt_uart_last_function = s_feedback.last_function;
    g_zdt_uart_last_reply = s_feedback.last_reply;
    g_zdt_uart_option_flags = s_feedback.option_flags;
    g_zdt_uart_status_flags = s_feedback.status_flags;
    g_zdt_uart_bus_voltage_mv = s_feedback.bus_voltage_mv;
    g_zdt_uart_options_valid = s_feedback.options_valid;
    g_zdt_uart_status_valid = s_feedback.status_valid;
    g_zdt_uart_bus_voltage_valid = s_feedback.bus_voltage_valid;

#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    if (g_zdt_uart_test_position_freeze != 1U)
#endif
    {
        g_zdt_uart_position_raw = s_feedback.position_raw;
        g_zdt_uart_position_valid = s_feedback.position_valid;
        if (zdt_uart_position_tenths(&s_feedback, &position_tenths))
        {
            g_zdt_uart_position_tenths = position_tenths;
        }
    }

    g_zdt_uart_last_frame_ms = now_ms;
}

static void zdtUartMspm0CheckPositionChange(uint32_t now_ms)
{
    int32_t position_tenths;
    int64_t observed_delta;
    const int64_t requiredDelta =
        (g_zdt_uart_move_observed_delta_tenths < 0)
            ? -(int64_t)ZDT_UART_MOVE_MIN_DELTA_TENTHS
            : (int64_t)ZDT_UART_MOVE_MIN_DELTA_TENTHS;

    if (!s_moveActive || !zdt_uart_position_tenths(&s_feedback, &position_tenths))
    {
        return;
    }

    observed_delta = (int64_t)position_tenths - g_zdt_uart_move_baseline_tenths;
    if (((requiredDelta > 0) && (observed_delta >= requiredDelta)) ||
        ((requiredDelta < 0) && (observed_delta <= requiredDelta)))
    {
        g_zdt_uart_move_observed_delta_tenths = (int32_t)observed_delta;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_POSITION_CHANGED;
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NONE;
        g_zdt_uart_move_complete_ms = now_ms;
        s_moveActive = false;
    }
}

static void zdtUartMspm0ServiceRx(uint32_t now_ms)
{
    uint8_t budget = ZDT_UART_RX_SERVICE_BUDGET;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    if (g_zdt_uart_test_feedback_pause == 1U)
    {
        s_rxTail = s_rxHead;
        return;
    }
#endif

    while ((s_rxTail != s_rxHead) && (budget > 0U))
    {
        const uint8_t byte = s_rxRing[s_rxTail];
        zdt_uart_parse_result_t result;

        s_rxTail = (uint8_t)((s_rxTail + 1U) & ZDT_UART_RX_RING_MASK);
        --budget;
        result = zdt_uart_parser_feed(&s_parser, &s_feedback, byte);
        if (result == ZDT_UART_PARSE_ACCEPTED)
        {
            zdtUartMspm0PublishFeedback(now_ms);
            if (s_feedback.last_function == 0x1AU)
            {
                zdt_uart_query_schedule_rebase(
                    now_ms,
                    &s_lastPositionQueryMs,
                    &s_lastStatusQueryMs,
                    &s_lastBusVoltageQueryMs);
            }
            else if (s_feedback.last_function == 0x36U)
            {
                s_lastPositionRxMs = now_ms;
                g_zdt_uart_position_rx_ms = now_ms;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION) && \
    defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
                if (s_moveActive &&
                    (g_zdt_uart_test_first_position_rx_ms == UINT32_MAX))
                {
                    int32_t probePositionTenths;
                    if (zdt_uart_position_tenths(
                            &s_feedback, &probePositionTenths))
                    {
                        const int64_t probeDelta =
                            (int64_t)probePositionTenths -
                            g_zdt_uart_move_baseline_tenths;
                        g_zdt_uart_test_first_position_rx_ms = now_ms;
                        if (((s_targetDeltaTenths > 0) && (probeDelta > 0)) ||
                            ((s_targetDeltaTenths < 0) && (probeDelta < 0)))
                        {
                            g_zdt_uart_test_first_position_progress_ms = now_ms;
                        }
                    }
                }
#endif
#if defined(DUALECU_ENABLE_ZDT_UART_BENCH_TEST)
                zdtUartMspm0CheckPositionChange(now_ms);
#endif
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
                if (s_moveActive)
                {
                    const int64_t delta = (int64_t)g_zdt_uart_position_tenths -
                                          g_zdt_uart_move_baseline_tenths;
                    const uint32_t statusAfterStart =
                        (uint32_t)(s_lastStatusRxMs - s_moveStartMs);
                    if (zdt_uart_step_complete(delta, s_targetDeltaTenths,
                        s_feedback.status_flags,
                        (statusAfterStart > 0U) && (statusAfterStart < 0x80000000U)))
                    {
                        (void)zdt_uart_mspm0_request_stop();
                        g_zdt_uart_move_observed_delta_tenths = (int32_t)delta;
                        g_zdt_uart_move_complete_ms = now_ms;
                        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_POSITION_CHANGED;
                    }
                }
#endif
            }
            else if (s_feedback.last_function == 0x3AU)
            {
                s_lastStatusRxMs = now_ms;
                g_zdt_uart_last_status_ms = now_ms;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION) && \
    defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
                if (s_moveActive &&
                    (g_zdt_uart_test_first_status_rx_ms == UINT32_MAX))
                {
                    g_zdt_uart_test_first_status_rx_ms = now_ms;
                }
#endif
            }
            else if (s_feedback.last_function == 0x24U)
            {
                g_zdt_uart_last_bus_voltage_ms = now_ms;
            }
        }
        else if (result == ZDT_UART_PARSE_REJECTED)
        {
            g_zdt_uart_rx_frame_count = s_feedback.frames_received;
            g_zdt_uart_rx_rejected_count = s_feedback.frames_rejected;
        }
        else if (result == ZDT_UART_PARSE_DEVICE_ERROR)
        {
            uint8_t index;

            g_zdt_uart_rx_frame_count = s_feedback.frames_received;
            g_zdt_uart_device_error_count = s_feedback.device_error_count;
            g_zdt_uart_last_device_error = s_feedback.last_device_error;
            g_zdt_uart_last_device_error_processed_ms = now_ms;
            g_zdt_uart_last_device_error_tx_frame_count =
                g_zdt_uart_tx_frame_count;
            g_zdt_uart_last_device_error_tx_frame_length =
                g_zdt_uart_last_tx_frame_length;
            for (index = 0U; index < ZDT_UART_DEVICE_ERROR_FRAME_LEN; ++index)
            {
                g_zdt_uart_last_device_error_frame[index] =
                    s_feedback.last_device_error_frame[index];
            }
            for (index = 0U;
                 (index < g_zdt_uart_last_tx_frame_length) &&
                 (index < ZDT_UART_FRAME_MAX_LEN);
                 ++index)
            {
                g_zdt_uart_last_device_error_tx_frame[index] =
                    g_zdt_uart_last_tx_frame[index];
            }
            if (s_moveActive)
            {
                (void)zdt_uart_mspm0_request_stop();
                g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_FEEDBACK;
                g_zdt_uart_move_result =
                    ZDT_UART_MOVE_RESULT_FEEDBACK_STOPPED;
                g_zdt_uart_move_complete_ms = now_ms;
            }
        }
    }
}

static void zdtUartMspm0RequestMotion(uint32_t request, uint32_t now_ms)
{
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN];
    int32_t baselineTenths;
    const int32_t delta = (request == ZDT_UART_MOVE_REQUEST_POSITIVE)
                              ? ZDT_UART_MOVE_DELTA_TENTHS
                              : -ZDT_UART_MOVE_DELTA_TENTHS;

    if (s_moveActive)
    {
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_ALREADY_ACTIVE;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        return;
    }

    if ((g_zdt_uart_rx_overflow_count != 0U) ||
        !zdt_uart_motion_feedback_is_safe(
            &s_feedback, now_ms, s_lastPositionRxMs, s_lastStatusRxMs))
    {
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NOT_READY;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        return;
    }

    if (!zdt_uart_position_tenths(&s_feedback, &baselineTenths) ||
        !zdt_uart_build_relative_move(1U,
                                      delta,
                                      ZDT_UART_MOVE_ACCEL_RPM_S,
                                      ZDT_UART_MOVE_DECEL_RPM_S,
                                      ZDT_UART_MOVE_SPEED_TENTHS_RPM,
                                      ZDT_UART_MOVE_CURRENT_MA,
                                      frame) ||
        !zdtUartMspm0Send(frame, ZDT_UART_FRAME_MAX_LEN))
    {
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_TX;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        return;
    }

    g_zdt_uart_move_baseline_tenths = baselineTenths;
    g_zdt_uart_move_observed_delta_tenths = delta;
    g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NONE;
    g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_RUNNING;
    g_zdt_uart_move_complete_ms = 0U;
    s_moveStartMs = now_ms;
    s_lastPositionQueryMs = now_ms;
    s_moveActive = true;
}

#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
static void zdtUartMspm0RequestWindowStep(uint32_t request, uint32_t now_ms)
{
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN];
    zdt_uart_window_output_t output;
    int32_t baselineTenths;
    int32_t delta;

    if (s_moveActive)
    {
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_ALREADY_ACTIVE;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        return;
    }

    if ((g_zdt_uart_rx_overflow_count != 0U) ||
        !zdt_uart_mspm0_motion_ready(now_ms))
    {
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NOT_READY;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        s_requestedOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
        return;
    }

    output = (request == ZDT_UART_MOVE_REQUEST_POSITIVE)
                 ? ZDT_UART_WINDOW_OUTPUT_UP
                 : ZDT_UART_WINDOW_OUTPUT_DOWN;
    if (!zdt_uart_position_tenths(&s_feedback, &baselineTenths))
    {
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        return;
    }
    delta = motor_range_delta(&s_range, baselineTenths,
        output == ZDT_UART_WINDOW_OUTPUT_UP, ZDT_UART_MOVE_DELTA_TENTHS);
    if (delta == 0)
    {
        (void)zdt_uart_mspm0_request_stop();
        return;
    }
    if (!zdt_uart_build_relative_move(1U, delta,
        ZDT_UART_MOVE_ACCEL_RPM_S, ZDT_UART_MOVE_DECEL_RPM_S,
        s_requestedSpeedTenthsRpm, ZDT_UART_MOVE_CURRENT_MA, frame) ||
        !zdtUartMspm0Send(frame, ZDT_UART_FRAME_MAX_LEN))
    {
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_TX;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        s_requestedOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
        return;
    }

    g_zdt_uart_move_baseline_tenths = baselineTenths;
    s_targetDeltaTenths = delta;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    if (g_zdt_uart_test_feedback_pause == 2U)
    {
        g_zdt_uart_test_feedback_pause = 1U;
    }
    if (g_zdt_uart_test_position_freeze == 2U)
    {
        g_zdt_uart_test_position_freeze = 1U;
    }
#endif
    g_zdt_uart_move_observed_delta_tenths = 0;
    g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NONE;
    g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_RUNNING;
    g_zdt_uart_move_complete_ms = 0U;
    s_moveStartMs = now_ms;
    s_lastPositionQueryMs = now_ms;
    s_moveActive = true;
    s_currentOutput = output;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    g_zdt_uart_test_first_position_query_ms = UINT32_MAX;
    g_zdt_uart_test_first_status_query_ms = UINT32_MAX;
    g_zdt_uart_test_first_position_rx_ms = UINT32_MAX;
    g_zdt_uart_test_first_status_rx_ms = UINT32_MAX;
    g_zdt_uart_test_first_position_progress_ms = UINT32_MAX;
#endif
}
#endif

static void zdtUartMspm0ServiceMoveRequest(uint32_t now_ms)
{
    const uint32_t request = g_zdt_uart_move_request;

    if (request == ZDT_UART_MOVE_REQUEST_NONE)
    {
        return;
    }

    g_zdt_uart_move_request = ZDT_UART_MOVE_REQUEST_NONE;
    if (request == ZDT_UART_MOVE_REQUEST_STOP)
    {
        if (zdt_uart_mspm0_request_stop())
        {
            g_zdt_uart_move_complete_ms = now_ms;
        }
        else
        {
            g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_TX;
            g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
        }
        return;
    }

    if ((request == ZDT_UART_MOVE_REQUEST_POSITIVE) ||
        (request == ZDT_UART_MOVE_REQUEST_NEGATIVE))
    {
#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
        zdtUartMspm0RequestWindowStep(request, now_ms);
#else
        zdtUartMspm0RequestMotion(request, now_ms);
#endif
    }
    else
    {
        g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_NOT_READY;
        g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
    }
}

bool zdt_uart_mspm0_init(uint32_t now_ms)
{
    uint8_t optionsFrame[3];
    const uint8_t optionsLength = zdt_uart_build_request_options(1U, optionsFrame);

    s_rxHead = 0U;
    s_rxTail = 0U;
    s_moveActive = false;
    zdt_uart_mspm0_invalidate_range();
    s_currentOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
    s_requestedOutput = ZDT_UART_WINDOW_OUTPUT_STOP;
    g_zdt_uart_move_request = ZDT_UART_MOVE_REQUEST_NONE;
    s_lastOptionsQueryMs = now_ms;
    zdt_uart_query_schedule_rebase(
        now_ms,
        &s_lastPositionQueryMs,
        &s_lastStatusQueryMs,
        &s_lastBusVoltageQueryMs);
    s_lastPositionRxMs = 0U;
    s_lastStatusRxMs = 0U;
    g_zdt_uart_last_status_ms = 0U;
    g_zdt_uart_position_rx_ms = 0U;
    g_zdt_uart_first_rx_byte_count = 0U;
    g_zdt_uart_rx_byte_count = 0U;
    g_zdt_uart_rx_frame_count = 0U;
    g_zdt_uart_rx_rejected_count = 0U;
    g_zdt_uart_rx_overflow_count = 0U;
    g_zdt_uart_device_error_count = 0U;
    g_zdt_uart_last_device_error = 0U;
    g_zdt_uart_tx_frame_count = 0U;
    g_zdt_uart_tx_byte_count = 0U;
    g_zdt_uart_options_valid = 0U;
    g_zdt_uart_status_valid = 0U;
    g_zdt_uart_position_valid = 0U;
    g_zdt_uart_bus_voltage_mv = 0U;
    g_zdt_uart_bus_voltage_valid = 0U;
    g_zdt_uart_last_bus_voltage_ms = 0U;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    g_zdt_uart_test_feedback_pause = 0U;
    g_zdt_uart_test_position_freeze = 0U;
#endif
    zdt_uart_parser_init(&s_parser, 1U);
    s_feedback = (zdt_uart_feedback_t){0};

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    g_zdt_uart_init_result = 1U;

#if defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    if (!zdtUartMspm0SendStop())
    {
        g_zdt_uart_init_result = 3U;
        return false;
    }
#endif

    if ((optionsLength == 0U) || !zdtUartMspm0Send(optionsFrame, optionsLength))
    {
        g_zdt_uart_init_result = 2U;
        return false;
    }

    return true;
}

void zdt_uart_mspm0_process(uint32_t now_ms)
{
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN];

    if (g_zdt_uart_init_result != 1U)
    {
        return;
    }

    zdtUartMspm0ServiceRx(now_ms);
#if defined(DUALECU_ENABLE_ZDT_UART_BENCH_TEST) || \
    defined(DUALECU_ENABLE_ZDT_UART_CONTROL)
    zdtUartMspm0ServiceMoveRequest(now_ms);
#endif

    if (g_zdt_uart_rx_overflow_count != 0U)
    {
        if (s_moveActive)
        {
            if (zdt_uart_mspm0_request_stop())
            {
                g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_RX_OVERFLOW_STOPPED;
                g_zdt_uart_move_complete_ms = now_ms;
            }
            else
            {
                g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_TX;
                g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
            }
        }
        return;
    }

    if (s_moveActive &&
        !zdt_uart_motion_feedback_is_safe(
            &s_feedback, now_ms, s_lastPositionRxMs, s_lastStatusRxMs))
    {
        if (zdt_uart_mspm0_request_stop())
        {
            g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_FEEDBACK_STOPPED;
            g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_FEEDBACK;
            g_zdt_uart_move_complete_ms = now_ms;
        }
        else
        {
            s_moveActive = false;
            g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_TX;
            g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
            g_zdt_uart_move_complete_ms = now_ms;
        }
        return;
    }

    if (s_moveActive &&
        ((uint32_t)(now_ms - s_moveStartMs) >= ZDT_UART_MOVE_TIMEOUT_MS))
    {
        if (zdt_uart_mspm0_request_stop())
        {
            g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_TIMEOUT_STOPPED;
            g_zdt_uart_move_complete_ms = now_ms;
        }
        else
        {
            s_moveActive = false;
            g_zdt_uart_move_error = ZDT_UART_MOVE_ERROR_TX;
            g_zdt_uart_move_result = ZDT_UART_MOVE_RESULT_REJECTED;
            g_zdt_uart_move_complete_ms = now_ms;
        }
        return;
    }

    if (s_feedback.options_valid == 0U)
    {
        if ((uint32_t)(now_ms - s_lastOptionsQueryMs) >= ZDT_UART_OPTIONS_RETRY_MS)
        {
            s_lastOptionsQueryMs = now_ms;
            (void)zdtUartMspm0SendSimpleRequest(zdt_uart_build_request_options);
        }
        return;
    }

    if (zdt_uart_position_query_due_for_motion(
            now_ms, s_lastPositionQueryMs, s_moveActive))
    {
        const uint8_t length = zdt_uart_build_request_position(1U, frame);
        s_lastPositionQueryMs = now_ms;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
        if (s_moveActive &&
            (g_zdt_uart_test_first_position_query_ms == UINT32_MAX))
        {
            g_zdt_uart_test_first_position_query_ms = now_ms;
        }
#endif
        if (length != 0U)
        {
            (void)zdtUartMspm0Send(frame, length);
        }
    }
    else if (zdt_uart_status_query_due_for_motion(
                 now_ms, s_lastStatusQueryMs, s_moveActive))
    {
        const uint8_t length = zdt_uart_build_request_status(1U, frame);
        s_lastStatusQueryMs = now_ms;
#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
        if (s_moveActive &&
            (g_zdt_uart_test_first_status_query_ms == UINT32_MAX))
        {
            g_zdt_uart_test_first_status_query_ms = now_ms;
        }
#endif
        if (length != 0U)
        {
            (void)zdtUartMspm0Send(frame, length);
        }
    }
    else if (zdt_uart_bus_voltage_query_due(
                 now_ms, s_lastBusVoltageQueryMs))
    {
        s_lastBusVoltageQueryMs = now_ms;
        (void)zdtUartMspm0SendSimpleRequest(
            zdt_uart_build_request_bus_voltage);
    }
}

void UART1_IRQHandler(void)
{
    uint8_t budget = ZDT_UART_RX_SERVICE_BUDGET;
    while ((budget-- > 0U) && !DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
    {
        const uint8_t byte = DL_UART_Main_receiveData(UART_1_INST);
        const uint8_t nextHead = (uint8_t)((s_rxHead + 1U) & ZDT_UART_RX_RING_MASK);

        ++g_zdt_uart_rx_byte_count;
        g_zdt_uart_rx_trace[(g_zdt_uart_rx_byte_count - 1U) &
                            ZDT_UART_RX_TRACE_MASK] = byte;
        if (g_zdt_uart_first_rx_byte_count < sizeof(g_zdt_uart_first_rx_bytes))
        {
            g_zdt_uart_first_rx_bytes[g_zdt_uart_first_rx_byte_count++] = byte;
        }
        if (nextHead == s_rxTail)
        {
            ++g_zdt_uart_rx_overflow_count;
        }
        else
        {
            s_rxRing[s_rxHead] = byte;
            s_rxHead = nextHead;
        }
    }
}

#endif /* DUALECU_ENABLE_ZDT_UART_BENCH_TEST || DUALECU_ENABLE_ZDT_UART_CONTROL */
