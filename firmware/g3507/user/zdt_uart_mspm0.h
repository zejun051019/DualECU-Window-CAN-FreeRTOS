#ifndef ZDT_UART_MSPM0_H
#define ZDT_UART_MSPM0_H

#include <stdbool.h>
#include <stdint.h>

#include "zdt_uart_protocol.h"
#include "motor_range.h"

typedef enum
{
    ZDT_UART_MOVE_REQUEST_NONE = 0U,
    ZDT_UART_MOVE_REQUEST_POSITIVE = 1U,
    ZDT_UART_MOVE_REQUEST_NEGATIVE = 2U,
    ZDT_UART_MOVE_REQUEST_STOP = 3U
} zdt_uart_move_request_t;

typedef enum
{
    ZDT_UART_MOVE_RESULT_IDLE = 0U,
    ZDT_UART_MOVE_RESULT_RUNNING,
    ZDT_UART_MOVE_RESULT_POSITION_CHANGED,
    ZDT_UART_MOVE_RESULT_STOP_SENT,
    ZDT_UART_MOVE_RESULT_TIMEOUT_STOPPED,
    ZDT_UART_MOVE_RESULT_REJECTED,
    ZDT_UART_MOVE_RESULT_RX_OVERFLOW_STOPPED,
    ZDT_UART_MOVE_RESULT_FEEDBACK_STOPPED
} zdt_uart_move_result_t;

typedef enum
{
    ZDT_UART_MOVE_ERROR_NONE = 0U,
    ZDT_UART_MOVE_ERROR_NOT_READY,
    ZDT_UART_MOVE_ERROR_ALREADY_ACTIVE,
    ZDT_UART_MOVE_ERROR_TX,
    ZDT_UART_MOVE_ERROR_FEEDBACK
} zdt_uart_move_error_t;

extern volatile uint32_t g_zdt_uart_init_result;
extern volatile uint32_t g_zdt_uart_tx_frame_count;
extern volatile uint32_t g_zdt_uart_tx_byte_count;
extern volatile uint8_t g_zdt_uart_tx_trace[64];
extern volatile uint32_t g_zdt_uart_rx_byte_count;
extern volatile uint8_t g_zdt_uart_rx_trace[64];
extern volatile uint32_t g_zdt_uart_rx_frame_count;
extern volatile uint32_t g_zdt_uart_rx_rejected_count;
extern volatile uint32_t g_zdt_uart_rx_overflow_count;
extern volatile uint32_t g_zdt_uart_device_error_count;
extern volatile uint32_t g_zdt_uart_last_device_error;
extern volatile uint32_t g_zdt_uart_first_rx_byte_count;
extern volatile uint8_t g_zdt_uart_first_rx_bytes[8];
extern volatile uint32_t g_zdt_uart_last_function;
extern volatile uint32_t g_zdt_uart_last_reply;
extern volatile uint32_t g_zdt_uart_option_flags;
extern volatile uint32_t g_zdt_uart_status_flags;
extern volatile uint32_t g_zdt_uart_bus_voltage_mv;
extern volatile uint32_t g_zdt_uart_position_raw;
extern volatile int32_t g_zdt_uart_position_tenths;
extern volatile uint32_t g_zdt_uart_options_valid;
extern volatile uint32_t g_zdt_uart_status_valid;
extern volatile uint32_t g_zdt_uart_position_valid;
extern volatile uint32_t g_zdt_uart_bus_voltage_valid;
extern volatile uint32_t g_zdt_uart_last_frame_ms;
extern volatile uint32_t g_zdt_uart_position_rx_ms;
extern volatile uint32_t g_zdt_uart_last_status_ms;
extern volatile uint32_t g_zdt_uart_last_bus_voltage_ms;
extern volatile uint32_t g_zdt_uart_move_request;
extern volatile uint32_t g_zdt_uart_move_result;
extern volatile uint32_t g_zdt_uart_move_error;
extern volatile int32_t g_zdt_uart_move_baseline_tenths;
extern volatile int32_t g_zdt_uart_move_observed_delta_tenths;
extern volatile uint32_t g_zdt_uart_move_complete_ms;
extern volatile uint32_t g_zdt_uart_range_valid;
extern volatile int32_t g_zdt_uart_range_min_tenths;
extern volatile int32_t g_zdt_uart_range_max_tenths;
extern volatile uint32_t g_zdt_uart_range_request;
extern volatile uint32_t g_zdt_uart_range_result;

/* Caller owns STOP/recovery gating; setting the bench range never moves. */
bool zdt_uart_mspm0_set_bench_range(uint32_t now_ms);
bool zdt_uart_mspm0_set_demo_range(uint32_t now_ms);
bool zdt_uart_mspm0_get_range(motor_range_t *range_out);
void zdt_uart_mspm0_invalidate_range(void);
void zdt_uart_mspm0_apply_range_status(can_protocol_status_t *status);

bool zdt_uart_mspm0_init(uint32_t now_ms);
void zdt_uart_mspm0_process(uint32_t now_ms);
bool zdt_uart_mspm0_motion_ready(uint32_t now_ms);
bool zdt_uart_mspm0_request_stop(void);
bool zdt_uart_mspm0_request_window_output(
    zdt_uart_window_output_t output, uint32_t now_ms);
bool zdt_uart_mspm0_request_window_output_at_speed(
    zdt_uart_window_output_t output, uint32_t now_ms,
    uint16_t speed_tenths_rpm);

#if defined(DUALECU_ENABLE_TEST_FAULT_INJECTION)
bool zdt_uart_mspm0_test_restart_controller(uint32_t now_ms);
#endif

#endif /* ZDT_UART_MSPM0_H */
