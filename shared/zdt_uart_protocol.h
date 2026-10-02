#ifndef ZDT_UART_PROTOCOL_H
#define ZDT_UART_PROTOCOL_H

#include "can_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#define ZDT_UART_CHECK_BYTE (0x6BU)
#define ZDT_UART_FRAME_MAX_LEN (18U)
#define ZDT_UART_OPTION_FIRMWARE_EMM (1U << 1)
#define ZDT_UART_STATUS_ENABLED (1U << 0)
#define ZDT_UART_STATUS_POSITION_REACHED (1U << 1)
#define ZDT_UART_STATUS_STALL (1U << 2)
#define ZDT_UART_STATUS_STALL_PROTECTION (1U << 3)
#define ZDT_UART_STATUS_POWER_MARKER (1U << 7)
#define ZDT_UART_POSITION_FRESH_MS (300U)
#define ZDT_UART_STATUS_FRESH_MS (1000U)
#define ZDT_UART_POSITION_QUERY_PERIOD_MS (100U)
#define ZDT_UART_ACTIVE_POSITION_QUERY_PERIOD_MS (20U)
#define ZDT_UART_STATUS_QUERY_PERIOD_MS (500U)
#define ZDT_UART_ACTIVE_STATUS_QUERY_PERIOD_MS (20U)
#define ZDT_UART_BUS_VOLTAGE_QUERY_PERIOD_MS (1000U)
#define ZDT_UART_BUS_VOLTAGE_QUERY_PHASE_MS (550U)

typedef enum
{
    ZDT_UART_PARSE_NONE = 0,
    ZDT_UART_PARSE_ACCEPTED,
    ZDT_UART_PARSE_REJECTED,
    ZDT_UART_PARSE_DEVICE_ERROR
} zdt_uart_parse_result_t;

typedef enum
{
    ZDT_UART_WINDOW_OUTPUT_STOP = 0U,
    ZDT_UART_WINDOW_OUTPUT_UP,
    ZDT_UART_WINDOW_OUTPUT_DOWN
} zdt_uart_window_output_t;

typedef struct
{
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN];
    uint8_t length;
    uint8_t expected_length;
    uint8_t address;
} zdt_uart_parser_t;

typedef struct
{
    uint8_t last_function;
    uint8_t last_reply;
    uint16_t option_flags;
    uint8_t status_flags;
    uint16_t bus_voltage_mv;
    uint32_t position_raw;
    uint8_t position_negative;
    uint8_t options_valid;
    uint8_t status_valid;
    uint8_t position_valid;
    uint8_t bus_voltage_valid;
    uint32_t frames_received;
    uint32_t frames_rejected;
    uint32_t device_error_count;
    uint8_t last_device_error;
    uint8_t last_device_error_frame[4U];
} zdt_uart_feedback_t;

void zdt_uart_parser_init(zdt_uart_parser_t *parser, uint8_t address);
zdt_uart_parse_result_t zdt_uart_parser_feed(
    zdt_uart_parser_t *parser, zdt_uart_feedback_t *feedback, uint8_t byte);
bool zdt_uart_position_tenths(
    const zdt_uart_feedback_t *feedback, int32_t *position_tenths);
bool zdt_uart_step_complete(int64_t observed_delta, int32_t requested_delta,
                            uint8_t status_flags, bool status_after_start);
bool zdt_uart_motion_feedback_is_safe(const zdt_uart_feedback_t *feedback,
                                     uint32_t now_ms,
                                     uint32_t position_rx_ms,
                                     uint32_t status_rx_ms);
bool zdt_uart_device_error_event_take(uint32_t error_count,
                                     uint8_t error_code,
                                     uint32_t *handled_error_count,
                                     uint8_t *error_code_out);
bool zdt_uart_window_motion_is_permitted(
    const can_protocol_status_t *window_status,
    bool motor_feedback_safe,
    bool motor_rx_overflow);
zdt_uart_window_output_t zdt_uart_select_window_output(
    const can_protocol_status_t *window_status,
    bool motor_feedback_safe,
    bool motor_rx_overflow);
uint32_t zdt_uart_status_query_initial_mark_ms(uint32_t now_ms);
bool zdt_uart_status_query_due(uint32_t now_ms, uint32_t last_query_ms);
bool zdt_uart_position_query_due_for_motion(uint32_t now_ms,
                                           uint32_t last_query_ms,
                                           bool motion_active);
bool zdt_uart_status_query_due_for_motion(uint32_t now_ms,
                                         uint32_t last_query_ms,
                                         bool motion_active);
void zdt_uart_query_schedule_rebase(uint32_t now_ms,
                                    uint32_t *last_position_query_ms,
                                    uint32_t *last_status_query_ms,
                                    uint32_t *last_bus_voltage_query_ms);
bool zdt_uart_bus_voltage_query_due(uint32_t now_ms,
                                    uint32_t last_query_ms);

uint8_t zdt_uart_build_request_options(uint8_t address, uint8_t frame[3]);
uint8_t zdt_uart_build_request_position(uint8_t address, uint8_t frame[3]);
uint8_t zdt_uart_build_request_status(uint8_t address, uint8_t frame[3]);
uint8_t zdt_uart_build_request_bus_voltage(uint8_t address, uint8_t frame[3]);
uint8_t zdt_uart_build_restart(uint8_t address, uint8_t frame[4]);
uint8_t zdt_uart_build_stop(uint8_t address, uint8_t frame[5]);
bool zdt_uart_build_relative_move(uint8_t address,
                                  int32_t delta_tenths,
                                  uint16_t acceleration_rpm_s,
                                  uint16_t deceleration_rpm_s,
                                  uint16_t speed_tenths_rpm,
                                  uint16_t max_current_ma,
                                  uint8_t frame[ZDT_UART_FRAME_MAX_LEN]);
bool zdt_uart_build_limited_speed(uint8_t address,
                                  uint8_t direction,
                                  uint16_t acceleration_rpm_s,
                                  uint16_t speed_tenths_rpm,
                                  uint16_t max_current_ma,
                                  uint8_t frame[ZDT_UART_FRAME_MAX_LEN]);
uint8_t zdt_uart_build_window_output_frame(
    zdt_uart_window_output_t output,
    uint16_t acceleration_rpm_s,
    uint16_t speed_tenths_rpm,
    uint16_t max_current_ma,
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN]);

#endif /* ZDT_UART_PROTOCOL_H */
