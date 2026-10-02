#include "zdt_uart_protocol.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define ZDT_UART_FUNCTION_OPTIONS (0x1AU)
#define ZDT_UART_FUNCTION_RESTART (0x08U)
#define ZDT_UART_FUNCTION_BUS_VOLTAGE (0x24U)
#define ZDT_UART_FUNCTION_ERROR (0x00U)
#define ZDT_UART_FUNCTION_POSITION (0x36U)
#define ZDT_UART_FUNCTION_STATUS (0x3AU)
#define ZDT_UART_FUNCTION_STOP (0xFEU)
#define ZDT_UART_FUNCTION_LIMITED_SPEED (0xC6U)
#define ZDT_UART_FUNCTION_RELATIVE_POSITION (0xCDU)
#define ZDT_UART_REPLY_LENGTH (4U)
#define ZDT_UART_OPTIONS_LENGTH (5U)
#define ZDT_UART_BUS_VOLTAGE_LENGTH (5U)
#define ZDT_UART_POSITION_LENGTH (8U)
#define ZDT_UART_MAX_CURRENT_MA (5000U)
#define ZDT_UART_MAX_SPEED_TENTHS_RPM (30000U)

static void zdt_uart_parser_reset_frame(zdt_uart_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

static uint8_t zdt_uart_expected_length(uint8_t function)
{
    switch (function)
    {
    case ZDT_UART_FUNCTION_ERROR:
    case ZDT_UART_FUNCTION_RESTART:
        return ZDT_UART_REPLY_LENGTH;
    case ZDT_UART_FUNCTION_OPTIONS:
        return ZDT_UART_OPTIONS_LENGTH;
    case ZDT_UART_FUNCTION_BUS_VOLTAGE:
        return ZDT_UART_BUS_VOLTAGE_LENGTH;
    case ZDT_UART_FUNCTION_POSITION:
        return ZDT_UART_POSITION_LENGTH;
    case 0x26U: /* Bus current feedback. */
    case 0x27U: /* Phase current feedback. */
    case 0x35U: /* Speed feedback. */
    case 0x37U: /* Position-error feedback. */
    case 0x39U: /* Temperature feedback. */
    case ZDT_UART_FUNCTION_STATUS:
    case ZDT_UART_FUNCTION_LIMITED_SPEED:
    case ZDT_UART_FUNCTION_RELATIVE_POSITION: /* Command acknowledgement. */
        return ZDT_UART_REPLY_LENGTH;
    default:
        return 0U;
    }
}

static uint32_t zdt_uart_read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

static uint16_t zdt_uart_read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static void zdt_uart_write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static void zdt_uart_write_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}

static uint8_t zdt_uart_build_simple_request(uint8_t address,
                                            uint8_t function,
                                            uint8_t frame[3])
{
    if ((address == 0U) || (frame == NULL))
    {
        return 0U;
    }

    frame[0] = address;
    frame[1] = function;
    frame[2] = ZDT_UART_CHECK_BYTE;
    return 3U;
}

void zdt_uart_parser_init(zdt_uart_parser_t *parser, uint8_t address)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->address = (address == 0U) ? 1U : address;
}

zdt_uart_parse_result_t zdt_uart_parser_feed(
    zdt_uart_parser_t *parser, zdt_uart_feedback_t *feedback, uint8_t byte)
{
    uint8_t function;

    if ((parser == NULL) || (feedback == NULL))
    {
        return ZDT_UART_PARSE_REJECTED;
    }

    if (parser->length == 0U)
    {
        parser->frame[0] = byte;
        parser->length = 1U;
        return ZDT_UART_PARSE_NONE;
    }

    if (parser->length >= ZDT_UART_FRAME_MAX_LEN)
    {
        ++feedback->frames_rejected;
        zdt_uart_parser_reset_frame(parser);
        return ZDT_UART_PARSE_REJECTED;
    }

    parser->frame[parser->length++] = byte;
    if (parser->length == 2U)
    {
        parser->expected_length = zdt_uart_expected_length(byte);
        if (parser->expected_length == 0U)
        {
            /* Slide one byte forward so noise does not hide the next address. */
            parser->frame[0] = byte;
            parser->length = 1U;
        }
        return ZDT_UART_PARSE_NONE;
    }

    if (parser->expected_length == 0U || parser->length < parser->expected_length)
    {
        return ZDT_UART_PARSE_NONE;
    }

    function = parser->frame[1];
    if ((parser->frame[0] != parser->address) ||
        (parser->frame[parser->length - 1U] != ZDT_UART_CHECK_BYTE))
    {
        ++feedback->frames_rejected;
        zdt_uart_parser_reset_frame(parser);
        return ZDT_UART_PARSE_REJECTED;
    }

    if (function == ZDT_UART_FUNCTION_ERROR)
    {
        feedback->last_function = function;
        feedback->last_reply = parser->frame[2];
        feedback->last_device_error = parser->frame[2];
        memcpy(feedback->last_device_error_frame,
               parser->frame,
               ZDT_UART_REPLY_LENGTH);
        ++feedback->frames_received;
        ++feedback->device_error_count;
        zdt_uart_parser_reset_frame(parser);
        return ZDT_UART_PARSE_DEVICE_ERROR;
    }

    if ((function == ZDT_UART_FUNCTION_POSITION) && (parser->frame[2] > 1U))
    {
        ++feedback->frames_rejected;
        zdt_uart_parser_reset_frame(parser);
        return ZDT_UART_PARSE_REJECTED;
    }

    feedback->last_function = function;
    ++feedback->frames_received;
    if (function == ZDT_UART_FUNCTION_OPTIONS)
    {
        feedback->option_flags = ((uint16_t)parser->frame[2] << 8) | parser->frame[3];
        feedback->options_valid = 1U;
    }
    else if (function == ZDT_UART_FUNCTION_POSITION)
    {
        feedback->position_negative = parser->frame[2];
        feedback->position_raw = zdt_uart_read_u32_be(&parser->frame[3]);
        feedback->position_valid = 1U;
    }
    else if (function == ZDT_UART_FUNCTION_BUS_VOLTAGE)
    {
        feedback->bus_voltage_mv = zdt_uart_read_u16_be(&parser->frame[2]);
        feedback->bus_voltage_valid = 1U;
    }
    else
    {
        feedback->last_reply = parser->frame[2];
        if (function == ZDT_UART_FUNCTION_STATUS)
        {
            feedback->status_flags = parser->frame[2];
            feedback->status_valid = 1U;
        }
    }

    zdt_uart_parser_reset_frame(parser);
    return ZDT_UART_PARSE_ACCEPTED;
}

bool zdt_uart_position_tenths(
    const zdt_uart_feedback_t *feedback, int32_t *position_tenths)
{
    uint64_t magnitude;
    int32_t signed_position;

    if ((feedback == NULL) || (position_tenths == NULL) ||
        (feedback->options_valid == 0U) || (feedback->position_valid == 0U))
    {
        return false;
    }

    if ((feedback->option_flags & ZDT_UART_OPTION_FIRMWARE_EMM) != 0U)
    {
        magnitude = (((uint64_t)feedback->position_raw * 3600U) + 32768U) / 65536U;
    }
    else
    {
        magnitude = feedback->position_raw;
    }

    if (magnitude > (uint64_t)INT32_MAX)
    {
        return false;
    }

    signed_position = (int32_t)magnitude;
    *position_tenths = (feedback->position_negative != 0U) ? -signed_position : signed_position;
    return true;
}

bool zdt_uart_step_complete(int64_t observed_delta, int32_t requested_delta,
                            uint8_t status_flags, bool status_after_start)
{
    int64_t lower_bound;
    int64_t upper_bound;
    if (!status_after_start ||
        ((status_flags & ZDT_UART_STATUS_POSITION_REACHED) == 0U) ||
        (requested_delta == 0) || (observed_delta == 0) ||
        ((observed_delta > 0) != (requested_delta > 0)))
    {
        return false;
    }
    /* Full 0.5-degree bench steps allow 0.4-degree overshoot, covering the
     * observed 0.3-degree result; short boundary steps keep one quantum. */
    const int64_t overshoot =
        ((requested_delta >= 5) || (requested_delta <= -5)) ? 4 : 1;
    lower_bound = (requested_delta > 0) ?
        (int64_t)requested_delta - 1 : (int64_t)requested_delta - overshoot;
    upper_bound = (requested_delta > 0) ?
        (int64_t)requested_delta + overshoot : (int64_t)requested_delta + 1;
    return (observed_delta >= lower_bound) &&
           (observed_delta <= upper_bound);
}

bool zdt_uart_motion_feedback_is_safe(const zdt_uart_feedback_t *feedback,
                                     uint32_t now_ms,
                                     uint32_t position_rx_ms,
                                     uint32_t status_rx_ms)
{
    const uint8_t faultMask = (uint8_t)(ZDT_UART_STATUS_STALL |
                                        ZDT_UART_STATUS_STALL_PROTECTION);

    if ((feedback == NULL) ||
        (feedback->options_valid == 0U) ||
        (feedback->position_valid == 0U) ||
        (feedback->status_valid == 0U) ||
        ((feedback->status_flags & ZDT_UART_STATUS_ENABLED) == 0U) ||
        ((feedback->status_flags & faultMask) != 0U))
    {
        return false;
    }

    if (((uint32_t)(now_ms - position_rx_ms) > ZDT_UART_POSITION_FRESH_MS) ||
        ((uint32_t)(now_ms - status_rx_ms) > ZDT_UART_STATUS_FRESH_MS))
    {
        return false;
    }

    return true;
}

bool zdt_uart_device_error_event_take(uint32_t error_count,
                                      uint8_t error_code,
                                      uint32_t *handled_error_count,
                                      uint8_t *error_code_out)
{
    if ((handled_error_count == NULL) || (error_code_out == NULL) ||
        (error_count == *handled_error_count))
    {
        return false;
    }

    *handled_error_count = error_count;
    *error_code_out = error_code;
    return true;
}

bool zdt_uart_window_motion_is_permitted(
    const can_protocol_status_t *window_status,
    bool motor_feedback_safe,
    bool motor_rx_overflow)
{
    return (window_status != NULL) &&
           window_status->is_calibrated &&
           (window_status->fault == CAN_PROTOCOL_FAULT_NONE) &&
           motor_feedback_safe &&
           !motor_rx_overflow;
}

zdt_uart_window_output_t zdt_uart_select_window_output(
    const can_protocol_status_t *window_status,
    bool motor_feedback_safe,
    bool motor_rx_overflow)
{
    if (!zdt_uart_window_motion_is_permitted(
            window_status, motor_feedback_safe, motor_rx_overflow))
    {
        return ZDT_UART_WINDOW_OUTPUT_STOP;
    }

    switch (window_status->state)
    {
        case CAN_PROTOCOL_STATE_UP:
            return ZDT_UART_WINDOW_OUTPUT_UP;

        case CAN_PROTOCOL_STATE_DOWN:
            return ZDT_UART_WINDOW_OUTPUT_DOWN;

        case CAN_PROTOCOL_STATE_STOP:
        default:
            return ZDT_UART_WINDOW_OUTPUT_STOP;
    }
}

uint32_t zdt_uart_status_query_initial_mark_ms(uint32_t now_ms)
{
    return now_ms - (ZDT_UART_STATUS_QUERY_PERIOD_MS / 2U);
}

bool zdt_uart_status_query_due(uint32_t now_ms, uint32_t last_query_ms)
{
    return (uint32_t)(now_ms - last_query_ms) >=
           ZDT_UART_STATUS_QUERY_PERIOD_MS;
}

bool zdt_uart_position_query_due_for_motion(uint32_t now_ms,
                                           uint32_t last_query_ms,
                                           bool motion_active)
{
    const uint32_t periodMs = motion_active
                                  ? ZDT_UART_ACTIVE_POSITION_QUERY_PERIOD_MS
                                  : ZDT_UART_POSITION_QUERY_PERIOD_MS;

    return (uint32_t)(now_ms - last_query_ms) >= periodMs;
}

bool zdt_uart_status_query_due_for_motion(uint32_t now_ms,
                                          uint32_t last_query_ms,
                                          bool motion_active)
{
    const uint32_t periodMs = motion_active
                                  ? ZDT_UART_ACTIVE_STATUS_QUERY_PERIOD_MS
                                  : ZDT_UART_STATUS_QUERY_PERIOD_MS;

    return (uint32_t)(now_ms - last_query_ms) >= periodMs;
}

void zdt_uart_query_schedule_rebase(uint32_t now_ms,
                                    uint32_t *last_position_query_ms,
                                    uint32_t *last_status_query_ms,
                                    uint32_t *last_bus_voltage_query_ms)
{
    if ((last_position_query_ms == NULL) || (last_status_query_ms == NULL))
    {
        return;
    }

    *last_position_query_ms = now_ms;
    *last_status_query_ms = zdt_uart_status_query_initial_mark_ms(now_ms);
    if (last_bus_voltage_query_ms != NULL)
    {
        *last_bus_voltage_query_ms =
            now_ms - (ZDT_UART_BUS_VOLTAGE_QUERY_PERIOD_MS -
                      ZDT_UART_BUS_VOLTAGE_QUERY_PHASE_MS);
    }
}

bool zdt_uart_bus_voltage_query_due(uint32_t now_ms, uint32_t last_query_ms)
{
    return (uint32_t)(now_ms - last_query_ms) >=
           ZDT_UART_BUS_VOLTAGE_QUERY_PERIOD_MS;
}

uint8_t zdt_uart_build_request_options(uint8_t address, uint8_t frame[3])
{
    return zdt_uart_build_simple_request(address, ZDT_UART_FUNCTION_OPTIONS, frame);
}

uint8_t zdt_uart_build_request_position(uint8_t address, uint8_t frame[3])
{
    return zdt_uart_build_simple_request(address, ZDT_UART_FUNCTION_POSITION, frame);
}

uint8_t zdt_uart_build_request_status(uint8_t address, uint8_t frame[3])
{
    return zdt_uart_build_simple_request(address, ZDT_UART_FUNCTION_STATUS, frame);
}

uint8_t zdt_uart_build_request_bus_voltage(uint8_t address, uint8_t frame[3])
{
    return zdt_uart_build_simple_request(address,
                                         ZDT_UART_FUNCTION_BUS_VOLTAGE,
                                         frame);
}

uint8_t zdt_uart_build_restart(uint8_t address, uint8_t frame[4])
{
    if ((address == 0U) || (frame == NULL))
    {
        return 0U;
    }

    frame[0] = address;
    frame[1] = ZDT_UART_FUNCTION_RESTART;
    frame[2] = 0x97U;
    frame[3] = ZDT_UART_CHECK_BYTE;
    return 4U;
}

uint8_t zdt_uart_build_stop(uint8_t address, uint8_t frame[5])
{
    if ((address == 0U) || (frame == NULL))
    {
        return 0U;
    }

    frame[0] = address;
    frame[1] = ZDT_UART_FUNCTION_STOP;
    frame[2] = 0x98U;
    frame[3] = 0U;
    frame[4] = ZDT_UART_CHECK_BYTE;
    return 5U;
}

bool zdt_uart_build_relative_move(uint8_t address,
                                  int32_t delta_tenths,
                                  uint16_t acceleration_rpm_s,
                                  uint16_t deceleration_rpm_s,
                                  uint16_t speed_tenths_rpm,
                                  uint16_t max_current_ma,
                                  uint8_t frame[ZDT_UART_FRAME_MAX_LEN])
{
    uint32_t magnitude;
    uint64_t absolute_delta;

    if ((address == 0U) || (delta_tenths == 0) || (frame == NULL) ||
        (acceleration_rpm_s == 0U) || (deceleration_rpm_s == 0U) ||
        (speed_tenths_rpm == 0U) || (speed_tenths_rpm > ZDT_UART_MAX_SPEED_TENTHS_RPM) ||
        (max_current_ma == 0U) || (max_current_ma > ZDT_UART_MAX_CURRENT_MA))
    {
        return false;
    }

    absolute_delta = (delta_tenths < 0) ? (uint64_t)(-(int64_t)delta_tenths)
                                        : (uint64_t)delta_tenths;
    if (absolute_delta > UINT32_MAX)
    {
        return false;
    }
    magnitude = (uint32_t)absolute_delta;

    frame[0] = address;
    frame[1] = ZDT_UART_FUNCTION_RELATIVE_POSITION;
    frame[2] = (delta_tenths < 0) ? 1U : 0U;
    zdt_uart_write_u16_be(&frame[3], acceleration_rpm_s);
    zdt_uart_write_u16_be(&frame[5], deceleration_rpm_s);
    zdt_uart_write_u16_be(&frame[7], speed_tenths_rpm);
    zdt_uart_write_u32_be(&frame[9], magnitude);
    frame[13] = 2U; /* Relative to current actual position. */
    frame[14] = 0U; /* Do not synchronize motion with other drivers. */
    zdt_uart_write_u16_be(&frame[15], max_current_ma);
    frame[17] = ZDT_UART_CHECK_BYTE;
    return true;
}

bool zdt_uart_build_limited_speed(uint8_t address,
                                  uint8_t direction,
                                  uint16_t acceleration_rpm_s,
                                  uint16_t speed_tenths_rpm,
                                  uint16_t max_current_ma,
                                  uint8_t frame[ZDT_UART_FRAME_MAX_LEN])
{
    if ((address == 0U) || (direction > 1U) || (acceleration_rpm_s == 0U) ||
        (speed_tenths_rpm == 0U) ||
        (speed_tenths_rpm > ZDT_UART_MAX_SPEED_TENTHS_RPM) ||
        (max_current_ma == 0U) || (max_current_ma > ZDT_UART_MAX_CURRENT_MA) ||
        (frame == NULL))
    {
        return false;
    }

    frame[0] = address;
    frame[1] = ZDT_UART_FUNCTION_LIMITED_SPEED;
    frame[2] = direction;
    zdt_uart_write_u16_be(&frame[3], acceleration_rpm_s);
    zdt_uart_write_u16_be(&frame[5], speed_tenths_rpm);
    frame[7] = 0U; /* Execute immediately; do not queue behind a sync trigger. */
    zdt_uart_write_u16_be(&frame[8], max_current_ma);
    frame[10] = ZDT_UART_CHECK_BYTE;
    return true;
}

uint8_t zdt_uart_build_window_output_frame(
    zdt_uart_window_output_t output,
    uint16_t acceleration_rpm_s,
    uint16_t speed_tenths_rpm,
    uint16_t max_current_ma,
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN])
{
    if (frame == NULL)
    {
        return 0U;
    }

    switch (output)
    {
        case ZDT_UART_WINDOW_OUTPUT_UP:
            return zdt_uart_build_limited_speed(
                       1U, 0U, acceleration_rpm_s, speed_tenths_rpm,
                       max_current_ma, frame)
                       ? 11U
                       : 0U;

        case ZDT_UART_WINDOW_OUTPUT_DOWN:
            return zdt_uart_build_limited_speed(
                       1U, 1U, acceleration_rpm_s, speed_tenths_rpm,
                       max_current_ma, frame)
                       ? 11U
                       : 0U;

        case ZDT_UART_WINDOW_OUTPUT_STOP:
            return zdt_uart_build_stop(1U, frame);

        default:
            return 0U;
    }
}
