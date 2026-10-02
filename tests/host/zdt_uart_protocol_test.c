#include "../../shared/zdt_uart_protocol.h"
#include "../../shared/can_protocol.h"
#include "../../shared/window_state.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

extern uint8_t zdt_uart_build_restart(uint8_t address, uint8_t frame[4]);

static void test_motor_restart_wire_format_and_ack(void)
{
    zdt_uart_parser_t parser;
    zdt_uart_feedback_t feedback = {0};
    const uint8_t expected_request[] = {0x01U, 0x08U, 0x97U, 0x6BU};
    const uint8_t acknowledgement[] = {0x01U, 0x08U, 0x02U, 0x6BU};
    uint8_t request[4] = {0U};
    zdt_uart_parse_result_t result = ZDT_UART_PARSE_NONE;
    size_t index;

    assert(zdt_uart_build_restart(1U, request) == sizeof(request));
    assert(memcmp(request, expected_request, sizeof(request)) == 0);
    assert(zdt_uart_build_restart(0U, request) == 0U);

    zdt_uart_parser_init(&parser, 1U);
    for (index = 0U; index < sizeof(acknowledgement); ++index)
    {
        result = zdt_uart_parser_feed(
            &parser, &feedback, acknowledgement[index]);
    }
    assert(result == ZDT_UART_PARSE_ACCEPTED);
    assert(feedback.last_function == 0x08U);
    assert(feedback.last_reply == 0x02U);
}

static void test_read_only_requests(void)
{
    uint8_t frame[5] = {0U};

    assert(zdt_uart_build_request_options(1U, frame) == 3U);
    assert(frame[0] == 0x01U && frame[1] == 0x1AU && frame[2] == 0x6BU);

    assert(zdt_uart_build_request_position(1U, frame) == 3U);
    assert(frame[0] == 0x01U && frame[1] == 0x36U && frame[2] == 0x6BU);

    assert(zdt_uart_build_request_status(1U, frame) == 3U);
    assert(frame[0] == 0x01U && frame[1] == 0x3AU && frame[2] == 0x6BU);

    assert(zdt_uart_build_request_bus_voltage(1U, frame) == 3U);
    assert(frame[0] == 0x01U && frame[1] == 0x24U && frame[2] == 0x6BU);
}

static void test_status_poll_is_staggered_from_position_poll_across_wrap(void)
{
    const uint32_t startMs = 0xFFFFFF00U;
    uint32_t positionLastMs = startMs;
    uint32_t statusLastMs = zdt_uart_status_query_initial_mark_ms(startMs);
    uint32_t previousStatusMs = statusLastMs;
    uint32_t statusCount = 0U;
    uint32_t elapsedMs;

    for (elapsedMs = 1U; elapsedMs <= 2000U; ++elapsedMs)
    {
        const uint32_t nowMs = startMs + elapsedMs;
        bool positionQuerySent = false;

        if ((uint32_t)(nowMs - positionLastMs) >= 100U)
        {
            positionLastMs = nowMs;
            positionQuerySent = true;
        }

        if (zdt_uart_status_query_due(nowMs, statusLastMs))
        {
            assert(!positionQuerySent);
            assert((uint32_t)(nowMs - statusLastMs) == 500U);

            if (statusCount == 0U)
            {
                assert((uint32_t)(nowMs - startMs) == 250U);
            }
            else
            {
                assert((uint32_t)(nowMs - previousStatusMs) == 500U);
            }

            previousStatusMs = nowMs;
            statusLastMs = nowMs;
            ++statusCount;
        }
    }

    assert(statusCount == 4U);
}

static void test_active_status_poll_is_faster_and_wrap_safe(void)
{
    const uint32_t lastQueryMs = 0xFFFFFFF0U;

    assert(!zdt_uart_status_query_due_for_motion(
        lastQueryMs + ZDT_UART_ACTIVE_STATUS_QUERY_PERIOD_MS - 1U,
        lastQueryMs, true));
    assert(zdt_uart_status_query_due_for_motion(
        lastQueryMs + ZDT_UART_ACTIVE_STATUS_QUERY_PERIOD_MS,
        lastQueryMs, true));
    assert(!zdt_uart_status_query_due_for_motion(
        lastQueryMs + ZDT_UART_STATUS_QUERY_PERIOD_MS - 1U,
        lastQueryMs, false));
    assert(zdt_uart_status_query_due_for_motion(
        lastQueryMs + ZDT_UART_STATUS_QUERY_PERIOD_MS,
        lastQueryMs, false));
}

static void test_active_position_poll_is_faster_and_wrap_safe(void)
{
    const uint32_t lastQueryMs = 0xFFFFFFF0U;

    assert(!zdt_uart_position_query_due_for_motion(
        lastQueryMs + ZDT_UART_ACTIVE_POSITION_QUERY_PERIOD_MS - 1U,
        lastQueryMs, true));
    assert(zdt_uart_position_query_due_for_motion(
        lastQueryMs + ZDT_UART_ACTIVE_POSITION_QUERY_PERIOD_MS,
        lastQueryMs, true));
    assert(!zdt_uart_position_query_due_for_motion(
        lastQueryMs + ZDT_UART_POSITION_QUERY_PERIOD_MS - 1U,
        lastQueryMs, false));
    assert(zdt_uart_position_query_due_for_motion(
        lastQueryMs + ZDT_UART_POSITION_QUERY_PERIOD_MS,
        lastQueryMs, false));
}

static void test_voltage_poll_is_staggered_from_existing_queries_across_wrap(void)
{
    const uint32_t startMs = 0xFFFFFF00U;
    uint32_t positionLastMs = startMs;
    uint32_t statusLastMs = 0U;
    uint32_t voltageLastMs = 0U;
    uint32_t voltageCount = 0U;
    uint32_t previousVoltageMs = 0U;
    uint32_t elapsedMs;

    zdt_uart_query_schedule_rebase(
        startMs, &positionLastMs, &statusLastMs, &voltageLastMs);

    for (elapsedMs = 1U; elapsedMs <= 2550U; ++elapsedMs)
    {
        const uint32_t nowMs = startMs + elapsedMs;
        const bool positionDue =
            (uint32_t)(nowMs - positionLastMs) >=
            ZDT_UART_POSITION_QUERY_PERIOD_MS;
        const bool statusDue = zdt_uart_status_query_due(nowMs, statusLastMs);
        const bool voltageDue =
            zdt_uart_bus_voltage_query_due(nowMs, voltageLastMs);

        assert(!(positionDue && statusDue));
        assert(!(positionDue && voltageDue));
        assert(!(statusDue && voltageDue));

        if (positionDue)
        {
            positionLastMs = nowMs;
        }
        if (statusDue)
        {
            statusLastMs = nowMs;
        }
        if (voltageDue)
        {
            if (voltageCount == 0U)
            {
                assert(elapsedMs == ZDT_UART_BUS_VOLTAGE_QUERY_PHASE_MS);
            }
            else
            {
                assert((uint32_t)(nowMs - previousVoltageMs) ==
                       ZDT_UART_BUS_VOLTAGE_QUERY_PERIOD_MS);
            }
            previousVoltageMs = nowMs;
            voltageLastMs = nowMs;
            ++voltageCount;
        }
    }

    assert(voltageCount == 3U);
}

static void test_query_schedule_rebases_after_delayed_options_response(void)
{
    const uint32_t startupMs = 0xFFFFFF00U;
    const uint32_t optionsReadyMs = startupMs + 600U;
    uint32_t positionLastMs = startupMs;
    uint32_t statusLastMs = zdt_uart_status_query_initial_mark_ms(startupMs);
    uint32_t voltageLastMs = 0U;

    /* Both old marks are overdue after the option retry delay. */
    assert((uint32_t)(optionsReadyMs - positionLastMs) >=
           ZDT_UART_POSITION_QUERY_PERIOD_MS);
    assert(zdt_uart_status_query_due(optionsReadyMs, statusLastMs));

    zdt_uart_query_schedule_rebase(
        optionsReadyMs, &positionLastMs, &statusLastMs, &voltageLastMs);

    /* A valid options reply starts fresh, staggered query periods. */
    assert(positionLastMs == optionsReadyMs);
    assert(statusLastMs ==
           zdt_uart_status_query_initial_mark_ms(optionsReadyMs));
    assert((uint32_t)(optionsReadyMs - positionLastMs) == 0U);
    assert(!zdt_uart_status_query_due(optionsReadyMs, statusLastMs));
    assert((uint32_t)(optionsReadyMs + ZDT_UART_POSITION_QUERY_PERIOD_MS -
                      positionLastMs) == ZDT_UART_POSITION_QUERY_PERIOD_MS);
    assert(!zdt_uart_status_query_due(
        optionsReadyMs + ZDT_UART_POSITION_QUERY_PERIOD_MS, statusLastMs));
    assert(zdt_uart_status_query_due(optionsReadyMs + 250U, statusLastMs));
    assert(!zdt_uart_bus_voltage_query_due(optionsReadyMs, voltageLastMs));
    assert(!zdt_uart_bus_voltage_query_due(
        optionsReadyMs + ZDT_UART_BUS_VOLTAGE_QUERY_PHASE_MS - 1U,
        voltageLastMs));
    assert(zdt_uart_bus_voltage_query_due(
        optionsReadyMs + ZDT_UART_BUS_VOLTAGE_QUERY_PHASE_MS,
        voltageLastMs));
}

static void test_options_position_and_status_feedback(void)
{
    zdt_uart_parser_t parser;
    zdt_uart_feedback_t feedback = {0};
    const uint8_t options[] = {0x01U, 0x1AU, 0x00U, 0x04U, 0x6BU};
    const uint8_t position[] = {0x01U, 0x36U, 0x00U, 0x00U, 0x00U, 0x00U, 0x32U, 0x6BU};
    const uint8_t status[] = {0x01U, 0x3AU, 0x07U, 0x6BU};
    int32_t position_tenths = 0;
    size_t index;

    zdt_uart_parser_init(&parser, 1U);
    for (index = 0U; index < sizeof(options); ++index)
    {
        (void)zdt_uart_parser_feed(&parser, &feedback, options[index]);
    }
    assert(feedback.options_valid == 1U);
    assert(feedback.option_flags == 0x0004U);

    for (index = 0U; index < sizeof(position); ++index)
    {
        (void)zdt_uart_parser_feed(&parser, &feedback, position[index]);
    }
    assert(feedback.position_valid == 1U);
    assert(feedback.position_raw == 50U);
    assert(zdt_uart_position_tenths(&feedback, &position_tenths));
    assert(position_tenths == 50);

    for (index = 0U; index < sizeof(status); ++index)
    {
        (void)zdt_uart_parser_feed(&parser, &feedback, status[index]);
    }
    assert(feedback.status_valid == 1U);
    assert(feedback.status_flags == 0x07U);
    assert(feedback.frames_received == 3U);
}

static void test_bus_voltage_reply_is_five_bytes(void)
{
    zdt_uart_parser_t parser;
    zdt_uart_feedback_t feedback = {0};
    const uint8_t voltage[] = {0x01U, 0x24U, 0x2EU, 0xE0U, 0x6BU};
    zdt_uart_parse_result_t result = ZDT_UART_PARSE_NONE;
    size_t index;

    zdt_uart_parser_init(&parser, 1U);
    for (index = 0U; index < sizeof(voltage); ++index)
    {
        result = zdt_uart_parser_feed(&parser, &feedback, voltage[index]);
    }

    /* 0x2EE0 is 12000 mV; the reply includes two data bytes and checksum. */
    assert(result == ZDT_UART_PARSE_ACCEPTED);
    assert(feedback.frames_received == 1U);
    assert(feedback.bus_voltage_valid == 1U);
    assert(feedback.bus_voltage_mv == 12000U);

    {
        const uint8_t badChecksum[] =
            {0x01U, 0x24U, 0x12U, 0x34U, 0x00U};

        for (index = 0U; index < sizeof(badChecksum); ++index)
        {
            result = zdt_uart_parser_feed(
                &parser, &feedback, badChecksum[index]);
        }
    }
    assert(result == ZDT_UART_PARSE_REJECTED);
    assert(feedback.frames_received == 1U);
    assert(feedback.frames_rejected == 1U);
    assert(feedback.bus_voltage_valid == 1U);
    assert(feedback.bus_voltage_mv == 12000U);
}

static void test_device_command_error_frame_is_reported_separately(void)
{
    zdt_uart_parser_t parser;
    zdt_uart_feedback_t feedback = {0};
    const uint8_t error[] = {0x01U, 0x00U, 0xEEU, 0x6BU};
    zdt_uart_parse_result_t result = ZDT_UART_PARSE_NONE;
    size_t index;

    zdt_uart_parser_init(&parser, 1U);
    for (index = 0U; index < sizeof(error); ++index)
    {
        result = zdt_uart_parser_feed(&parser, &feedback, error[index]);
    }

    assert(result == ZDT_UART_PARSE_DEVICE_ERROR);
    assert(feedback.device_error_count == 1U);
    assert(feedback.last_device_error == 0xEEU);
    for (index = 0U; index < sizeof(error); ++index)
    {
        assert(feedback.last_device_error_frame[index] == error[index]);
    }
    assert(feedback.status_valid == 0U);
    assert(feedback.frames_rejected == 0U);
}

static void test_device_error_event_is_consumed_once(void)
{
    uint32_t consumed_count = 0U;
    uint8_t error_code = 0U;

    assert(zdt_uart_device_error_event_take(
        1U, 0xE2U, &consumed_count, &error_code));
    assert(consumed_count == 1U);
    assert(error_code == 0xE2U);
    assert(!zdt_uart_device_error_event_take(
        1U, 0xE2U, &consumed_count, &error_code));

    assert(zdt_uart_device_error_event_take(
        2U, 0xEEU, &consumed_count, &error_code));
    assert(consumed_count == 2U);
    assert(error_code == 0xEEU);
}

static void test_position_units_and_sign_depend_on_options(void)
{
    zdt_uart_feedback_t feedback = {0};
    int32_t position_tenths = 0;

    feedback.options_valid = 1U;
    feedback.position_valid = 1U;
    feedback.option_flags = ZDT_UART_OPTION_FIRMWARE_EMM;
    feedback.position_raw = 65536U;
    feedback.position_negative = 1U;
    assert(zdt_uart_position_tenths(&feedback, &position_tenths));
    assert(position_tenths == -3600);

    feedback.option_flags = 0U;
    feedback.position_raw = 0xFFFFFFFFU;
    assert(!zdt_uart_position_tenths(&feedback, &position_tenths));

    feedback.options_valid = 0U;
    feedback.position_raw = 100U;
    assert(!zdt_uart_position_tenths(&feedback, &position_tenths));
}

static void test_motion_gate_requires_fresh_enabled_fault_free_feedback(void)
{
    zdt_uart_feedback_t feedback = {0};
    const uint32_t now_ms = 2000U;

    feedback.options_valid = 1U;
    feedback.position_valid = 1U;
    feedback.status_valid = 1U;
    feedback.status_flags = 0x03U;

    assert(zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1900U, 1500U));

    feedback.status_valid = 0U;
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1900U, 1500U));
    feedback.status_valid = 1U;

    feedback.options_valid = 0U;
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1900U, 1500U));
    feedback.options_valid = 1U;

    feedback.position_valid = 0U;
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1900U, 1500U));
}

static void test_motion_gate_rejects_stale_faulted_or_disabled_feedback(void)
{
    zdt_uart_feedback_t feedback = {0};
    const uint32_t now_ms = 2000U;

    feedback.options_valid = 1U;
    feedback.position_valid = 1U;
    feedback.status_valid = 1U;
    feedback.status_flags = 0x03U;

    assert(zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1700U, 1000U));
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1699U, 1000U));
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1700U, 999U));

    feedback.status_flags = 0x00U;
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1700U, 1500U));
    feedback.status_flags = 0x07U; /* Enabled + position reached + stall. */
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1700U, 1500U));
    feedback.status_flags = 0x0BU; /* Enabled + position reached + stall protection. */
    assert(!zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1700U, 1500U));
    feedback.status_flags = (uint8_t)(0x03U | ZDT_UART_STATUS_POWER_MARKER);
    /* The manual says bit7 is a configurable reset marker, not a live fault. */
    assert(zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1700U, 1500U));

    feedback.status_flags = 0x33U; /* UART pin levels are not limit-switch faults. */
    assert(zdt_uart_motion_feedback_is_safe(&feedback, now_ms, 1700U, 1500U));
}

static void test_motion_gate_handles_tick_wraparound(void)
{
    zdt_uart_feedback_t feedback = {0};

    feedback.options_valid = 1U;
    feedback.position_valid = 1U;
    feedback.status_valid = 1U;
    feedback.status_flags = 0x01U;

    assert(zdt_uart_motion_feedback_is_safe(
        &feedback, 0x00000020U, 0xFFFFFFF0U, 0xFFFFFF00U));
}

static void test_bad_frames_are_rejected(void)
{
    zdt_uart_parser_t parser;
    zdt_uart_feedback_t feedback = {0};
    const uint8_t wrong_address[] = {0x02U, 0x3AU, 0x01U, 0x6BU};
    const uint8_t wrong_check[] = {0x01U, 0x3AU, 0x01U, 0x00U};
    const uint8_t bad_sign[] = {0x01U, 0x36U, 0x02U, 0U, 0U, 0U, 1U, 0x6BU};
    size_t index;
    zdt_uart_parse_result_t result = ZDT_UART_PARSE_NONE;

    zdt_uart_parser_init(&parser, 1U);
    for (index = 0U; index < sizeof(wrong_address); ++index)
    {
        result = zdt_uart_parser_feed(&parser, &feedback, wrong_address[index]);
    }
    assert(result == ZDT_UART_PARSE_REJECTED);

    for (index = 0U; index < sizeof(wrong_check); ++index)
    {
        result = zdt_uart_parser_feed(&parser, &feedback, wrong_check[index]);
    }
    assert(result == ZDT_UART_PARSE_REJECTED);

    for (index = 0U; index < sizeof(bad_sign); ++index)
    {
        result = zdt_uart_parser_feed(&parser, &feedback, bad_sign[index]);
    }
    assert(result == ZDT_UART_PARSE_REJECTED);
    assert(feedback.frames_received == 0U);
    assert(feedback.frames_rejected == 3U);
}

static void test_relative_move_is_explicit_and_bounded(void)
{
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN] = {0U};
    const uint8_t expected[] = {
        0x01U, 0xCDU, 0x00U,
        0x00U, 0x0AU, 0x00U, 0x0AU,
        0x00U, 0x0AU, 0x00U, 0x00U, 0x00U, 0x05U,
        0x02U, 0x00U, 0x01U, 0xF4U, 0x6BU};
    const uint8_t expected_negative_direction = 0x01U;
    size_t index;

    assert(zdt_uart_build_relative_move(1U, 5, 10U, 10U, 10U, 500U, frame));
    for (index = 0U; index < sizeof(expected); ++index)
    {
        assert(frame[index] == expected[index]);
    }

    assert(zdt_uart_build_relative_move(1U, -5, 10U, 10U, 10U, 500U, frame));
    assert(frame[2] == expected_negative_direction);
    assert(!zdt_uart_build_relative_move(1U, 0, 10U, 10U, 10U, 500U, frame));
    assert(!zdt_uart_build_relative_move(1U, 5, 10U, 10U, 0U, 500U, frame));
    assert(!zdt_uart_build_relative_move(1U, 5, 10U, 10U, 10U, 6000U, frame));
}

static void test_limited_speed_command_matches_x42s_wire_format(void)
{
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN] = {0U};
    const uint8_t expected[] = {
        0x01U, 0xC6U, 0x00U, 0x00U, 0x0AU, 0x00U,
        0x0AU, 0x00U, 0x01U, 0xF4U, 0x6BU};
    size_t index;

    assert(zdt_uart_build_limited_speed(1U, 0U, 10U, 10U, 500U, frame));
    for (index = 0U; index < sizeof(expected); ++index)
    {
        assert(frame[index] == expected[index]);
    }

    assert(zdt_uart_build_limited_speed(1U, 1U, 10U, 10U, 500U, frame));
    assert(frame[2] == 1U);
    assert(!zdt_uart_build_limited_speed(0U, 0U, 10U, 10U, 500U, frame));
    assert(!zdt_uart_build_limited_speed(1U, 2U, 10U, 10U, 500U, frame));
    assert(!zdt_uart_build_limited_speed(1U, 0U, 0U, 10U, 500U, frame));
    assert(!zdt_uart_build_limited_speed(1U, 0U, 10U, 0U, 500U, frame));
    assert(!zdt_uart_build_limited_speed(1U, 0U, 10U, 10U, 0U, frame));
    assert(!zdt_uart_build_limited_speed(1U, 0U, 10U, 10U, 5001U, frame));
}

static void test_window_outputs_map_to_bounded_uart_frames(void)
{
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN] = {0U};
    const uint8_t expected_up[] = {
        0x01U, 0xC6U, 0x00U, 0x00U, 0x0AU, 0x00U,
        0x0AU, 0x00U, 0x01U, 0xF4U, 0x6BU};
    const uint8_t expected_down[] = {
        0x01U, 0xC6U, 0x01U, 0x00U, 0x0AU, 0x00U,
        0x0AU, 0x00U, 0x01U, 0xF4U, 0x6BU};
    size_t index;

    assert(zdt_uart_build_window_output_frame(
               ZDT_UART_WINDOW_OUTPUT_UP, 10U, 10U, 500U, frame) ==
           sizeof(expected_up));
    for (index = 0U; index < sizeof(expected_up); ++index)
    {
        assert(frame[index] == expected_up[index]);
    }

    assert(zdt_uart_build_window_output_frame(
               ZDT_UART_WINDOW_OUTPUT_DOWN, 10U, 10U, 500U, frame) ==
           sizeof(expected_down));
    for (index = 0U; index < sizeof(expected_down); ++index)
    {
        assert(frame[index] == expected_down[index]);
    }

    assert(zdt_uart_build_window_output_frame(
               ZDT_UART_WINDOW_OUTPUT_STOP, 10U, 10U, 500U, frame) == 5U);
    assert(frame[0] == 0x01U && frame[1] == 0xFEU && frame[2] == 0x98U &&
           frame[3] == 0x00U && frame[4] == 0x6BU);
    assert(zdt_uart_build_window_output_frame(
               (zdt_uart_window_output_t)3, 10U, 10U, 500U, frame) == 0U);
}

static void test_limited_speed_ack_is_accepted(void)
{
    zdt_uart_parser_t parser;
    zdt_uart_feedback_t feedback = {0};
    const uint8_t acknowledgement[] = {0x01U, 0xC6U, 0x02U, 0x6BU};
    zdt_uart_parse_result_t result = ZDT_UART_PARSE_NONE;
    size_t index;

    zdt_uart_parser_init(&parser, 1U);
    for (index = 0U; index < sizeof(acknowledgement); ++index)
    {
        result = zdt_uart_parser_feed(&parser, &feedback, acknowledgement[index]);
    }

    assert(result == ZDT_UART_PARSE_ACCEPTED);
    assert(feedback.last_function == 0xC6U);
    assert(feedback.last_reply == 0x02U);
}

static void test_can_motion_requires_calibration_and_fresh_motor_feedback(void)
{
    can_protocol_status_t window = {0};
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN] = {0U};

    window.state = CAN_PROTOCOL_STATE_UP;
    window.fault = CAN_PROTOCOL_FAULT_NONE;
    window.is_calibrated = false;
    assert(!zdt_uart_window_motion_is_permitted(&window, true, false));
    assert(zdt_uart_select_window_output(&window, true, false) ==
           ZDT_UART_WINDOW_OUTPUT_STOP);
    assert(zdt_uart_build_window_output_frame(
               zdt_uart_select_window_output(&window, true, false),
               10U, 10U, 500U, frame) == 5U);
    assert(frame[1] == 0xFEU);

    window.is_calibrated = true;
    assert(zdt_uart_window_motion_is_permitted(&window, true, false));
    assert(!zdt_uart_window_motion_is_permitted(&window, false, false));
    assert(!zdt_uart_window_motion_is_permitted(&window, true, true));
    assert(zdt_uart_select_window_output(&window, false, false) ==
           ZDT_UART_WINDOW_OUTPUT_STOP);
    assert(zdt_uart_select_window_output(&window, true, true) ==
           ZDT_UART_WINDOW_OUTPUT_STOP);

    window.fault = CAN_PROTOCOL_FAULT_COMM_TIMEOUT;
    assert(zdt_uart_select_window_output(&window, true, false) ==
           ZDT_UART_WINDOW_OUTPUT_STOP);

    window.fault = CAN_PROTOCOL_FAULT_NONE;
    assert(zdt_uart_select_window_output(&window, true, false) ==
           ZDT_UART_WINDOW_OUTPUT_UP);

    window.state = CAN_PROTOCOL_STATE_DOWN;
    assert(zdt_uart_select_window_output(&window, true, false) ==
           ZDT_UART_WINDOW_OUTPUT_DOWN);

    window.state = CAN_PROTOCOL_STATE_STOP;
    assert(zdt_uart_select_window_output(&window, true, false) ==
           ZDT_UART_WINDOW_OUTPUT_STOP);
}

static void test_local_motor_stop_does_not_restart_repeated_can_motion(void)
{
    window_state_t state;
    can_protocol_command_t command = {10U, CAN_PROTOCOL_CMD_STOP};
    can_protocol_frame_t frame = {0};
    can_protocol_status_t status = {0};

    window_state_init(&state);
    assert(can_protocol_encode_command(&command, &frame));
    assert(window_state_receive_frame_at(&state, &frame, 1U, 1U) ==
           WINDOW_STATE_RX_BASELINE_ACCEPTED);

    command.sequence = 11U;
    command.command = CAN_PROTOCOL_CMD_CLEAR_FAULT;
    assert(can_protocol_encode_command(&command, &frame));
    assert(window_state_receive_frame_at(&state, &frame, 2U, 2U) ==
           WINDOW_STATE_RX_CLEAR_SUCCEEDED);

    command.sequence = 12U;
    command.command = CAN_PROTOCOL_CMD_STOP;
    assert(can_protocol_encode_command(&command, &frame));
    assert(window_state_receive_frame_at(&state, &frame, 3U, 3U) ==
           WINDOW_STATE_RX_ACCEPTED);

    command.sequence = 13U;
    command.command = CAN_PROTOCOL_CMD_UP;
    assert(can_protocol_encode_command(&command, &frame));
    assert(window_state_receive_frame_at(&state, &frame, 4U, 4U) ==
           WINDOW_STATE_RX_ACCEPTED);
    assert(window_state_get_status(&state, &status));
    assert(status.state == CAN_PROTOCOL_STATE_UP);
    assert(!status.is_calibrated);
    assert(!zdt_uart_window_motion_is_permitted(&status, true, false));
    assert(zdt_uart_select_window_output(&status, true, false) ==
           ZDT_UART_WINDOW_OUTPUT_STOP);

    /* An uncalibrated gate or motor timeout keeps this event consumed. */
    window_state_note_local_output_stop(&state);
    assert(window_state_get_status(&state, &status));
    assert(status.state == CAN_PROTOCOL_STATE_STOP);
    assert(status.fault == CAN_PROTOCOL_FAULT_NONE);
    assert(status.last_sequence_valid && status.last_sequence == 13U);

    /* F407 repeats the current event every 50 ms; it must not restart motion. */
    assert(window_state_receive_frame_at(&state, &frame, 54U, 54U) ==
           WINDOW_STATE_RX_REPEAT_ACCEPTED);
    assert(window_state_get_status(&state, &status));
    assert(status.state == CAN_PROTOCOL_STATE_STOP);
    assert(status.last_sequence_valid && status.last_sequence == 13U);

    /* Only a new command event may request another movement. */
    command.sequence = 14U;
    assert(can_protocol_encode_command(&command, &frame));
    assert(window_state_receive_frame_at(&state, &frame, 55U, 55U) ==
           WINDOW_STATE_RX_ACCEPTED);
    assert(window_state_get_status(&state, &status));
    assert(status.state == CAN_PROTOCOL_STATE_UP);
}

int main(void)
{
    uint8_t stop[5] = {0U};

    test_motor_restart_wire_format_and_ack();
    test_read_only_requests();
    assert(zdt_uart_step_complete(-4, -5, 3U, true));
    assert(zdt_uart_step_complete(6, 5, 3U, true));
    /* Bench trace: a commanded +0.5 degree step reported +0.8 degree. */
    assert(zdt_uart_step_complete(8, 5, 3U, true));
    assert(zdt_uart_step_complete(7, 5, 3U, true));
    assert(zdt_uart_step_complete(-7, -5, 3U, true));
    assert(zdt_uart_step_complete(9, 5, 3U, true));
    assert(zdt_uart_step_complete(-9, -5, 3U, true));
    assert(!zdt_uart_step_complete(10, 5, 3U, true));
    assert(!zdt_uart_step_complete(-10, -5, 3U, true));
    assert(!zdt_uart_step_complete(3, 1, 3U, true));
    assert(!zdt_uart_step_complete(4, 5, 3U, false));
    assert(!zdt_uart_step_complete(4, 5, 1U, true));
    assert(!zdt_uart_step_complete(0, 1, 3U, true));
    assert(!zdt_uart_step_complete(-5, 5, 3U, true));
    assert(!zdt_uart_step_complete(3, 5, 3U, true));
    assert(!zdt_uart_step_complete(100, 5, 3U, true));
    assert(!zdt_uart_step_complete(INT64_MAX, 5, 3U, true));
    test_status_poll_is_staggered_from_position_poll_across_wrap();
    test_active_status_poll_is_faster_and_wrap_safe();
    test_active_position_poll_is_faster_and_wrap_safe();
    test_query_schedule_rebases_after_delayed_options_response();
    test_options_position_and_status_feedback();
    test_bus_voltage_reply_is_five_bytes();
    test_voltage_poll_is_staggered_from_existing_queries_across_wrap();
    test_device_command_error_frame_is_reported_separately();
    test_device_error_event_is_consumed_once();
    test_position_units_and_sign_depend_on_options();
    test_motion_gate_requires_fresh_enabled_fault_free_feedback();
    test_motion_gate_rejects_stale_faulted_or_disabled_feedback();
    test_motion_gate_handles_tick_wraparound();
    test_bad_frames_are_rejected();
    test_relative_move_is_explicit_and_bounded();
    test_limited_speed_command_matches_x42s_wire_format();
    test_window_outputs_map_to_bounded_uart_frames();
    test_limited_speed_ack_is_accepted();
    test_can_motion_requires_calibration_and_fresh_motor_feedback();
    test_local_motor_stop_does_not_restart_repeated_can_motion();

    assert(zdt_uart_build_stop(1U, stop) == 5U);
    assert(stop[0] == 0x01U && stop[1] == 0xFEU && stop[2] == 0x98U &&
           stop[3] == 0x00U && stop[4] == 0x6BU);

    puts("ZDT UART protocol checks passed");
    return 0;
}
