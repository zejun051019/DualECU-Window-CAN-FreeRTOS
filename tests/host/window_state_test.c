#include "../../shared/window_state.h"

#include <assert.h>
#include <stdio.h>

static window_state_rx_result_t receive_frame_now(
    window_state_t *window,
    const can_protocol_frame_t *frame,
    uint32_t now_ms)
{
    return window_state_receive_frame_at(window, frame, now_ms, now_ms);
}

#define window_state_receive_frame(window, frame, now_ms) \
    receive_frame_now((window), (frame), (now_ms))

static can_protocol_frame_t make_command(
    uint8_t sequence,
    can_protocol_command_id_t command_id)
{
    const can_protocol_command_t command = {sequence, command_id};
    can_protocol_frame_t frame = {0};

    assert(can_protocol_encode_command(&command, &frame));
    return frame;
}

static can_protocol_status_t get_status(const window_state_t *window)
{
    can_protocol_status_t status = {0};

    assert(window_state_get_status(window, &status));
    return status;
}

static void assert_status(
    const window_state_t *window,
    can_protocol_state_t expected_state,
    can_protocol_fault_t expected_fault,
    bool expected_sequence_valid,
    uint8_t expected_sequence)
{
    const can_protocol_status_t status = get_status(window);

    assert(status.state == expected_state);
    assert(status.fault == expected_fault);
    assert(status.last_sequence_valid == expected_sequence_valid);
    assert(status.last_sequence == expected_sequence);
    assert(status.position == 0xFFFFU);
    assert(!status.is_calibrated);
}

static void enter_ready(window_state_t *window, uint32_t start_ms, uint8_t seq)
{
    can_protocol_frame_t frame;

    frame = make_command(seq, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(window, &frame, start_ms) ==
           WINDOW_STATE_RX_BASELINE_ACCEPTED);

    frame = make_command((uint8_t)(seq + 1U), CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(window, &frame, start_ms + 1U) ==
           WINDOW_STATE_RX_CLEAR_SUCCEEDED);

    frame = make_command((uint8_t)(seq + 2U), CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(window, &frame, start_ms + 2U) ==
           WINDOW_STATE_RX_ACCEPTED);
}

static void test_startup_and_recovery_gates(void)
{
    window_state_t window;
    can_protocol_frame_t frame;

    window_state_init(&window);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_STARTUP_LOCKED,
        false,
        0U);

    frame = make_command(10U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 10U) ==
           WINDOW_STATE_RX_GATED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_STARTUP_LOCKED,
        false,
        0U);

    frame = make_command(255U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 20U) ==
           WINDOW_STATE_RX_BASELINE_ACCEPTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_STARTUP_LOCKED,
        true,
        255U);

    frame = make_command(0U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 30U) ==
           WINDOW_STATE_RX_GATED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_STARTUP_LOCKED,
        true,
        255U);

    frame = make_command(0U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(&window, &frame, 40U) ==
           WINDOW_STATE_RX_CLEAR_SUCCEEDED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        0U);

    frame = make_command(1U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 50U) ==
           WINDOW_STATE_RX_GATED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        0U);

    frame = make_command(1U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 60U) ==
           WINDOW_STATE_RX_ACCEPTED);
    frame = make_command(2U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 70U) ==
           WINDOW_STATE_RX_ACCEPTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        2U);

    frame = make_command(2U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 80U) ==
           WINDOW_STATE_RX_REPEAT_ACCEPTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        2U);

    frame = make_command(3U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 90U) ==
           WINDOW_STATE_RX_ACCEPTED);
    frame = make_command(2U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 100U) ==
           WINDOW_STATE_RX_REJECTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        3U);
}

static void test_motor_restart_rezero_requires_ready_stationary_recovery(void)
{
    window_state_t window;
    window_state_init(&window);
    assert(!window_state_can_rezero_after_motor_recovery(
        &window, true, true, true));

    enter_ready(&window, 100U, 10U);
    assert(window_state_can_rezero_after_motor_recovery(
        &window, true, true, true));
    assert(!window_state_can_rezero_after_motor_recovery(
        &window, false, true, true));
    assert(!window_state_can_rezero_after_motor_recovery(
        &window, true, false, true));
    assert(!window_state_can_rezero_after_motor_recovery(
        &window, true, true, false));

    window_state_note_motor_fault(&window, 200U);
    assert(!window_state_can_rezero_after_motor_recovery(
        &window, true, true, true));
}

static void test_failed_clear_is_consumed(void)
{
    window_state_t window;
    can_protocol_frame_t frame;
    can_protocol_frame_t old_clear_frame;

    window_state_init(&window);
    frame = make_command(10U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 0U) ==
           WINDOW_STATE_RX_BASELINE_ACCEPTED);

    window_state_note_rx_overflow(&window, 1U);
    assert(get_status(&window).rx_overflow_active);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW,
        false,
        10U);
    assert(window_state_get_rx_overflow_observation_count(&window) == 1U);

    frame = make_command(100U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 2U) ==
           WINDOW_STATE_RX_BASELINE_ACCEPTED);
    frame = make_command(101U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(&window, &frame, 3U) ==
           WINDOW_STATE_RX_CLEAR_FAILED);
    assert(get_status(&window).rx_overflow_active);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW,
        true,
        101U);

    /* A newer STOP is allowed, but it must supersede the failed CLEAR event. */
    frame = make_command(102U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 4U) ==
           WINDOW_STATE_RX_ACCEPTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW,
        true,
        102U);

    window_state_confirm_rx_overflow_cleared(&window);
    assert(!get_status(&window).rx_overflow_active);
    old_clear_frame = make_command(101U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(&window, &old_clear_frame, 5U) ==
           WINDOW_STATE_RX_REJECTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW,
        true,
        102U);

    frame = make_command(103U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(&window, &frame, 6U) ==
           WINDOW_STATE_RX_CLEAR_SUCCEEDED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        103U);

    frame = make_command(104U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 7U) ==
           WINDOW_STATE_RX_GATED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        103U);

    frame = make_command(104U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 8U) ==
           WINDOW_STATE_RX_ACCEPTED);
    frame = make_command(105U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 9U) ==
           WINDOW_STATE_RX_ACCEPTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        105U);
}

static void test_invalid_and_stale_frames_do_not_refresh_timeout(void)
{
    window_state_t window;
    can_protocol_frame_t frame;

    window_state_init(&window);
    enter_ready(&window, 0U, 20U);
    frame = make_command(23U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 100U) ==
           WINDOW_STATE_RX_ACCEPTED);

    frame = make_command(24U, CAN_PROTOCOL_CMD_UP);
    frame.data[3] = 1U;
    assert(window_state_receive_frame(&window, &frame, 200U) ==
           WINDOW_STATE_RX_INVALID_FRAME);

    frame = make_command(22U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 250U) ==
           WINDOW_STATE_RX_REJECTED);

    window_state_tick(&window, 400U);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        23U);
    window_state_tick(&window, 401U);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        false,
        23U);

    frame = make_command(24U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame_at(&window, &frame, 350U, 402U) ==
           WINDOW_STATE_RX_STALE_FRAME);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        false,
        23U);

    frame = make_command(24U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 403U) ==
           WINDOW_STATE_RX_GATED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        false,
        23U);

    frame = make_command(200U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 404U) ==
           WINDOW_STATE_RX_BASELINE_ACCEPTED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        true,
        200U);
}

static void test_timeout_across_u32_wrap_and_repeat_refresh(void)
{
    window_state_t window;
    can_protocol_frame_t frame;
    const uint32_t start_ms = UINT32_MAX - 100U;
    const uint32_t last_command_ms = start_ms + 3U;
    const uint32_t repeat_ms = last_command_ms + 250U;

    window_state_init(&window);
    enter_ready(&window, start_ms, 30U);
    frame = make_command(33U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, last_command_ms) ==
           WINDOW_STATE_RX_ACCEPTED);

    frame = make_command(33U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, repeat_ms) ==
           WINDOW_STATE_RX_REPEAT_ACCEPTED);
    window_state_tick(
        &window,
        repeat_ms + WINDOW_STATE_COMMAND_TIMEOUT_MS);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        33U);
    window_state_tick(
        &window,
        repeat_ms + WINDOW_STATE_COMMAND_TIMEOUT_MS + 1U);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        false,
        33U);
}

static void test_timeout_precedes_clear_and_motion_after_tick_wrap(void)
{
    window_state_t window;
    can_protocol_frame_t frame;
    const uint32_t start_ms = UINT32_MAX - 100U;
    const uint32_t last_command_ms = start_ms + 3U;
    const uint32_t after_timeout_ms =
        last_command_ms + WINDOW_STATE_COMMAND_TIMEOUT_MS + 1U;

    window_state_init(&window);
    enter_ready(&window, start_ms, 30U);
    frame = make_command(33U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, last_command_ms) ==
           WINDOW_STATE_RX_ACCEPTED);

    /* At this one processing time, timeout is already overdue. The CLEAR
     * must see the latched fault because receive_frame_at ticks first. */
    frame = make_command(34U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame_at(
               &window, &frame, after_timeout_ms, after_timeout_ms) ==
           WINDOW_STATE_RX_STALE_FRAME);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        false,
        33U);

    /* A later, genuinely post-lock event is fresh in time but remains
     * gated until the recovery STOP establishes a new sequence baseline. */
    frame = make_command(34U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame_at(
               &window, &frame, after_timeout_ms + 1U,
               after_timeout_ms + 1U) == WINDOW_STATE_RX_GATED);

    frame = make_command(34U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame_at(
               &window, &frame, after_timeout_ms + 1U,
               after_timeout_ms + 1U) ==
           WINDOW_STATE_RX_GATED);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        false,
        33U);
}

static void test_t11_periodic_same_sequence_does_not_prove_sender_liveness(void)
{
    window_state_t window;
    can_protocol_frame_t frame;
    const uint32_t ready_at_ms = UINT32_MAX - 100U;
    const uint32_t first_up_at_ms = ready_at_ms + 31U;
    uint32_t last_repeat_at_ms = first_up_at_ms;

    window_state_init(&window);
    enter_ready(&window, ready_at_ms, 40U);
    frame = make_command(43U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, first_up_at_ms) ==
           WINDOW_STATE_RX_ACCEPTED);

    /* T11: repeated legal traffic can keep communication online while the
     * sender's application-level business state remains frozen. */
    for (uint32_t repeat = 1U; repeat <= 20U; ++repeat)
    {
        last_repeat_at_ms = first_up_at_ms + repeat * 50U;
        assert(window_state_receive_frame(&window, &frame, last_repeat_at_ms) ==
               WINDOW_STATE_RX_REPEAT_ACCEPTED);
        window_state_tick(&window, last_repeat_at_ms);
        assert_status(
            &window,
            CAN_PROTOCOL_STATE_UP,
            CAN_PROTOCOL_FAULT_NONE,
            true,
            43U);
    }

    window_state_tick(
        &window,
        last_repeat_at_ms + WINDOW_STATE_COMMAND_TIMEOUT_MS);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        true,
        43U);
    window_state_tick(
        &window,
        last_repeat_at_ms + WINDOW_STATE_COMMAND_TIMEOUT_MS + 1U);
    assert_status(
        &window,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        false,
        43U);
}

static void test_motor_fault_requires_new_clear_and_new_motion(void)
{
    window_state_t window;
    can_protocol_frame_t frame;
    window_state_init(&window);
    enter_ready(&window, 10U, 20U);
    frame = make_command(23U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 20U) == WINDOW_STATE_RX_ACCEPTED);
    window_state_note_motor_fault(&window, 21U);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    assert(window.fault == CAN_PROTOCOL_FAULT_MOTOR_LOCAL);
    assert(window_state_receive_frame(&window, &frame, 22U) == WINDOW_STATE_RX_GATED);
    frame = make_command(24U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 23U) == WINDOW_STATE_RX_BASELINE_ACCEPTED);
    frame = make_command(25U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(&window, &frame, 24U) == WINDOW_STATE_RX_CLEAR_FAILED);
    window_state_set_motor_fault_active(&window, false);
    assert(window_state_receive_frame(&window, &frame, 25U) == WINDOW_STATE_RX_REPEAT_ACCEPTED);
    assert(window.fault == CAN_PROTOCOL_FAULT_MOTOR_LOCAL);
    frame = make_command(26U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(&window, &frame, 26U) == WINDOW_STATE_RX_CLEAR_SUCCEEDED);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    frame = make_command(27U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 27U) == WINDOW_STATE_RX_ACCEPTED);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
}

static void test_demo_commands_do_not_replay_motion(void)
{
    window_state_t window;
    can_protocol_frame_t frame;
    window_state_init(&window);
    enter_ready(&window, 10U, 10U);

    frame = make_command(13U, (can_protocol_command_id_t)4U);
    assert(window_state_receive_frame(&window, &frame, 20U) == WINDOW_STATE_RX_ACCEPTED);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    assert(window_state_receive_frame(&window, &frame, 21U) == WINDOW_STATE_RX_REPEAT_ACCEPTED);

    frame = make_command(14U, (can_protocol_command_id_t)5U);
    assert(window_state_receive_frame(&window, &frame, 22U) == WINDOW_STATE_RX_ACCEPTED);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    assert(window_state_note_demo_output(&window, CAN_PROTOCOL_STATE_UP));
    assert(window.state == CAN_PROTOCOL_STATE_UP);
    assert(window_state_receive_frame(&window, &frame, 23U) == WINDOW_STATE_RX_REPEAT_ACCEPTED);
    assert(window.state == CAN_PROTOCOL_STATE_UP);
    frame = make_command(15U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 24U) == WINDOW_STATE_RX_ACCEPTED);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    frame = make_command(14U, (can_protocol_command_id_t)5U);
    assert(window_state_receive_frame(&window, &frame, 25U) == WINDOW_STATE_RX_REJECTED);
    assert(!window_state_note_demo_output(&window, CAN_PROTOCOL_STATE_UP));
}

static void test_bus_off_stops_without_clearing_existing_fault(void)
{
    window_state_t window;
    can_protocol_frame_t frame;
    window_state_init(&window);
    enter_ready(&window, 10U, 10U);
    frame = make_command(13U, CAN_PROTOCOL_CMD_UP);
    assert(window_state_receive_frame(&window, &frame, 20U) == WINDOW_STATE_RX_ACCEPTED);
    window_state_note_can_bus_off(&window, 21U);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    assert(window.fault == CAN_PROTOCOL_FAULT_COMM_TIMEOUT);
    assert(!window.last_sequence_valid);
    assert(window.recovery_phase == WINDOW_RECOVERY_WAIT_STOP);
    assert(window_state_receive_frame(&window, &frame, 22U) == WINDOW_STATE_RX_GATED);
    frame = make_command(14U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 23U) == WINDOW_STATE_RX_BASELINE_ACCEPTED);
    frame = make_command(15U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    assert(window_state_receive_frame(&window, &frame, 24U) == WINDOW_STATE_RX_CLEAR_SUCCEEDED);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    frame = make_command(16U, CAN_PROTOCOL_CMD_STOP);
    assert(window_state_receive_frame(&window, &frame, 25U) == WINDOW_STATE_RX_ACCEPTED);
    assert(window.state == CAN_PROTOCOL_STATE_STOP);
    window_state_note_motor_fault(&window, 26U);
    window_state_note_can_bus_off(&window, 27U);
    assert(window.fault == CAN_PROTOCOL_FAULT_MOTOR_LOCAL);
    assert(window.motor_fault_active);
}

int main(void)
{
    test_bus_off_stops_without_clearing_existing_fault();
    test_demo_commands_do_not_replay_motion();
    test_motor_fault_requires_new_clear_and_new_motion();
    test_startup_and_recovery_gates();
    test_motor_restart_rezero_requires_ready_stationary_recovery();
    test_failed_clear_is_consumed();
    test_invalid_and_stale_frames_do_not_refresh_timeout();
    test_timeout_across_u32_wrap_and_repeat_refresh();
    test_timeout_precedes_clear_and_motion_after_tick_wrap();
    test_t11_periodic_same_sequence_does_not_prove_sender_liveness();
    puts("G3507 window state checks passed");
    return 0;
}
