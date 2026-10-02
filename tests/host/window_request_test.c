#include "../../shared/window_request.h"

#include <assert.h>
#include <stdio.h>

static void test_latest_value_and_stop_latch(void)
{
    window_request_queue_t queue = {0};
    window_request_t actual = {0};

    window_request_init(&queue);
    assert(window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_UP,
        .sequence = 10U
    }));
    assert(window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_DOWN,
        .sequence = 11U
    }));
    assert(queue.overwritten_count == 1U);
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_DOWN);
    assert(actual.sequence == 11U);

    assert(window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_STOP,
        .sequence = 12U
    }));
    assert(queue.stop_latched);
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_STOP);
    assert(actual.sequence == 12U);

    assert(!window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_UP,
        .sequence = 13U
    }));
    assert(queue.rejected_while_latched_count == 1U);
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_STOP);
    assert(actual.sequence == 12U);

    assert(!window_request_rearm(&queue));
    assert(window_request_confirm_stop(&queue, 12U));
    assert(!window_request_confirm_stop(&queue, 11U));
    assert(queue.stop_latched);
    assert(window_request_rearm(&queue));
    assert(!queue.stop_latched);
    assert(!window_request_peek(&queue, &actual));
    assert(window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_UP,
        .sequence = 14U
    }));
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_UP);
    assert(actual.sequence == 14U);
}

static void test_stop_cannot_be_cleared_by_clear_command(void)
{
    window_request_queue_t queue = {0};
    window_request_t actual = {0};

    window_request_init(&queue);
    assert(window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_STOP,
        .sequence = 255U
    }));
    assert(!window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_CLEAR,
        .sequence = 0U
    }));
    assert(queue.stop_latched);
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_STOP);
    assert(actual.sequence == 255U);
}

static void test_peer_reset_reopens_confirmed_stop(void)
{
    window_request_queue_t queue = {0};
    window_request_t actual = {0};

    window_request_init(&queue);
    assert(window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_STOP,
        .sequence = 42U
    }));
    assert(window_request_confirm_stop(&queue, 42U));
    assert(queue.stop_acknowledged);
    /* STOP confirmation does not end the command watchdog heartbeat. */
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_STOP);
    assert(actual.sequence == 42U);

    assert(window_request_revoke_stop_confirmation(&queue));
    assert(queue.stop_latched);
    assert(!queue.stop_acknowledged);
    assert(!window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_UP,
        .sequence = 43U
    }));
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_STOP);
    assert(actual.sequence == 42U);
    assert(!window_request_revoke_stop_confirmation(&queue));
}

static void test_demo_request_is_latest_value_and_stop_wins(void)
{
    window_request_queue_t queue = {0};
    window_request_t actual = {0};

    window_request_init(&queue);
    assert(window_request_submit(&queue, (window_request_t){
        .command = (window_request_command_t)5U, .sequence = 50U}));
    assert(window_request_submit(&queue, (window_request_t){
        .command = (window_request_command_t)6U, .sequence = 51U}));
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == (window_request_command_t)6U);
    assert(window_request_submit(&queue, (window_request_t){
        .command = WINDOW_REQUEST_STOP, .sequence = 52U}));
    assert(!window_request_submit(&queue, (window_request_t){
        .command = (window_request_command_t)6U, .sequence = 53U}));
    assert(window_request_peek(&queue, &actual));
    assert(actual.command == WINDOW_REQUEST_STOP);
    assert(actual.sequence == 52U);
}

static void test_debug_injection_accepts_only_supported_commands(void)
{
    assert(window_request_command_is_valid(WINDOW_REQUEST_DEMO_SET_ZERO));
    assert(window_request_command_is_valid(WINDOW_REQUEST_DEMO_TOGGLE));
    assert(!window_request_command_is_valid(WINDOW_REQUEST_NONE));
    assert(!window_request_command_is_valid((window_request_command_t)7U));
}

int main(void)
{
    test_latest_value_and_stop_latch();
    test_stop_cannot_be_cleared_by_clear_command();
    test_peer_reset_reopens_confirmed_stop();
    test_demo_request_is_latest_value_and_stop_wins();
    test_debug_injection_accepts_only_supported_commands();
    puts("window request latest-value and STOP latch PASS");
    return 0;
}
