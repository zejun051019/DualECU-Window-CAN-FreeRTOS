#include "../../shared/window_demo.h"
#include "../../shared/zdt_uart_protocol.h"

#include <assert.h>
#include <stdio.h>

static void test_up_steps_require_completion(void)
{
    motor_range_t range = {0};
    window_demo_t demo = {0};
    assert(motor_range_set(&range, 100, 1000, 100));
    assert(window_demo_begin(&demo, &range, 100, 0U));
    assert(window_demo_update(&demo, &range, 100, WINDOW_DEMO_STEP_IDLE,
                              true, 0U) == WINDOW_DEMO_ACTION_STEP_UP);
    assert(window_demo_update(&demo, &range, 104, WINDOW_DEMO_STEP_RUNNING,
                              true, 100U) == WINDOW_DEMO_ACTION_NONE);
    assert(window_demo_update(&demo, &range, 105, WINDOW_DEMO_STEP_COMPLETE,
                              true, 200U) == WINDOW_DEMO_ACTION_STEP_UP);
    assert(window_demo_update(&demo, &range, 995, WINDOW_DEMO_STEP_COMPLETE,
                              true, 15000U) == WINDOW_DEMO_ACTION_COMPLETE);
    assert(!demo.active);

    assert(window_demo_begin(&demo, &range, 100, 16000U));
    assert(window_demo_update(&demo, &range, 100, WINDOW_DEMO_STEP_IDLE,
                              true, 16000U) == WINDOW_DEMO_ACTION_STEP_UP);
    assert(window_demo_update(&demo, &range, 1004, WINDOW_DEMO_STEP_COMPLETE,
                              true, 16100U) == WINDOW_DEMO_ACTION_COMPLETE);
}

static void test_down_stop_timeout_and_wrap(void)
{
    motor_range_t range = {0};
    window_demo_t demo = {0};
    assert(motor_range_set(&range, 100, 1000, 500));
    assert(window_demo_begin(&demo, &range, 500, UINT32_MAX - 20U));
    assert(window_demo_update(&demo, &range, 500, WINDOW_DEMO_STEP_IDLE,
                              true, UINT32_MAX - 20U) == WINDOW_DEMO_ACTION_STEP_DOWN);
    assert(window_demo_update(&demo, &range, 495, WINDOW_DEMO_STEP_RUNNING,
                              true, 20U) == WINDOW_DEMO_ACTION_NONE);
    window_demo_cancel(&demo);
    assert(window_demo_update(&demo, &range, 495, WINDOW_DEMO_STEP_COMPLETE,
                              true, 30U) == WINDOW_DEMO_ACTION_NONE);
    assert(window_demo_begin(&demo, &range, 500, 100U));
    assert(window_demo_update(&demo, &range, 500, WINDOW_DEMO_STEP_IDLE,
                              true, 60101U) == WINDOW_DEMO_ACTION_FAULT);
    assert(!demo.active);
}

static void test_invalid_feedback_and_endpoint(void)
{
    motor_range_t range = {0};
    window_demo_t demo = {0};
    assert(!window_demo_begin(&demo, &range, 100, 0U));
    assert(motor_range_set(&range, 100, 1000, 1000));
    assert(window_demo_begin(&demo, &range, 1000, 0U));
    assert(window_demo_update(&demo, &range, 1000, WINDOW_DEMO_STEP_IDLE,
                              true, 0U) == WINDOW_DEMO_ACTION_STEP_DOWN);
    assert(window_demo_update(&demo, &range, 995, WINDOW_DEMO_STEP_FAILED,
                              true, 100U) == WINDOW_DEMO_ACTION_FAULT);
    assert(window_demo_begin(&demo, &range, 1000, 200U));
    assert(window_demo_update(&demo, &range, 1000, WINDOW_DEMO_STEP_IDLE,
                              false, 200U) == WINDOW_DEMO_ACTION_FAULT);
    assert(window_demo_begin(&demo, &range, 1000, 300U));
    assert(window_demo_update(&demo, &range, 91, WINDOW_DEMO_STEP_IDLE,
                              true, 300U) == WINDOW_DEMO_ACTION_FAULT);
    assert(window_demo_begin(&demo, &range, 1000, 400U));
    assert(window_demo_update(&demo, &range, 1000, WINDOW_DEMO_STEP_IDLE,
                              true, 400U) == WINDOW_DEMO_ACTION_STEP_DOWN);
    assert(window_demo_update(&demo, &range, 107, WINDOW_DEMO_STEP_COMPLETE,
                              true, 15000U) == WINDOW_DEMO_ACTION_COMPLETE);
}

static void test_demo_uses_planned_one_rpm_profile(void)
{
    motor_range_t range = {0};
    window_demo_t demo = {0};
    uint8_t frame[ZDT_UART_FRAME_MAX_LEN] = {0};

    assert(motor_range_set(&range, 0, 900, 0));
    assert(window_demo_begin(&demo, &range, 0, 0U));
    assert(demo.speed_tenths_rpm == WINDOW_DEMO_SPEED_TENTHS_RPM);
    assert(demo.speed_tenths_rpm == 10U);
    assert(zdt_uart_build_relative_move(1U, 5, 1000U, 1000U,
                                       demo.speed_tenths_rpm, 500U, frame));
    assert(frame[7] == 0U);
    assert(frame[8] == 10U);
}

int main(void)
{
    test_up_steps_require_completion();
    test_down_stop_timeout_and_wrap();
    test_invalid_feedback_and_endpoint();
    test_demo_uses_planned_one_rpm_profile();
    puts("window demo planner PASS");
    return 0;
}
