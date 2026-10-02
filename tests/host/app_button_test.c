#include "../../firmware/f407/App/Inc/app_button.h"

#include <assert.h>
#include <stdio.h>

static void test_short_long_and_stop(void)
{
    app_button_t button;
    app_button_init(&button, false, 0U);
    assert(app_button_update(&button, true, false, 10U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 20U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, 50U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, 80U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 100U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 130U) == APP_BUTTON_TOGGLE);
    assert(app_button_update(&button, false, false, 160U) == APP_BUTTON_NONE);

    assert(app_button_update(&button, true, false, 200U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, 230U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, 2199U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, 2200U) == APP_BUTTON_SET_ZERO);
    assert(app_button_update(&button, true, false, 2300U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 2310U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 2340U) == APP_BUTTON_NONE);

    assert(app_button_update(&button, true, true, 2400U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, true, 2430U) == APP_BUTTON_STOP);
    assert(app_button_update(&button, true, true, 4500U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 4510U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 4540U) == APP_BUTTON_NONE);
}

static void test_held_at_boot_and_tick_wrap(void)
{
    app_button_t button;
    app_button_init(&button, true, 0U);
    assert(app_button_update(&button, true, false, 3000U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 3010U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 3040U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, UINT32_MAX - 60U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, UINT32_MAX - 30U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 0U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 30U) == APP_BUTTON_TOGGLE);
}

static void test_long_press_released_at_threshold(void)
{
    app_button_t button;
    app_button_init(&button, false, 0U);
    assert(app_button_update(&button, true, false, 200U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, 230U) == APP_BUTTON_NONE);

    /* Press began at 200 ms; release begins exactly 2,000 ms later. */
    assert(app_button_update(&button, false, false, 2200U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 2230U) == APP_BUTTON_SET_ZERO);
}

static void test_press_just_below_threshold_remains_short(void)
{
    app_button_t button;
    app_button_init(&button, false, 0U);
    assert(app_button_update(&button, true, false, 200U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, true, false, 230U) == APP_BUTTON_NONE);

    /* 1,999 ms is still a short press after release debounce. */
    assert(app_button_update(&button, false, false, 2199U) == APP_BUTTON_NONE);
    assert(app_button_update(&button, false, false, 2229U) == APP_BUTTON_TOGGLE);
}

static void test_long_press_requires_explicit_motor_recovery_conditions(void)
{
    app_button_recovery_context_t context = {
        .status_fresh = true,
        .remote_motor_fault = true,
        .remote_stopped = true,
        .stop_confirmed = true,
        .remote_motor_stop_latched = true,
        .rx_overflow = false,
        .remote_offline = false
    };

    assert(app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
    assert(!app_button_requests_motor_recovery(APP_BUTTON_TOGGLE, &context));
    context.status_fresh = false;
    assert(!app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
    context.status_fresh = true;
    context.stop_confirmed = false;
    assert(!app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
    context.stop_confirmed = true;
    context.rx_overflow = true;
    assert(!app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
}

static void test_immediate_short_press_waits_for_confirmed_zero(void)
{
    app_button_deferred_toggle_t toggle = {0};
    app_button_deferred_toggle_arm(&toggle, 0xFEU, UINT32_MAX - 100U);
    assert(toggle.armed);
    assert(!app_button_deferred_toggle_take(&toggle, true, false, 0xFEU, 0U));
    assert(!app_button_deferred_toggle_take(&toggle, true, true, 0xFDU, 10U));
    assert(app_button_deferred_toggle_take(&toggle, true, true, 0xFEU, 20U));
    assert(!app_button_deferred_toggle_take(&toggle, true, true, 0xFEU, 30U));
}

static void test_long_press_recovers_communication_latch_but_never_short_press(void)
{
    app_button_recovery_context_t context = {
        .status_fresh = true,
        .remote_motor_fault = false,
        .remote_stopped = true,
        .stop_confirmed = true,
        .remote_motor_stop_latched = false,
        .communication_stop_latched = true
    };
    assert(app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
    assert(!app_button_requests_motor_recovery(APP_BUTTON_TOGGLE, &context));
    context.remote_offline = true;
    assert(!app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
    context.remote_offline = false;
    context.remote_stopped = false;
    assert(!app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
    context.remote_stopped = true;
    context.rx_overflow = true;
    assert(!app_button_requests_motor_recovery(APP_BUTTON_SET_ZERO, &context));
}

static void test_deferred_short_press_drops_on_fault_or_timeout(void)
{
    app_button_deferred_toggle_t toggle = {0};
    app_button_deferred_toggle_arm(&toggle, 4U, 100U);
    assert(!app_button_deferred_toggle_take(&toggle, false, true, 4U, 200U));
    assert(!toggle.armed);
    assert(!app_button_deferred_toggle_take(&toggle, true, true, 4U, 300U));

    app_button_deferred_toggle_arm(&toggle, 5U, 100U);
    assert(!app_button_deferred_toggle_take(&toggle, true, true, 5U, 1601U));
    assert(!toggle.armed);
}

int main(void)
{
    test_long_press_recovers_communication_latch_but_never_short_press();
    test_short_long_and_stop();
    test_held_at_boot_and_tick_wrap();
    test_long_press_released_at_threshold();
    test_press_just_below_threshold_remains_short();
    test_long_press_requires_explicit_motor_recovery_conditions();
    test_immediate_short_press_waits_for_confirmed_zero();
    test_deferred_short_press_drops_on_fault_or_timeout();
    puts("F407 button events PASS");
    return 0;
}
