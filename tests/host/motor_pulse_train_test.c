#include "../../firmware/g3507/user/motor_pulse_train.h"

#include <stdio.h>

static int fail(const char *message)
{
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}

int main(void)
{
    motor_pulse_train_t train;
    uint32_t tick;
    uint32_t simulated_ms = 0U;
    uint32_t observed_edges = 0U;
    uint32_t last_edge_ms = 0U;
    uint32_t pulse_high_ms = 0U;
    bool previous_active;

    motor_pulse_train_init(&train);
    for (tick = 0U; tick < 1000U; tick++)
    {
        motor_pulse_train_tick_1ms(&train);
    }
    if (motor_pulse_train_is_active(&train) ||
        motor_pulse_train_is_pulse_active(&train) ||
        (motor_pulse_train_get_rising_edge_count(&train) != 0U))
    {
        return fail("reset/idle must never produce a pulse");
    }

    if (motor_pulse_train_start(&train, true, 0U) ||
        motor_pulse_train_start(
            &train, true, MOTOR_PULSE_TRAIN_MAX_PULSES + 1U) ||
        motor_pulse_train_is_active(&train))
    {
        return fail("zero and over-limit pulse requests must be rejected");
    }

    if (!motor_pulse_train_start(&train, true, 5U))
    {
        return fail("valid bounded pulse request must start");
    }
    if (motor_pulse_train_start(&train, false, 1U))
    {
        return fail("a second motion request must not replace an active train");
    }
    if (!motor_pulse_train_get_direction(&train) ||
        motor_pulse_train_is_pulse_active(&train))
    {
        return fail("direction must be set while STEP remains idle");
    }

    for (tick = 0U; tick < 14U; tick++)
    {
        motor_pulse_train_tick_1ms(&train);
        simulated_ms++;
        if (motor_pulse_train_is_pulse_active(&train))
        {
            return fail("first STEP edge must wait for direction setup and low phase");
        }
    }
    motor_pulse_train_tick_1ms(&train);
    simulated_ms++;
    if (!motor_pulse_train_is_pulse_active(&train))
    {
        return fail("first STEP edge must occur after the configured setup delay");
    }
    observed_edges = 1U;
    last_edge_ms = simulated_ms;
    pulse_high_ms++;

    for (tick = 0U; tick < 300U; tick++)
    {
        previous_active = motor_pulse_train_is_pulse_active(&train);
        motor_pulse_train_tick_1ms(&train);
        simulated_ms++;
        if (!previous_active && motor_pulse_train_is_pulse_active(&train))
        {
            observed_edges++;
            if ((simulated_ms - last_edge_ms) !=
                (2U * MOTOR_PULSE_TRAIN_HALF_PERIOD_TICKS))
            {
                return fail("STEP rising edges must use the configured period");
            }
            last_edge_ms = simulated_ms;
        }
        if (motor_pulse_train_is_pulse_active(&train))
        {
            pulse_high_ms++;
        }
        if (!motor_pulse_train_is_active(&train))
        {
            break;
        }
    }
    if (motor_pulse_train_is_active(&train) ||
        motor_pulse_train_is_pulse_active(&train) ||
        (observed_edges != 5U) ||
        (motor_pulse_train_get_rising_edge_count(&train) != 5U) ||
        (pulse_high_ms !=
         (5U * MOTOR_PULSE_TRAIN_HALF_PERIOD_TICKS)))
    {
        return fail("request must output five timed pulses then finish low");
    }

    if (!motor_pulse_train_start(&train, false, 3U))
    {
        return fail("a later explicit request must be allowed");
    }
    for (tick = 0U; tick < 100U; tick++)
    {
        motor_pulse_train_tick_1ms(&train);
        if (motor_pulse_train_is_pulse_active(&train))
        {
            break;
        }
    }
    if (!motor_pulse_train_is_pulse_active(&train) ||
        motor_pulse_train_get_direction(&train))
    {
        return fail("second direction must wait for an explicit pulse request");
    }
    motor_pulse_train_stop(&train);
    if (motor_pulse_train_is_active(&train) ||
        motor_pulse_train_is_pulse_active(&train))
    {
        return fail("stop must force STEP inactive immediately");
    }
    for (tick = 0U; tick < 100U; tick++)
    {
        motor_pulse_train_tick_1ms(&train);
    }
    if (motor_pulse_train_is_pulse_active(&train) ||
        (motor_pulse_train_get_rising_edge_count(&train) != 1U))
    {
        return fail("stop must prevent additional pulses");
    }

    (void)puts("motor pulse train checks passed");
    return 0;
}
