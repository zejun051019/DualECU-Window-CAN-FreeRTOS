#include "motor_pulse_train.h"

#include <stddef.h>

void motor_pulse_train_init(motor_pulse_train_t *train)
{
    if (train == NULL)
    {
        return;
    }

    train->requested_pulses = 0U;
    train->rising_edges = 0U;
    train->direction_setup_ticks = 0U;
    train->phase_ticks = 0U;
    train->direction = false;
    train->pulse_active = false;
    train->active = false;
    train->stop_after_fall = false;
}

bool motor_pulse_train_start(
    motor_pulse_train_t *train, bool direction, uint32_t pulse_count)
{
    if ((train == NULL) || (pulse_count == 0U) ||
        (pulse_count > MOTOR_PULSE_TRAIN_MAX_PULSES) || train->active)
    {
        return false;
    }

    train->requested_pulses = pulse_count;
    train->rising_edges = 0U;
    train->direction_setup_ticks =
        MOTOR_PULSE_TRAIN_DIRECTION_SETUP_TICKS;
    train->phase_ticks = 0U;
    train->direction = direction;
    train->pulse_active = false;
    train->active = true;
    train->stop_after_fall = false;
    return true;
}

void motor_pulse_train_tick_1ms(motor_pulse_train_t *train)
{
    if ((train == NULL) || !train->active)
    {
        return;
    }

    if (train->direction_setup_ticks > 0U)
    {
        train->direction_setup_ticks--;
        if (train->direction_setup_ticks == 0U)
        {
            train->phase_ticks = MOTOR_PULSE_TRAIN_HALF_PERIOD_TICKS;
        }
        return;
    }

    if (train->phase_ticks > 0U)
    {
        train->phase_ticks--;
        if (train->phase_ticks > 0U)
        {
            return;
        }
    }

    if (!train->pulse_active)
    {
        train->pulse_active = true;
        train->rising_edges++;
        train->stop_after_fall =
            (train->rising_edges >= train->requested_pulses);
        train->phase_ticks = MOTOR_PULSE_TRAIN_HALF_PERIOD_TICKS;
    }
    else
    {
        train->pulse_active = false;
        if (train->stop_after_fall)
        {
            train->active = false;
        }
        else
        {
            train->phase_ticks = MOTOR_PULSE_TRAIN_HALF_PERIOD_TICKS;
        }
    }
}

void motor_pulse_train_stop(motor_pulse_train_t *train)
{
    if (train == NULL)
    {
        return;
    }

    train->active = false;
    train->pulse_active = false;
    train->direction_setup_ticks = 0U;
    train->phase_ticks = 0U;
    train->stop_after_fall = false;
}

bool motor_pulse_train_is_active(const motor_pulse_train_t *train)
{
    return (train != NULL) && train->active;
}

bool motor_pulse_train_is_pulse_active(const motor_pulse_train_t *train)
{
    return (train != NULL) && train->pulse_active;
}

bool motor_pulse_train_get_direction(const motor_pulse_train_t *train)
{
    return (train != NULL) && train->direction;
}

uint32_t motor_pulse_train_get_rising_edge_count(
    const motor_pulse_train_t *train)
{
    return (train != NULL) ? train->rising_edges : 0U;
}
