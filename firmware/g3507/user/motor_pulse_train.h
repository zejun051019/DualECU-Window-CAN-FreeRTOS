#ifndef MOTOR_PULSE_TRAIN_H
#define MOTOR_PULSE_TRAIN_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PULSE_TRAIN_MAX_PULSES (5U)
#define MOTOR_PULSE_TRAIN_DIRECTION_SETUP_TICKS (10U)
#define MOTOR_PULSE_TRAIN_HALF_PERIOD_TICKS (5U)

typedef struct
{
    uint32_t requested_pulses;
    uint32_t rising_edges;
    uint32_t direction_setup_ticks;
    uint32_t phase_ticks;
    bool direction;
    bool pulse_active;
    bool active;
    bool stop_after_fall;
} motor_pulse_train_t;

void motor_pulse_train_init(motor_pulse_train_t *train);
bool motor_pulse_train_start(
    motor_pulse_train_t *train, bool direction, uint32_t pulse_count);
void motor_pulse_train_tick_1ms(motor_pulse_train_t *train);
void motor_pulse_train_stop(motor_pulse_train_t *train);
bool motor_pulse_train_is_active(const motor_pulse_train_t *train);
bool motor_pulse_train_is_pulse_active(const motor_pulse_train_t *train);
bool motor_pulse_train_get_direction(const motor_pulse_train_t *train);
uint32_t motor_pulse_train_get_rising_edge_count(
    const motor_pulse_train_t *train);

#endif /* MOTOR_PULSE_TRAIN_H */
