#ifndef SHARED_MOTOR_RANGE_H
#define SHARED_MOTOR_RANGE_H
#include <stdbool.h>
#include <stdint.h>

/* RAM-only software angular range, in 0.1 degree units; no homing claim. */
typedef struct
{
    int32_t minimum;
    int32_t maximum;
    bool valid;
} motor_range_t;

bool motor_range_set(motor_range_t *range, int32_t minimum,
                     int32_t maximum, int32_t current);
void motor_range_invalidate(motor_range_t *range);
/* Feedback up to 0.8 degrees beyond an endpoint is clamped to that endpoint.
 * A requested delta may only move inward while feedback is outside the range. */
bool motor_range_position(const motor_range_t *range, int32_t current,
                          uint16_t *position);
int32_t motor_range_delta(const motor_range_t *range, int32_t current,
                         bool positive, uint32_t maximum_step);
#endif
