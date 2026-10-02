#include "motor_range.h"
#include <stddef.h>

#define MOTOR_RANGE_ENDPOINT_TOLERANCE_TENTHS (8)

bool motor_range_set(motor_range_t *range, int32_t minimum,
                     int32_t maximum, int32_t current)
{
    if ((range == NULL) || (minimum >= maximum) ||
        (current < minimum) || (current > maximum))
    {
        return false;
    }
    *range = (motor_range_t){minimum, maximum, true};
    return true;
}
void motor_range_invalidate(motor_range_t *range)
{
    if (range != NULL) { range->valid = false; }
}
bool motor_range_position(const motor_range_t *range, int32_t current,
                          uint16_t *position)
{
    int64_t clamped_current;
    if ((range == NULL) || !range->valid || (position == NULL) ||
        ((int64_t)current <
         ((int64_t)range->minimum - MOTOR_RANGE_ENDPOINT_TOLERANCE_TENTHS)) ||
        ((int64_t)current >
         ((int64_t)range->maximum + MOTOR_RANGE_ENDPOINT_TOLERANCE_TENTHS)))
    {
        return false;
    }

    clamped_current = current;
    if (clamped_current < range->minimum)
    {
        clamped_current = range->minimum;
    }
    else if (clamped_current > range->maximum)
    {
        clamped_current = range->maximum;
    }

    *position = (uint16_t)((clamped_current - range->minimum) * 1000 /
                          ((int64_t)range->maximum - range->minimum));
    return true;
}
int32_t motor_range_delta(const motor_range_t *range, int32_t current,
                         bool positive, uint32_t maximum_step)
{
    uint16_t unused;
    int64_t remaining;
    if (!motor_range_position(range, current, &unused) ||
        (maximum_step > INT32_MAX))
    {
        return 0;
    }
    remaining = positive ? ((int64_t)range->maximum - current)
                         : ((int64_t)current - range->minimum);
    /* Within one 0.1 degree feedback quantum, treat it as the software end. */
    if (remaining <= 1) { return 0; }
    if (remaining > maximum_step) { remaining = maximum_step; }
    return positive ? (int32_t)remaining : -(int32_t)remaining;
}
