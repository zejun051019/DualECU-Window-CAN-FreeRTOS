#include "../../shared/motor_range.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main(void)
{
    motor_range_t range = {0};
    uint16_t position = 0;
    assert(!motor_range_position(&range, 0, &position));
    assert(motor_range_delta(&range, 0, true, 5) == 0);
    assert(!motor_range_set(&range, 10, 10, 10));
    assert(!motor_range_set(&range, 10, 0, 5));
    assert(!motor_range_set(&range, 0, 10, 20));
    assert(motor_range_set(&range, -50, 50, 0));
    assert(motor_range_position(&range, 0, &position) && position == 500);
    assert(motor_range_position(&range, -50, &position) && position == 0);
    assert(motor_range_position(&range, 50, &position) && position == 1000);
    assert(motor_range_position(&range, 51, &position) && position == 1000);
    assert(motor_range_position(&range, 58, &position) && position == 1000);
    assert(!motor_range_position(&range, 59, &position));
    assert(motor_range_position(&range, -58, &position) && position == 0);
    assert(!motor_range_position(&range, -59, &position));
    assert(motor_range_delta(&range, 48, true, 5) == 2);
    assert(motor_range_delta(&range, -48, false, 5) == -2);
    assert(motor_range_delta(&range, 50, true, 5) == 0);
    assert(motor_range_delta(&range, -50, false, 5) == 0);
    assert(motor_range_delta(&range, -49, false, 5) == 0);
    assert(motor_range_delta(&range, 49, true, 5) == 0);
    assert(motor_range_delta(&range, 51, true, 5) == 0);
    assert(motor_range_delta(&range, 51, false, 5) == -5);
    assert(motor_range_delta(&range, 58, false, 5) == -5);
    assert(motor_range_delta(&range, 58, true, 5) == 0);
    assert(motor_range_delta(&range, -58, false, 5) == 0);
    assert(motor_range_delta(&range, -58, true, 5) == 5);
    assert(motor_range_delta(&range, 0, true, 0) == 0);
    /* Widen before subtraction/multiplication, including extreme references. */
    assert(motor_range_set(&range, INT32_MIN, INT32_MAX, 0));
    assert(motor_range_position(&range, INT32_MAX, &position) && position == 1000);
    assert(motor_range_delta(&range, INT32_MIN, true, 5) == 5);
    motor_range_invalidate(&range);
    assert(!motor_range_position(&range, 0, &position));
    puts("motor software range checks passed");
    return 0;
}
