#include "window_demo.h"

#include <stddef.h>

#define WINDOW_DEMO_ENDPOINT_TOLERANCE_TENTHS (8)
#define WINDOW_DEMO_STEP_MAX_TENTHS (5U)
#define WINDOW_DEMO_TOTAL_TIMEOUT_MS (60000U)

void window_demo_cancel(window_demo_t *demo)
{
    if (demo != NULL)
    {
        *demo = (window_demo_t){0};
    }
}

bool window_demo_begin(window_demo_t *demo, const motor_range_t *range,
                       int32_t position_tenths, uint32_t now_ms)
{
    uint16_t unused;
    if ((demo == NULL) || !motor_range_position(range, position_tenths, &unused))
    {
        return false;
    }
    *demo = (window_demo_t){0};
    demo->active = true;
    demo->direction_up =
        ((int64_t)position_tenths - range->minimum) <=
        WINDOW_DEMO_ENDPOINT_TOLERANCE_TENTHS;
    demo->started_ms = now_ms;
    demo->speed_tenths_rpm = WINDOW_DEMO_SPEED_TENTHS_RPM;
    return true;
}

window_demo_action_t window_demo_update(window_demo_t *demo,
                                        const motor_range_t *range,
                                        int32_t position_tenths,
                                        window_demo_step_t step,
                                        bool feedback_ready,
                                        uint32_t now_ms)
{
    uint16_t unused;
    int64_t remaining;
    if ((demo == NULL) || !demo->active)
    {
        return WINDOW_DEMO_ACTION_NONE;
    }
    if (!feedback_ready ||
        !motor_range_position(range, position_tenths, &unused) ||
        (step == WINDOW_DEMO_STEP_FAILED) ||
        ((uint32_t)(now_ms - demo->started_ms) > WINDOW_DEMO_TOTAL_TIMEOUT_MS))
    {
        window_demo_cancel(demo);
        return WINDOW_DEMO_ACTION_FAULT;
    }
    if (demo->waiting_step)
    {
        if (step != WINDOW_DEMO_STEP_COMPLETE)
        {
            return WINDOW_DEMO_ACTION_NONE;
        }
        demo->waiting_step = false;
    }
    remaining = demo->direction_up ?
        ((int64_t)range->maximum - position_tenths) :
        ((int64_t)position_tenths - range->minimum);
    if (remaining <= WINDOW_DEMO_ENDPOINT_TOLERANCE_TENTHS)
    {
        window_demo_cancel(demo);
        return WINDOW_DEMO_ACTION_COMPLETE;
    }
    if (motor_range_delta(range, position_tenths, demo->direction_up,
                          WINDOW_DEMO_STEP_MAX_TENTHS) == 0)
    {
        window_demo_cancel(demo);
        return WINDOW_DEMO_ACTION_FAULT;
    }
    demo->waiting_step = true;
    return demo->direction_up ? WINDOW_DEMO_ACTION_STEP_UP :
                                WINDOW_DEMO_ACTION_STEP_DOWN;
}
