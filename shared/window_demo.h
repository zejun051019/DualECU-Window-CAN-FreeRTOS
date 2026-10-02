#ifndef SHARED_WINDOW_DEMO_H
#define SHARED_WINDOW_DEMO_H

#include "motor_range.h"

#define WINDOW_DEMO_SPEED_TENTHS_RPM (10U) /* 1 rpm, in 0.1 rpm units */

typedef enum
{
    WINDOW_DEMO_STEP_IDLE = 0U,
    WINDOW_DEMO_STEP_RUNNING,
    WINDOW_DEMO_STEP_COMPLETE,
    WINDOW_DEMO_STEP_FAILED
} window_demo_step_t;

typedef enum
{
    WINDOW_DEMO_ACTION_NONE = 0U,
    WINDOW_DEMO_ACTION_STEP_UP,
    WINDOW_DEMO_ACTION_STEP_DOWN,
    WINDOW_DEMO_ACTION_COMPLETE,
    WINDOW_DEMO_ACTION_FAULT
} window_demo_action_t;

typedef struct
{
    uint32_t started_ms;
    uint16_t speed_tenths_rpm;
    bool active;
    bool direction_up;
    bool waiting_step;
} window_demo_t;

bool window_demo_begin(window_demo_t *demo, const motor_range_t *range,
                       int32_t position_tenths, uint32_t now_ms);
window_demo_action_t window_demo_update(window_demo_t *demo,
                                        const motor_range_t *range,
                                        int32_t position_tenths,
                                        window_demo_step_t step,
                                        bool feedback_ready,
                                        uint32_t now_ms);
void window_demo_cancel(window_demo_t *demo);

#endif
