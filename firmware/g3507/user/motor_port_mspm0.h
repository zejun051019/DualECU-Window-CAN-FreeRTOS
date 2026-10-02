#ifndef MOTOR_PORT_MSPM0_H
#define MOTOR_PORT_MSPM0_H

#include <stdint.h>

enum
{
    MOTOR_STEP_TEST_REQUEST_NONE = 0U,
    MOTOR_STEP_TEST_REQUEST_DIR_LOW = 1U,
    MOTOR_STEP_TEST_REQUEST_DIR_HIGH = 2U,
    MOTOR_STEP_TEST_REQUEST_STOP = 3U
};

enum
{
    MOTOR_STEP_TEST_RESULT_IDLE = 0U,
    MOTOR_STEP_TEST_RESULT_RUNNING,
    MOTOR_STEP_TEST_RESULT_COMPLETE,
    MOTOR_STEP_TEST_RESULT_REJECTED,
    MOTOR_STEP_TEST_RESULT_STOPPED
};

#if defined(DUALECU_ENABLE_MOTOR_STEP_TEST)
extern volatile uint32_t g_motor_step_test_request;
extern volatile uint32_t g_motor_step_test_result;
extern volatile uint32_t g_motor_step_test_rising_edges;
extern volatile uint32_t g_motor_step_test_active;

void motor_port_mspm0_init(void);
void motor_port_mspm0_tick_1ms(void);
#endif

#endif /* MOTOR_PORT_MSPM0_H */
