#include "motor_port_mspm0.h"

#if defined(DUALECU_ENABLE_MOTOR_STEP_TEST)

#include "motor_pulse_train.h"
#include "ti_msp_dl_config.h"

volatile uint32_t g_motor_step_test_request = MOTOR_STEP_TEST_REQUEST_NONE;
volatile uint32_t g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_IDLE;
volatile uint32_t g_motor_step_test_rising_edges = 0U;
volatile uint32_t g_motor_step_test_active = 0U;

static motor_pulse_train_t s_pulseTrain;
static bool s_previousStepPinLevel;
static bool s_previousDirection;

void motor_port_mspm0_init(void)
{
    motor_pulse_train_init(&s_pulseTrain);
    g_motor_step_test_request = MOTOR_STEP_TEST_REQUEST_NONE;
    g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_IDLE;
    g_motor_step_test_rising_edges = 0U;
    g_motor_step_test_active = 0U;
    s_previousStepPinLevel = false;
    s_previousDirection = false;

    /* SysConfig also initializes both pins low before enabling their outputs. */
    DL_GPIO_clearPins(GPIOB, GPIO_MOTOR_MOTOR_STEP_SAFE_PIN);
    DL_GPIO_clearPins(GPIOA, GPIO_MOTOR_MOTOR_DIR_PIN);
}

static void motorPortMspm0ApplyOutputs(void)
{
    const bool pulseActive =
        motor_pulse_train_is_pulse_active(&s_pulseTrain);
    const bool direction = motor_pulse_train_get_direction(&s_pulseTrain);

    if (pulseActive != s_previousStepPinLevel)
    {
        if (pulseActive)
        {
            DL_GPIO_setPins(GPIOB, GPIO_MOTOR_MOTOR_STEP_SAFE_PIN);
        }
        else
        {
            DL_GPIO_clearPins(GPIOB, GPIO_MOTOR_MOTOR_STEP_SAFE_PIN);
        }
        s_previousStepPinLevel = pulseActive;
    }

    if (direction != s_previousDirection)
    {
        if (direction)
        {
            DL_GPIO_setPins(GPIOA, GPIO_MOTOR_MOTOR_DIR_PIN);
        }
        else
        {
            DL_GPIO_clearPins(GPIOA, GPIO_MOTOR_MOTOR_DIR_PIN);
        }
        s_previousDirection = direction;
    }
}

void motor_port_mspm0_tick_1ms(void)
{
    const uint32_t request = g_motor_step_test_request;

    if (request != MOTOR_STEP_TEST_REQUEST_NONE)
    {
        g_motor_step_test_request = MOTOR_STEP_TEST_REQUEST_NONE;

        if (request == MOTOR_STEP_TEST_REQUEST_STOP)
        {
            motor_pulse_train_stop(&s_pulseTrain);
            g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_STOPPED;
        }
        else if (motor_pulse_train_is_active(&s_pulseTrain))
        {
            g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_REJECTED;
        }
        else if ((request == MOTOR_STEP_TEST_REQUEST_DIR_LOW) ||
                 (request == MOTOR_STEP_TEST_REQUEST_DIR_HIGH))
        {
            const bool direction =
                (request == MOTOR_STEP_TEST_REQUEST_DIR_HIGH);

            if (motor_pulse_train_start(
                    &s_pulseTrain, direction, MOTOR_PULSE_TRAIN_MAX_PULSES))
            {
                g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_RUNNING;
            }
            else
            {
                g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_REJECTED;
            }
        }
        else
        {
            g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_REJECTED;
        }
    }

    motor_pulse_train_tick_1ms(&s_pulseTrain);
    motorPortMspm0ApplyOutputs();
    g_motor_step_test_rising_edges =
        motor_pulse_train_get_rising_edge_count(&s_pulseTrain);
    g_motor_step_test_active =
        motor_pulse_train_is_active(&s_pulseTrain) ? 1U : 0U;

    if ((g_motor_step_test_result == MOTOR_STEP_TEST_RESULT_RUNNING) &&
        (g_motor_step_test_active == 0U))
    {
        g_motor_step_test_result = MOTOR_STEP_TEST_RESULT_COMPLETE;
    }
}

#endif /* DUALECU_ENABLE_MOTOR_STEP_TEST */
