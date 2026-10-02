#ifndef APP_BUTTON_H
#define APP_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#define APP_BUTTON_ZERO_CONFIRM_TIMEOUT_MS (1500U)

typedef enum
{
    APP_BUTTON_NONE = 0U,
    APP_BUTTON_SET_ZERO,
    APP_BUTTON_TOGGLE,
    APP_BUTTON_STOP
} app_button_event_t;

typedef struct
{
    bool status_fresh;
    bool remote_motor_fault;
    bool remote_stopped;
    bool stop_confirmed;
    bool remote_motor_stop_latched;
    bool rx_overflow;
    bool remote_offline;
} app_button_recovery_context_t;

typedef struct
{
    uint32_t raw_changed_ms;
    uint32_t pressed_ms;
    bool raw_pressed;
    bool stable_pressed;
    bool wait_for_release;
    bool press_consumed;
} app_button_t;

typedef struct
{
    uint32_t armed_at_ms;
    uint8_t set_zero_sequence;
    bool armed;
} app_button_deferred_toggle_t;

void app_button_init(app_button_t *button, bool raw_pressed, uint32_t now_ms);
app_button_event_t app_button_update(app_button_t *button, bool raw_pressed,
                                     bool motion_pending, uint32_t now_ms);
bool app_button_requests_motor_recovery(
    app_button_event_t event, const app_button_recovery_context_t *context);
void app_button_deferred_toggle_arm(app_button_deferred_toggle_t *toggle,
                                    uint8_t set_zero_sequence,
                                    uint32_t now_ms);
void app_button_deferred_toggle_cancel(app_button_deferred_toggle_t *toggle);
bool app_button_deferred_toggle_take(app_button_deferred_toggle_t *toggle,
                                     bool safe, bool calibrated,
                                     uint8_t confirmed_sequence,
                                     uint32_t now_ms);

#endif
