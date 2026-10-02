#include "../Inc/app_button.h"

#include <stddef.h>

#define APP_BUTTON_DEBOUNCE_MS (30U)
#define APP_BUTTON_LONG_PRESS_MS (2000U)

void app_button_init(app_button_t *button, bool raw_pressed, uint32_t now_ms)
{
    if (button == NULL)
    {
        return;
    }
    *button = (app_button_t){0};
    button->raw_pressed = raw_pressed;
    button->stable_pressed = raw_pressed;
    button->raw_changed_ms = now_ms;
    button->pressed_ms = now_ms;
    button->wait_for_release = raw_pressed;
}

app_button_event_t app_button_update(app_button_t *button, bool raw_pressed,
                                     bool motion_pending, uint32_t now_ms)
{
    if (button == NULL)
    {
        return APP_BUTTON_NONE;
    }
    if (raw_pressed != button->raw_pressed)
    {
        button->raw_pressed = raw_pressed;
        button->raw_changed_ms = now_ms;
    }
    if ((button->stable_pressed != button->raw_pressed) &&
        ((uint32_t)(now_ms - button->raw_changed_ms) >= APP_BUTTON_DEBOUNCE_MS))
    {
        button->stable_pressed = button->raw_pressed;
        if (button->stable_pressed)
        {
            /* Start timing at the sampled press edge, not after debounce. */
            button->pressed_ms = button->raw_changed_ms;
            button->press_consumed = motion_pending;
            if (motion_pending && !button->wait_for_release)
            {
                return APP_BUTTON_STOP;
            }
        }
        else
        {
            if (button->wait_for_release)
            {
                button->wait_for_release = false;
            }
            else if (!button->press_consumed)
            {
                button->press_consumed = true;
                if ((uint32_t)(button->raw_changed_ms - button->pressed_ms) >=
                    APP_BUTTON_LONG_PRESS_MS)
                {
                    return APP_BUTTON_SET_ZERO;
                }
                return APP_BUTTON_TOGGLE;
            }
        }
    }
    if (button->stable_pressed && button->raw_pressed &&
        !button->wait_for_release &&
        !button->press_consumed &&
        ((uint32_t)(now_ms - button->pressed_ms) >= APP_BUTTON_LONG_PRESS_MS))
    {
        button->press_consumed = true;
        return APP_BUTTON_SET_ZERO;
    }
    return APP_BUTTON_NONE;
}

bool app_button_requests_motor_recovery(
    app_button_event_t event, const app_button_recovery_context_t *context)
{
    return (event == APP_BUTTON_SET_ZERO) && (context != NULL) &&
           context->status_fresh &&
           context->remote_stopped && context->stop_confirmed &&
           ((context->remote_motor_fault && context->remote_motor_stop_latched) ||
            context->communication_stop_latched) && !context->rx_overflow &&
           !context->remote_offline;
}

void app_button_deferred_toggle_arm(app_button_deferred_toggle_t *toggle,
                                    uint8_t set_zero_sequence,
                                    uint32_t now_ms)
{
    if ((toggle == NULL) || toggle->armed)
    {
        return;
    }
    toggle->armed_at_ms = now_ms;
    toggle->set_zero_sequence = set_zero_sequence;
    toggle->armed = true;
}

void app_button_deferred_toggle_cancel(app_button_deferred_toggle_t *toggle)
{
    if (toggle != NULL)
    {
        toggle->armed = false;
    }
}

bool app_button_deferred_toggle_take(app_button_deferred_toggle_t *toggle,
                                     bool safe, bool calibrated,
                                     uint8_t confirmed_sequence,
                                     uint32_t now_ms)
{
    if ((toggle == NULL) || !toggle->armed)
    {
        return false;
    }
    if (!safe ||
        ((uint32_t)(now_ms - toggle->armed_at_ms) >
         APP_BUTTON_ZERO_CONFIRM_TIMEOUT_MS))
    {
        app_button_deferred_toggle_cancel(toggle);
        return false;
    }
    if (!calibrated || (confirmed_sequence != toggle->set_zero_sequence))
    {
        return false;
    }
    app_button_deferred_toggle_cancel(toggle);
    return true;
}
