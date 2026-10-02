#include "window_state.h"

#include <stddef.h>

static bool window_state_phase_allows_command(
    const window_state_t *window,
    can_protocol_command_id_t command_id)
{
    switch (window->recovery_phase)
    {
        case WINDOW_RECOVERY_WAIT_STOP:
            return (command_id == CAN_PROTOCOL_CMD_STOP);

        case WINDOW_RECOVERY_WAIT_CLEAR:
            return ((command_id == CAN_PROTOCOL_CMD_STOP) ||
                    (command_id == CAN_PROTOCOL_CMD_CLEAR_FAULT));

        case WINDOW_RECOVERY_WAIT_FINAL_STOP:
            return (command_id == CAN_PROTOCOL_CMD_STOP);

        case WINDOW_RECOVERY_READY:
            return true;

        default:
            return false;
    }
}

static bool window_state_frame_time_is_fresh(
    const window_state_t *window,
    uint32_t received_at_ms,
    uint32_t processed_at_ms)
{
    const uint32_t frame_age_ms = (uint32_t)(processed_at_ms - received_at_ms);

    /* Also rejects future timestamps; valid queue age is bounded below 2^31 ms. */
    if (frame_age_ms > WINDOW_STATE_COMMAND_TIMEOUT_MS)
    {
        return false;
    }

    if (window->accepted_command_time_valid)
    {
        const uint32_t age_from_last_accepted_ms =
            (uint32_t)(received_at_ms - window->last_accepted_command_ms);

        if (age_from_last_accepted_ms >= 0x80000000U)
        {
            return false;
        }
    }

    if ((window->recovery_phase == WINDOW_RECOVERY_WAIT_STOP) &&
        window->recovery_entry_time_valid)
    {
        const uint32_t age_from_recovery_entry_ms =
            (uint32_t)(received_at_ms - window->recovery_entry_ms);

        /* Recovery must use a STOP received strictly after the lock was entered. */
        if ((age_from_recovery_entry_ms == 0U) ||
            (age_from_recovery_entry_ms >= 0x80000000U))
        {
            return false;
        }
    }

    return true;
}

static void window_state_accept_event(
    window_state_t *window,
    const can_protocol_command_t *command,
    uint32_t received_at_ms)
{
    window->last_sequence = command->sequence;
    window->last_sequence_valid = true;
    window->last_command = command->command;
    window->last_accepted_command_ms = received_at_ms;
    window->accepted_command_time_valid = true;
}

void window_state_init(window_state_t *window)
{
    if (window == NULL)
    {
        return;
    }

    *window = (window_state_t){0};
    window->state = CAN_PROTOCOL_STATE_STOP;
    window->fault = CAN_PROTOCOL_FAULT_STARTUP_LOCKED;
    window->last_command = CAN_PROTOCOL_CMD_STOP;
    window->recovery_phase = WINDOW_RECOVERY_WAIT_STOP;
}

window_state_rx_result_t window_state_receive_frame_at(
    window_state_t *window,
    const can_protocol_frame_t *frame,
    uint32_t received_at_ms,
    uint32_t processed_at_ms)
{
    can_protocol_command_t command;
    can_protocol_event_result_t event_result;

    if (window == NULL)
    {
        return WINDOW_STATE_RX_INVALID_FRAME;
    }

    window_state_tick(window, processed_at_ms);
    if (!can_protocol_decode_command(frame, &command))
    {
        return WINDOW_STATE_RX_INVALID_FRAME;
    }

    if (!window_state_frame_time_is_fresh(
            window,
            received_at_ms,
            processed_at_ms))
    {
        return WINDOW_STATE_RX_STALE_FRAME;
    }

    if (!window_state_phase_allows_command(window, command.command))
    {
        return WINDOW_STATE_RX_GATED;
    }

    event_result = can_protocol_classify_event(
        &command,
        window->last_sequence_valid,
        window->last_sequence,
        window->last_command);

    switch (event_result)
    {
        case CAN_PROTOCOL_EVENT_BASELINE_STOP:
            window_state_accept_event(window, &command, received_at_ms);
            window->state = CAN_PROTOCOL_STATE_STOP;
            window->recovery_phase = WINDOW_RECOVERY_WAIT_CLEAR;
            return WINDOW_STATE_RX_BASELINE_ACCEPTED;

        case CAN_PROTOCOL_EVENT_REPEAT:
            window->last_accepted_command_ms = received_at_ms;
            window->accepted_command_time_valid = true;
            /* A repeated motion event maintains state but never restarts it. */
            return WINDOW_STATE_RX_REPEAT_ACCEPTED;

        case CAN_PROTOCOL_EVENT_NEW:
            window_state_accept_event(window, &command, received_at_ms);
            break;

        case CAN_PROTOCOL_EVENT_BASELINE_REQUIRED:
            return WINDOW_STATE_RX_GATED;

        case CAN_PROTOCOL_EVENT_CONFLICT:
        case CAN_PROTOCOL_EVENT_AMBIGUOUS:
        case CAN_PROTOCOL_EVENT_STALE:
        case CAN_PROTOCOL_EVENT_INVALID:
        default:
            return WINDOW_STATE_RX_REJECTED;
    }

    switch (command.command)
    {
        case CAN_PROTOCOL_CMD_STOP:
            window->state = CAN_PROTOCOL_STATE_STOP;
            if (window->recovery_phase == WINDOW_RECOVERY_WAIT_FINAL_STOP)
            {
                window->recovery_phase = WINDOW_RECOVERY_READY;
            }
            return WINDOW_STATE_RX_ACCEPTED;

        case CAN_PROTOCOL_CMD_UP:
            window->state = CAN_PROTOCOL_STATE_UP;
            return WINDOW_STATE_RX_ACCEPTED;

        case CAN_PROTOCOL_CMD_DOWN:
            window->state = CAN_PROTOCOL_STATE_DOWN;
            return WINDOW_STATE_RX_ACCEPTED;

        case CAN_PROTOCOL_CMD_DEMO_SET_ZERO:
        case CAN_PROTOCOL_CMD_DEMO_TOGGLE:
            window->state = CAN_PROTOCOL_STATE_STOP;
            return WINDOW_STATE_RX_ACCEPTED;

        case CAN_PROTOCOL_CMD_CLEAR_FAULT:
            if (window->recovery_phase == WINDOW_RECOVERY_WAIT_CLEAR)
            {
                if ((window->fault != CAN_PROTOCOL_FAULT_NONE) &&
                    !window->rx_overflow_active && !window->motor_fault_active)
                {
                    window->fault = CAN_PROTOCOL_FAULT_NONE;
                    window->state = CAN_PROTOCOL_STATE_STOP;
                    window->recovery_phase = WINDOW_RECOVERY_WAIT_FINAL_STOP;
                    return WINDOW_STATE_RX_CLEAR_SUCCEEDED;
                }

                return WINDOW_STATE_RX_CLEAR_FAILED;
            }

            /* CLEAR outside recovery is consumed as a no-op; it never moves. */
            return WINDOW_STATE_RX_ACCEPTED;

        default:
            return WINDOW_STATE_RX_REJECTED;
    }
}

void window_state_tick(window_state_t *window, uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if ((window == NULL) || !window->accepted_command_time_valid)
    {
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - window->last_accepted_command_ms);
    if (elapsed_ms <= WINDOW_STATE_COMMAND_TIMEOUT_MS)
    {
        return;
    }

    window_state_note_can_bus_off(window, now_ms);
}

void window_state_note_can_bus_off(window_state_t *window, uint32_t now_ms)
{
    if (window == NULL) { return; }
    window->state = CAN_PROTOCOL_STATE_STOP;
    /* Keep the existing recovery cause; always discard the old seq baseline. */
    if (window->fault == CAN_PROTOCOL_FAULT_NONE)
    {
        window->fault = CAN_PROTOCOL_FAULT_COMM_TIMEOUT;
    }
    window->last_sequence_valid = false;
    window->accepted_command_time_valid = false;
    window->recovery_phase = WINDOW_RECOVERY_WAIT_STOP;
    window->recovery_entry_ms = now_ms;
    window->recovery_entry_time_valid = true;
}

void window_state_note_local_output_stop(window_state_t *window)
{
    if (window != NULL)
    {
        window->state = CAN_PROTOCOL_STATE_STOP;
    }
}

bool window_state_note_demo_output(window_state_t *window,
                                   can_protocol_state_t output)
{
    if ((window == NULL) ||
        (window->fault != CAN_PROTOCOL_FAULT_NONE) ||
        (window->recovery_phase != WINDOW_RECOVERY_READY) ||
        (window->last_command != CAN_PROTOCOL_CMD_DEMO_TOGGLE) ||
        ((output != CAN_PROTOCOL_STATE_UP) &&
         (output != CAN_PROTOCOL_STATE_DOWN)))
    {
        return false;
    }

    window->state = output;
    return true;
}

void window_state_note_motor_fault(window_state_t *window, uint32_t now_ms)
{
    if (window == NULL) { return; }
    /* A persistent cause must not repeatedly discard the recovery STOP. */
    if (window->motor_fault_active) { return; }
    window->motor_fault_active = true;
    window->state = CAN_PROTOCOL_STATE_STOP;
    window->fault = CAN_PROTOCOL_FAULT_MOTOR_LOCAL;
    window->last_sequence_valid = false;
    window->accepted_command_time_valid = false;
    window->recovery_phase = WINDOW_RECOVERY_WAIT_STOP;
    window->recovery_entry_ms = now_ms;
    window->recovery_entry_time_valid = true;
}

void window_state_set_motor_fault_active(window_state_t *window, bool active)
{
    if (window != NULL) { window->motor_fault_active = active; }
}

bool window_state_can_rezero_after_motor_recovery(
    const window_state_t *window, bool restart_rezero_pending,
    bool motor_feedback_safe, bool motor_stationary)
{
    return (window != NULL) && restart_rezero_pending &&
           motor_feedback_safe && motor_stationary &&
           (window->state == CAN_PROTOCOL_STATE_STOP) &&
           (window->fault == CAN_PROTOCOL_FAULT_NONE) &&
           (window->recovery_phase == WINDOW_RECOVERY_READY);
}

void window_state_note_rx_overflow(
    window_state_t *window,
    uint32_t observed_at_ms)
{
    if (window == NULL)
    {
        return;
    }

    window_state_tick(window, observed_at_ms);
    if (window->rx_overflow_observation_count != UINT32_MAX)
    {
        window->rx_overflow_observation_count++;
    }
    window->rx_overflow_active = true;
    window->state = CAN_PROTOCOL_STATE_STOP;
    window->fault = CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW;
    window->last_sequence_valid = false;
    window->accepted_command_time_valid = false;
    window->recovery_phase = WINDOW_RECOVERY_WAIT_STOP;
    window->recovery_entry_ms = observed_at_ms;
    window->recovery_entry_time_valid = true;
}

void window_state_confirm_rx_overflow_cleared(window_state_t *window)
{
    if (window != NULL)
    {
        window->rx_overflow_active = false;
    }
}

bool window_state_get_status(
    const window_state_t *window,
    can_protocol_status_t *status_out)
{
    can_protocol_status_t status;

    if ((window == NULL) || (status_out == NULL))
    {
        return false;
    }

    status.last_sequence = window->last_sequence;
    status.state = window->state;
    status.fault = window->fault;
    status.position = 0xFFFFU;
    status.is_calibrated = false;
    status.last_sequence_valid = window->last_sequence_valid;
    status.rx_overflow_active = window->rx_overflow_active;
    *status_out = status;
    return true;
}

uint32_t window_state_get_rx_overflow_observation_count(
    const window_state_t *window)
{
    return (window != NULL) ? window->rx_overflow_observation_count : 0U;
}
