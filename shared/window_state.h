#ifndef SHARED_WINDOW_STATE_H
#define SHARED_WINDOW_STATE_H

#include "can_protocol.h"

/* D3 command supervision target; source of the threshold is the project plan. */
#define WINDOW_STATE_COMMAND_TIMEOUT_MS (300U)

typedef enum
{
    WINDOW_RECOVERY_WAIT_STOP = 0U,
    WINDOW_RECOVERY_WAIT_CLEAR,
    WINDOW_RECOVERY_WAIT_FINAL_STOP,
    WINDOW_RECOVERY_READY
} window_recovery_phase_t;

typedef struct
{
    /* Module-owned state: initialize/read through the functions below. */
    can_protocol_state_t state;
    can_protocol_fault_t fault;
    can_protocol_command_id_t last_command;
    window_recovery_phase_t recovery_phase;
    uint32_t last_accepted_command_ms;
    uint32_t recovery_entry_ms;
    uint32_t rx_overflow_observation_count;
    uint8_t last_sequence;
    bool last_sequence_valid;
    bool accepted_command_time_valid;
    bool recovery_entry_time_valid;
    bool rx_overflow_active;
    bool motor_fault_active;
} window_state_t;

typedef enum
{
    WINDOW_STATE_RX_INVALID_FRAME = 0U,
    WINDOW_STATE_RX_GATED,
    WINDOW_STATE_RX_REJECTED,
    WINDOW_STATE_RX_STALE_FRAME,
    WINDOW_STATE_RX_BASELINE_ACCEPTED,
    WINDOW_STATE_RX_REPEAT_ACCEPTED,
    WINDOW_STATE_RX_ACCEPTED,
    WINDOW_STATE_RX_CLEAR_FAILED,
    WINDOW_STATE_RX_CLEAR_SUCCEEDED
} window_state_rx_result_t;

/* Initializes the V0 state model; the target must set physical outputs safe first. */
void window_state_init(window_state_t *window);

/*
 * Takes both CAN receive time and processing time. Tick first, so a delayed
 * pre-timeout STOP cannot establish a new recovery baseline.
 */
window_state_rx_result_t window_state_receive_frame_at(
    window_state_t *window,
    const can_protocol_frame_t *frame,
    uint32_t received_at_ms,
    uint32_t processed_at_ms);

/* Call from the bounded local safety cadence; elapsed-time math wraps safely. */
void window_state_tick(window_state_t *window, uint32_t now_ms);

/* Link recovery never clears a business fault or restores an old motion. */
void window_state_note_can_bus_off(window_state_t *window, uint32_t now_ms);

/*
 * Records that a local actuator guard stopped the output. It preserves the
 * accepted command/sequence so a repeated old motion event cannot restart it.
 */
void window_state_note_local_output_stop(window_state_t *window);

/* The application calls this only after a new demo action starts its output. */
bool window_state_note_demo_output(window_state_t *window,
                                   can_protocol_state_t output);

/* UART/output cause remains latched until absent and a NEW CLEAR succeeds. */
void window_state_note_motor_fault(window_state_t *window, uint32_t now_ms);
void window_state_set_motor_fault_active(window_state_t *window, bool active);
bool window_state_can_rezero_after_motor_recovery(
    const window_state_t *window, bool restart_rezero_pending,
    bool motor_feedback_safe, bool motor_stationary);

/* Call once for each newly observed MCAN RF0L event. */
void window_state_note_rx_overflow(
    window_state_t *window,
    uint32_t observed_at_ms);

/* Call only after the MCAN FIFO and sticky overflow condition are confirmed clear. */
void window_state_confirm_rx_overflow_cleared(window_state_t *window);

bool window_state_get_status(
    const window_state_t *window,
    can_protocol_status_t *status_out);

uint32_t window_state_get_rx_overflow_observation_count(
    const window_state_t *window);

#endif /* SHARED_WINDOW_STATE_H */
