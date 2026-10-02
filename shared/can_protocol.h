#ifndef SHARED_CAN_PROTOCOL_H
#define SHARED_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_PROTOCOL_VERSION       (4U)
#define CAN_PROTOCOL_COMMAND_ID    (0x100U)
#define CAN_PROTOCOL_COMMAND_DLC   (4U)
#define CAN_PROTOCOL_STATUS_ID     (0x180U)
#define CAN_PROTOCOL_STATUS_DLC    (8U)
#define CAN_PROTOCOL_MAX_DATA_LEN  (8U)

typedef enum
{
    CAN_PROTOCOL_CMD_STOP = 0U,
    CAN_PROTOCOL_CMD_UP = 1U,
    CAN_PROTOCOL_CMD_DOWN = 2U,
    CAN_PROTOCOL_CMD_CLEAR_FAULT = 3U,
    CAN_PROTOCOL_CMD_DEMO_SET_ZERO = 4U,
    CAN_PROTOCOL_CMD_DEMO_TOGGLE = 5U
} can_protocol_command_id_t;

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    bool is_extended;
    bool is_remote;
    uint8_t data[CAN_PROTOCOL_MAX_DATA_LEN];
} can_protocol_frame_t;

typedef struct
{
    uint8_t sequence;
    can_protocol_command_id_t command;
} can_protocol_command_t;

typedef enum
{
    CAN_PROTOCOL_STATE_STOP = 0U,
    CAN_PROTOCOL_STATE_UP,
    CAN_PROTOCOL_STATE_DOWN
} can_protocol_state_t;

typedef enum
{
    CAN_PROTOCOL_FAULT_NONE = 0U,
    CAN_PROTOCOL_FAULT_STARTUP_LOCKED,
    CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
    CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW,
    CAN_PROTOCOL_FAULT_MOTOR_LOCAL
} can_protocol_fault_t;

typedef struct
{
    uint8_t last_sequence;
    can_protocol_state_t state;
    can_protocol_fault_t fault;
    uint16_t position;
    bool is_calibrated;
    bool last_sequence_valid;
    bool rx_overflow_active;
} can_protocol_status_t;

typedef enum
{
    CAN_PROTOCOL_EVENT_BASELINE_REQUIRED = 0U,
    CAN_PROTOCOL_EVENT_BASELINE_STOP,
    CAN_PROTOCOL_EVENT_REPEAT,
    CAN_PROTOCOL_EVENT_NEW,
    CAN_PROTOCOL_EVENT_CONFLICT,
    CAN_PROTOCOL_EVENT_AMBIGUOUS,
    CAN_PROTOCOL_EVENT_STALE,
    CAN_PROTOCOL_EVENT_INVALID
} can_protocol_event_result_t;

/* Checks only wire structure; sequence acceptance and safety gating are separate. */
bool can_protocol_decode_command(
    const can_protocol_frame_t *frame,
    can_protocol_command_t *command_out);

bool can_protocol_encode_command(
    const can_protocol_command_t *command,
    can_protocol_frame_t *frame_out);

bool can_protocol_decode_status(
    const can_protocol_frame_t *frame,
    can_protocol_status_t *status_out);

bool can_protocol_encode_status(
    const can_protocol_status_t *status,
    can_protocol_frame_t *frame_out);

/*
 * Classifies sequence freshness only; callers own state updates and gating.
 * A false last_sequence_valid denotes a startup/latched recovery phase.
 */
can_protocol_event_result_t can_protocol_classify_event(
    const can_protocol_command_t *command,
    bool last_sequence_valid,
    uint8_t last_sequence,
    can_protocol_command_id_t last_command);

#endif /* SHARED_CAN_PROTOCOL_H */
