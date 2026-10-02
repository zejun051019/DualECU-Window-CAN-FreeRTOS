#ifndef SHARED_CAN_RECOVERY_CLIENT_H
#define SHARED_CAN_RECOVERY_CLIENT_H

#include "can_protocol.h"

#define CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS (50U)
#define CAN_RECOVERY_CLIENT_ACK_DEADLINE_MS (300U)

typedef enum
{
    CAN_RECOVERY_CLIENT_SEND_STARTUP_STOP = 0U,
    CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS,
    CAN_RECOVERY_CLIENT_SEND_CLEAR,
    CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS,
    CAN_RECOVERY_CLIENT_SEND_FINAL_STOP,
    CAN_RECOVERY_CLIENT_WAIT_FINAL_STOP_STATUS,
    CAN_RECOVERY_CLIENT_COMPLETE
} can_recovery_client_stage_t;

typedef enum
{
    CAN_RECOVERY_CLIENT_EVENT_NONE = 0U,
    CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED,
    CAN_RECOVERY_CLIENT_EVENT_CLEAR_CONFIRMED,
    CAN_RECOVERY_CLIENT_EVENT_FINAL_STOP_CONFIRMED,
    CAN_RECOVERY_CLIENT_EVENT_CLEAR_REJECTED
} can_recovery_client_event_t;

typedef struct
{
    can_recovery_client_stage_t stage;
    uint8_t next_sequence;
    uint32_t ack_started_ms;
    uint32_t last_tx_attempt_ms;
    uint16_t last_tx_timestamp;
    bool has_tx_attempt;
    bool has_ack_deadline;
    bool ack_deadline_reported;
    bool has_tx_timestamp;
    bool clear_failed;
} can_recovery_client_t;

void can_recovery_client_init(can_recovery_client_t *client,
                              uint8_t startup_stop_sequence);
bool can_recovery_client_get_command_at(
    const can_recovery_client_t *client,
    uint32_t now_ms,
    can_protocol_command_t *command_out);
bool can_recovery_client_note_tx_success_at(
    can_recovery_client_t *client,
    uint32_t now_ms,
    uint16_t tx_timestamp);
bool can_recovery_client_note_tx_failure_at(
    can_recovery_client_t *client,
    uint32_t now_ms);
bool can_recovery_client_take_ack_timeout(
    can_recovery_client_t *client,
    uint32_t now_ms);
can_recovery_client_event_t can_recovery_client_observe_status_at(
    can_recovery_client_t *client,
    const can_protocol_status_t *status,
    uint16_t rx_timestamp);

#endif
