#include "can_recovery_client.h"

#include <stddef.h>

void can_recovery_client_init(can_recovery_client_t *client,
                              uint8_t startup_stop_sequence)
{
    if (client != NULL) {
        *client = (can_recovery_client_t){0};
        client->stage = CAN_RECOVERY_CLIENT_SEND_STARTUP_STOP;
        client->next_sequence = startup_stop_sequence;
    }
}

static bool can_recovery_client_is_wait_stage(
    can_recovery_client_stage_t stage)
{
    return (stage == CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS) ||
           (stage == CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS) ||
           (stage == CAN_RECOVERY_CLIENT_WAIT_FINAL_STOP_STATUS);
}

static bool can_recovery_client_command_for_stage(
    can_recovery_client_stage_t stage,
    can_protocol_command_id_t *command_out)
{
    if (command_out == NULL) {
        return false;
    }
    switch (stage) {
    case CAN_RECOVERY_CLIENT_SEND_STARTUP_STOP:
    case CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS:
    case CAN_RECOVERY_CLIENT_SEND_FINAL_STOP:
    case CAN_RECOVERY_CLIENT_WAIT_FINAL_STOP_STATUS:
    case CAN_RECOVERY_CLIENT_COMPLETE:
        *command_out = CAN_PROTOCOL_CMD_STOP;
        return true;
    case CAN_RECOVERY_CLIENT_SEND_CLEAR:
    case CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS:
        *command_out = CAN_PROTOCOL_CMD_CLEAR_FAULT;
        return true;
    default:
        return false;
    }
}

static bool can_recovery_client_tx_due(
    const can_recovery_client_t *client,
    uint32_t now_ms)
{
    const uint32_t elapsed_ms = now_ms - client->last_tx_attempt_ms;

    /* Treat intervals over half a tick cycle as a stale/future snapshot. */
    return !client->has_tx_attempt ||
           ((elapsed_ms < 0x80000000U) &&
            (elapsed_ms >= CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS));
}

bool can_recovery_client_get_command_at(
    const can_recovery_client_t *client,
    uint32_t now_ms,
    can_protocol_command_t *command_out)
{
    can_protocol_command_id_t command;

    if ((client == NULL) || (command_out == NULL) ||
        !can_recovery_client_command_for_stage(client->stage, &command) ||
        !can_recovery_client_tx_due(client, now_ms)) {
        return false;
    }
    command_out->sequence = client->next_sequence;
    command_out->command = command;
    return true;
}

bool can_recovery_client_note_tx_success_at(
    can_recovery_client_t *client,
    uint32_t now_ms,
    uint16_t tx_timestamp)
{
    can_protocol_command_t unused_command;
    can_recovery_client_stage_t next_wait_stage;

    if ((client == NULL) ||
        !can_recovery_client_get_command_at(client, now_ms, &unused_command)) {
        return false;
    }
    client->last_tx_attempt_ms = now_ms;
    client->has_tx_attempt = true;
    client->last_tx_timestamp = tx_timestamp;
    client->has_tx_timestamp = true;

    if (client->stage == CAN_RECOVERY_CLIENT_COMPLETE) {
        return true;
    }

    switch (client->stage) {
    case CAN_RECOVERY_CLIENT_SEND_STARTUP_STOP:
        next_wait_stage = CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS;
        break;
    case CAN_RECOVERY_CLIENT_SEND_CLEAR:
        next_wait_stage = CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS;
        client->clear_failed = false;
        break;
    case CAN_RECOVERY_CLIENT_SEND_FINAL_STOP:
        next_wait_stage = CAN_RECOVERY_CLIENT_WAIT_FINAL_STOP_STATUS;
        break;
    default:
        if (!can_recovery_client_is_wait_stage(client->stage)) {
            return false;
        }
        /* Retransmission refreshes the freshness anchor, not the deadline. */
        return true;
    }

    client->stage = next_wait_stage;
    client->ack_started_ms = now_ms;
    client->has_ack_deadline = true;
    client->ack_deadline_reported = false;
    return true;
}

bool can_recovery_client_note_tx_failure_at(
    can_recovery_client_t *client,
    uint32_t now_ms)
{
    can_protocol_command_t unused_command;

    if ((client == NULL) ||
        !can_recovery_client_get_command_at(client, now_ms, &unused_command)) {
        return false;
    }
    client->last_tx_attempt_ms = now_ms;
    client->has_tx_attempt = true;
    return true;
}

bool can_recovery_client_take_ack_timeout(
    can_recovery_client_t *client,
    uint32_t now_ms)
{
    if ((client == NULL) || !can_recovery_client_is_wait_stage(client->stage) ||
        !client->has_ack_deadline || client->ack_deadline_reported ||
        ((uint32_t)(now_ms - client->ack_started_ms) <
         CAN_RECOVERY_CLIENT_ACK_DEADLINE_MS)) {
        return false;
    }
    client->ack_deadline_reported = true;
    return true;
}

static bool can_recovery_client_status_is_fresh(
    const can_recovery_client_t *client,
    uint16_t rx_timestamp)
{
    const uint16_t elapsed_ticks =
        (uint16_t)(rx_timestamp - client->last_tx_timestamp);

    /* bxCAN timestamps are 16-bit; accept only unambiguous forward time. */
    return client->has_tx_timestamp && (elapsed_ticks != 0U) &&
           (elapsed_ticks < 0x8000U);
}

static void can_recovery_client_start_next_stage(
    can_recovery_client_t *client,
    can_recovery_client_stage_t stage)
{
    client->stage = stage;
    client->has_tx_attempt = false;
    client->has_ack_deadline = false;
    client->ack_deadline_reported = false;
    client->has_tx_timestamp = false;
    client->clear_failed = false;
}

can_recovery_client_event_t can_recovery_client_observe_status_at(
    can_recovery_client_t *client,
    const can_protocol_status_t *status,
    uint16_t rx_timestamp)
{
    if ((client == NULL) || (status == NULL) ||
        !can_recovery_client_is_wait_stage(client->stage) ||
        !can_recovery_client_status_is_fresh(client, rx_timestamp) ||
        !status->last_sequence_valid ||
        (status->last_sequence != client->next_sequence) ||
        (status->state != CAN_PROTOCOL_STATE_STOP)) {
        return CAN_RECOVERY_CLIENT_EVENT_NONE;
    }

    switch (client->stage) {
    case CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS:
        if (status->rx_overflow_active) {
            return CAN_RECOVERY_CLIENT_EVENT_NONE;
        }
        client->next_sequence = (uint8_t)(client->next_sequence + 1U);
        can_recovery_client_start_next_stage(
            client, CAN_RECOVERY_CLIENT_SEND_CLEAR);
        return CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED;

    case CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS:
        if (client->clear_failed) {
            return CAN_RECOVERY_CLIENT_EVENT_NONE;
        }
        if (status->fault != CAN_PROTOCOL_FAULT_NONE) {
            client->clear_failed = true;
            return CAN_RECOVERY_CLIENT_EVENT_CLEAR_REJECTED;
        }
        client->next_sequence = (uint8_t)(client->next_sequence + 1U);
        can_recovery_client_start_next_stage(
            client, CAN_RECOVERY_CLIENT_SEND_FINAL_STOP);
        return CAN_RECOVERY_CLIENT_EVENT_CLEAR_CONFIRMED;

    case CAN_RECOVERY_CLIENT_WAIT_FINAL_STOP_STATUS:
        if (status->fault != CAN_PROTOCOL_FAULT_NONE) {
            return CAN_RECOVERY_CLIENT_EVENT_NONE;
        }
        client->stage = CAN_RECOVERY_CLIENT_COMPLETE;
        client->has_tx_attempt = false;
        client->has_ack_deadline = false;
        client->has_tx_timestamp = false;
        return CAN_RECOVERY_CLIENT_EVENT_FINAL_STOP_CONFIRMED;

    default:
        return CAN_RECOVERY_CLIENT_EVENT_NONE;
    }
}
