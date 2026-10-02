#include "../../shared/can_recovery_client.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>

static can_protocol_status_t make_status(uint8_t sequence,
                                         can_protocol_fault_t fault)
{
    can_protocol_status_t status = {0};
    status.last_sequence = sequence;
    status.last_sequence_valid = true;
    status.state = CAN_PROTOCOL_STATE_STOP;
    status.fault = fault;
    return status;
}

static void expect_retry(const can_recovery_client_t *client,
                         uint32_t now_ms,
                         uint8_t sequence,
                         can_protocol_command_id_t command_id)
{
    can_protocol_command_t command = {0};
    assert(can_recovery_client_get_command_at(client, now_ms, &command));
    assert(command.sequence == sequence);
    assert(command.command == command_id);
}

static void test_tx_failure_does_not_start_ack_deadline(void)
{
    can_recovery_client_t client = {0};
    const uint32_t start_ms = UINT32_MAX - 20U;
    const uint16_t tx_timestamp = 0x1234U;
    can_protocol_command_t command = {0};
    can_protocol_status_t status;

    can_recovery_client_init(&client, 0x5AU);
    expect_retry(&client, start_ms, 0x5AU, CAN_PROTOCOL_CMD_STOP);
    assert(can_recovery_client_note_tx_failure_at(&client, start_ms));
    assert(client.stage == CAN_RECOVERY_CLIENT_SEND_STARTUP_STOP);
    assert(!client.has_ack_deadline);
    assert(!client.has_tx_timestamp);
    assert(!can_recovery_client_take_ack_timeout(&client, start_ms + 300U));

    assert(!can_recovery_client_get_command_at(
        &client, start_ms + CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS - 1U,
        &command));
    expect_retry(&client, start_ms + CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS,
                 0x5AU, CAN_PROTOCOL_CMD_STOP);
    assert(can_recovery_client_note_tx_success_at(
        &client, start_ms + CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS,
        tx_timestamp));
    assert(client.stage == CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS);
    assert(client.has_ack_deadline);
    assert(client.last_tx_timestamp == tx_timestamp);

    status = make_status(0x5AU, CAN_PROTOCOL_FAULT_STARTUP_LOCKED);
    assert(can_recovery_client_observe_status_at(
        &client, &status, (uint16_t)(tx_timestamp - 1U)) ==
        CAN_RECOVERY_CLIENT_EVENT_NONE);
    assert(client.stage == CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS);

    puts("CAN TX failure keeps STOP pending without starting ACK deadline");
}

static void test_stale_task_tick_does_not_trigger_retry(void)
{
    can_recovery_client_t client = {0};
    can_protocol_command_t command = {0};

    can_recovery_client_init(&client, 0x31U);
    expect_retry(&client, 100U, 0x31U, CAN_PROTOCOL_CMD_STOP);
    assert(can_recovery_client_note_tx_success_at(&client, 101U, 10U));

    /* A task can hold a tick snapshot from just before TX completion. */
    assert(!can_recovery_client_get_command_at(&client, 100U, &command));
    assert(!can_recovery_client_note_tx_success_at(&client, 102U, 11U));
    expect_retry(&client, 151U, 0x31U, CAN_PROTOCOL_CMD_STOP);

    puts("CAN recovery ignores stale task ticks when checking retries");
}

int main(void)
{
    test_tx_failure_does_not_start_ack_deadline();
    test_stale_task_tick_does_not_trigger_retry();
    can_recovery_client_t client = {0};
    const uint32_t start_ms = UINT32_MAX - 20U;
    uint16_t tx_timestamp = 0xFFF0U;
    can_protocol_status_t status;

    can_recovery_client_init(&client, 0x5AU);
    expect_retry(&client, start_ms, 0x5AU, CAN_PROTOCOL_CMD_STOP);
    assert(can_recovery_client_note_tx_success_at(
        &client, start_ms, tx_timestamp));

    assert(!can_recovery_client_get_command_at(
        &client, start_ms + 49U, &(can_protocol_command_t){0}));
    expect_retry(&client, start_ms + 50U, 0x5AU, CAN_PROTOCOL_CMD_STOP);
    tx_timestamp = (uint16_t)(tx_timestamp + 25000U);
    assert(can_recovery_client_note_tx_success_at(
        &client, start_ms + 50U, tx_timestamp));

    /* A matching sequence from before the latest retry is stale. */
    status = make_status(0x5AU, CAN_PROTOCOL_FAULT_STARTUP_LOCKED);
    assert(can_recovery_client_observe_status_at(
        &client, &status, (uint16_t)(tx_timestamp - 1U)) ==
        CAN_RECOVERY_CLIENT_EVENT_NONE);

    for (uint32_t retry = 2U; retry <= 5U; ++retry) {
        const uint32_t elapsed_ms = retry * 50U;
        expect_retry(&client, start_ms + elapsed_ms, 0x5AU,
                     CAN_PROTOCOL_CMD_STOP);
        tx_timestamp = (uint16_t)(tx_timestamp + 25000U);
        assert(can_recovery_client_note_tx_success_at(
            &client, start_ms + elapsed_ms, tx_timestamp));
    }

    assert(!can_recovery_client_take_ack_timeout(&client,
                                                 start_ms + 299U));
    assert(can_recovery_client_take_ack_timeout(&client,
                                                 start_ms + 300U));
    assert(!can_recovery_client_take_ack_timeout(&client,
                                                 start_ms + 301U));
    expect_retry(&client, start_ms + 300U, 0x5AU, CAN_PROTOCOL_CMD_STOP);
    tx_timestamp = (uint16_t)(tx_timestamp + 25000U);
    assert(can_recovery_client_note_tx_success_at(
        &client, start_ms + 300U, tx_timestamp));

    /* Late success is still accepted when its hardware SOF follows the retry. */
    status = make_status(0x5AU, CAN_PROTOCOL_FAULT_COMM_TIMEOUT);
    assert(can_recovery_client_observe_status_at(
        &client, &status, (uint16_t)(tx_timestamp + 500U)) ==
        CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED);
    assert(client.stage == CAN_RECOVERY_CLIENT_SEND_CLEAR);

    puts("CAN recovery retry, deadline, freshness, and tick-wrap checks passed");
    return 0;
}
