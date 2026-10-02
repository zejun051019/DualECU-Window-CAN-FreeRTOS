#include "../../shared/can_recovery_client.h"

#include <assert.h>
#include <stdio.h>

typedef struct
{
    uint32_t now_ms;
    uint16_t tx_timestamp;
} test_clock_t;

static can_protocol_status_t make_status(uint8_t sequence,
                                         bool sequence_valid,
                                         can_protocol_state_t state,
                                         can_protocol_fault_t fault)
{
    can_protocol_status_t status = {0};
    status.last_sequence = sequence;
    status.last_sequence_valid = sequence_valid;
    status.state = state;
    status.fault = fault;
    return status;
}

static void send_expected(can_recovery_client_t *client,
                          test_clock_t *clock,
                          uint8_t sequence,
                          can_protocol_command_id_t command_id)
{
    can_protocol_command_t command = {0};
    assert(can_recovery_client_get_command_at(client, clock->now_ms,
                                               &command));
    assert(command.sequence == sequence);
    assert(command.command == command_id);
    clock->tx_timestamp = (uint16_t)(clock->tx_timestamp + 25000U);
    assert(can_recovery_client_note_tx_success_at(
        client, clock->now_ms, clock->tx_timestamp));
}

static can_recovery_client_event_t observe_fresh(
    can_recovery_client_t *client,
    const can_protocol_status_t *status,
    const test_clock_t *clock)
{
    return can_recovery_client_observe_status_at(
        client, status, (uint16_t)(clock->tx_timestamp + 1U));
}

static void test_happy_path_and_sequence_wrap(void)
{
    can_recovery_client_t client = {0};
    test_clock_t clock = {0U, 0xA000U};
    can_protocol_status_t status;

    can_recovery_client_init(&client, 0xA5U);
    send_expected(&client, &clock, 0xA5U, CAN_PROTOCOL_CMD_STOP);

    clock.now_ms += 49U;
    assert(!can_recovery_client_get_command_at(
        &client, clock.now_ms, &(can_protocol_command_t){0}));
    clock.now_ms += 1U;
    send_expected(&client, &clock, 0xA5U, CAN_PROTOCOL_CMD_STOP);

    status = make_status(0xA5U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_STARTUP_LOCKED);
    assert(can_recovery_client_observe_status_at(
        &client, &status, (uint16_t)(clock.tx_timestamp - 1U)) ==
        CAN_RECOVERY_CLIENT_EVENT_NONE);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED);

    clock.now_ms += 1U;
    send_expected(&client, &clock, 0xA6U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    status = make_status(0xA5U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_STARTUP_LOCKED);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_NONE);
    status = make_status(0xA6U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_NONE);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_CLEAR_CONFIRMED);

    clock.now_ms += 1U;
    const uint16_t timestamp_before_final_stop = clock.tx_timestamp;
    send_expected(&client, &clock, 0xA7U, CAN_PROTOCOL_CMD_STOP);
    status = make_status(0xA7U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_NONE);
    assert(can_recovery_client_observe_status_at(
        &client, &status, (uint16_t)(timestamp_before_final_stop + 1U)) ==
        CAN_RECOVERY_CLIENT_EVENT_NONE);
    status = make_status(0xA7U, false, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_NONE);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_NONE);
    status = make_status(0xA7U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_COMM_TIMEOUT);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_NONE);
    status = make_status(0xA7U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_NONE);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_FINAL_STOP_CONFIRMED);
    assert(client.stage == CAN_RECOVERY_CLIENT_COMPLETE);
    send_expected(&client, &clock, 0xA7U, CAN_PROTOCOL_CMD_STOP);
    clock.now_ms += CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS - 1U;
    assert(!can_recovery_client_get_command_at(
        &client, clock.now_ms, &(can_protocol_command_t){0}));
    clock.now_ms += 1U;
    send_expected(&client, &clock, 0xA7U, CAN_PROTOCOL_CMD_STOP);
    assert(client.stage == CAN_RECOVERY_CLIENT_COMPLETE);
    assert(client.next_sequence == 0xA7U);

    can_recovery_client_init(&client, 0x55U);
    assert(client.stage == CAN_RECOVERY_CLIENT_SEND_STARTUP_STOP);
    assert(client.next_sequence == 0x55U);
    can_protocol_command_t restart_command = {0};
    assert(can_recovery_client_get_command_at(
        &client, clock.now_ms, &restart_command));
    assert(restart_command.sequence == 0x55U);
    assert(restart_command.command == CAN_PROTOCOL_CMD_STOP);

    can_recovery_client_t wrap_client = {0};
    test_clock_t wrap_clock = {0U, 0xFFF0U};
    can_recovery_client_init(&wrap_client, 0xFFU);
    send_expected(&wrap_client, &wrap_clock, 0xFFU, CAN_PROTOCOL_CMD_STOP);
    status = make_status(0xFFU, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_STARTUP_LOCKED);
    assert(observe_fresh(&wrap_client, &status, &wrap_clock) ==
           CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED);
    wrap_clock.now_ms += 1U;
    send_expected(&wrap_client, &wrap_clock, 0x00U,
                  CAN_PROTOCOL_CMD_CLEAR_FAULT);
    status = make_status(0x00U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_NONE);
    assert(observe_fresh(&wrap_client, &status, &wrap_clock) ==
           CAN_RECOVERY_CLIENT_EVENT_CLEAR_CONFIRMED);
    assert(wrap_client.next_sequence == 0x01U);
}

static void test_failed_clear_is_not_revived(void)
{
    can_recovery_client_t client = {0};
    test_clock_t clock = {10U, 0x1000U};
    can_protocol_status_t status;
    can_protocol_command_t command = {0};

    can_recovery_client_init(&client, 0x20U);
    send_expected(&client, &clock, 0x20U, CAN_PROTOCOL_CMD_STOP);
    status = make_status(0x20U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_STARTUP_LOCKED);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED);

    clock.now_ms += 1U;
    send_expected(&client, &clock, 0x21U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    status = make_status(0x21U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_COMM_TIMEOUT);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_CLEAR_REJECTED);

    clock.now_ms += CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS;
    assert(can_recovery_client_get_command_at(&client, clock.now_ms,
                                               &command));
    assert(command.sequence == 0x21U);
    assert(command.command == CAN_PROTOCOL_CMD_CLEAR_FAULT);
    clock.tx_timestamp = (uint16_t)(clock.tx_timestamp + 25000U);
    assert(can_recovery_client_note_tx_success_at(
        &client, clock.now_ms, clock.tx_timestamp));
    status = make_status(0x21U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_NONE);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_NONE);
    assert(client.stage == CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS);
    assert(client.next_sequence == 0x21U);
}

static void test_overflow_cause_gates_recovery(void)
{
    can_recovery_client_t client = {0};
    test_clock_t clock = {0U, 0x2000U};
    can_protocol_status_t status;

    can_recovery_client_init(&client, 0x40U);
    send_expected(&client, &clock, 0x40U, CAN_PROTOCOL_CMD_STOP);

    status = make_status(0x40U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW);
    status.rx_overflow_active = true;
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_NONE);
    assert(client.stage == CAN_RECOVERY_CLIENT_WAIT_STARTUP_STOP_STATUS);

    clock.now_ms += CAN_RECOVERY_CLIENT_RETRY_PERIOD_MS;
    send_expected(&client, &clock, 0x40U, CAN_PROTOCOL_CMD_STOP);
    status.rx_overflow_active = false;
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_STARTUP_STOP_CONFIRMED);

    clock.now_ms += 1U;
    send_expected(&client, &clock, 0x41U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    status = make_status(0x41U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW);
    status.rx_overflow_active = true;
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_CLEAR_REJECTED);
    assert(client.clear_failed);

    status = make_status(0x41U, true, CAN_PROTOCOL_STATE_STOP,
                         CAN_PROTOCOL_FAULT_NONE);
    assert(observe_fresh(&client, &status, &clock) ==
           CAN_RECOVERY_CLIENT_EVENT_NONE);
    assert(client.stage == CAN_RECOVERY_CLIENT_WAIT_CLEAR_STATUS);
    assert(client.next_sequence == 0x41U);
}

int main(void)
{
    test_happy_path_and_sequence_wrap();
    test_failed_clear_is_not_revived();
    test_overflow_cause_gates_recovery();
    puts("CAN recovery client gating and freshness checks passed");
    return 0;
}
