#include "../../shared/can_protocol.h"

#include <assert.h>
#include <stdio.h>

static can_protocol_frame_t make_command_frame(
    uint8_t sequence, uint8_t command)
{
    can_protocol_frame_t frame = {0};

    frame.id = CAN_PROTOCOL_COMMAND_ID;
    frame.dlc = CAN_PROTOCOL_COMMAND_DLC;
    frame.data[0] = CAN_PROTOCOL_VERSION;
    frame.data[1] = sequence;
    frame.data[2] = command;
    frame.data[3] = 0U;
    return frame;
}

static can_protocol_frame_t make_status_frame(
    uint8_t last_sequence,
    uint8_t state,
    uint8_t fault,
    uint16_t position,
    uint8_t flags)
{
    can_protocol_frame_t frame = {0};

    frame.id = CAN_PROTOCOL_STATUS_ID;
    frame.dlc = CAN_PROTOCOL_STATUS_DLC;
    frame.data[0] = CAN_PROTOCOL_VERSION;
    frame.data[1] = last_sequence;
    frame.data[2] = state;
    frame.data[3] = fault;
    frame.data[4] = (uint8_t)(position & 0xFFU);
    frame.data[5] = (uint8_t)(position >> 8U);
    frame.data[6] = flags;
    frame.data[7] = 0U;
    return frame;
}

static void expect_valid_command(uint8_t sequence, uint8_t command)
{
    can_protocol_frame_t frame = make_command_frame(sequence, command);
    can_protocol_command_t decoded = {0xA5U, CAN_PROTOCOL_CMD_CLEAR_FAULT};

    assert(can_protocol_decode_command(&frame, &decoded));
    assert(decoded.sequence == sequence);
    assert(decoded.command == (can_protocol_command_id_t)command);
}

static void expect_invalid_command(const can_protocol_frame_t *frame)
{
    can_protocol_command_t decoded = {0xA5U, CAN_PROTOCOL_CMD_CLEAR_FAULT};

    assert(!can_protocol_decode_command(frame, &decoded));
    assert(decoded.sequence == 0xA5U);
    assert(decoded.command == CAN_PROTOCOL_CMD_CLEAR_FAULT);
}

static void expect_valid_status(const can_protocol_frame_t *frame)
{
    can_protocol_status_t decoded = {0};

    assert(can_protocol_decode_status(frame, &decoded));
    assert(decoded.last_sequence == frame->data[1]);
    assert(decoded.state == (can_protocol_state_t)frame->data[2]);
    assert(decoded.fault == (can_protocol_fault_t)frame->data[3]);
    assert(decoded.position ==
           (uint16_t)(((uint16_t)frame->data[5] << 8U) | frame->data[4]));
    assert(decoded.is_calibrated == ((frame->data[6] & 0x01U) != 0U));
    assert(decoded.last_sequence_valid == ((frame->data[6] & 0x02U) != 0U));
    assert(decoded.rx_overflow_active == ((frame->data[6] & 0x04U) != 0U));
}

static void expect_invalid_status(const can_protocol_frame_t *frame)
{
    can_protocol_status_t decoded = {
        0xA5U,
        CAN_PROTOCOL_STATE_DOWN,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        0x1234U,
        true,
        true,
        true};

    assert(!can_protocol_decode_status(frame, &decoded));
    assert(decoded.last_sequence == 0xA5U);
    assert(decoded.state == CAN_PROTOCOL_STATE_DOWN);
    assert(decoded.fault == CAN_PROTOCOL_FAULT_COMM_TIMEOUT);
    assert(decoded.position == 0x1234U);
    assert(decoded.is_calibrated);
    assert(decoded.last_sequence_valid);
    assert(decoded.rx_overflow_active);
}

static void test_command_encoding(void)
{
    const can_protocol_command_t command = {0xA5U, CAN_PROTOCOL_CMD_DOWN};
    can_protocol_frame_t frame = {0};
    can_protocol_command_t decoded = {0};

    assert(can_protocol_encode_command(&command, &frame));
    assert(frame.id == CAN_PROTOCOL_COMMAND_ID);
    assert(frame.dlc == CAN_PROTOCOL_COMMAND_DLC);
    assert(!frame.is_extended);
    assert(!frame.is_remote);
    assert(frame.data[0] == CAN_PROTOCOL_VERSION);
    assert(frame.data[1] == 0xA5U);
    assert(frame.data[2] == CAN_PROTOCOL_CMD_DOWN);
    assert(frame.data[3] == 0U);
    assert(can_protocol_decode_command(&frame, &decoded));
    assert(decoded.sequence == command.sequence);
    assert(decoded.command == command.command);
}

static void test_demo_command_extension(void)
{
    const can_protocol_command_t set_zero = {0x31U, (can_protocol_command_id_t)4U};
    const can_protocol_command_t toggle = {0x32U, (can_protocol_command_id_t)5U};
    can_protocol_command_t decoded = {0};
    can_protocol_frame_t frame = {0};

    assert(CAN_PROTOCOL_VERSION == 4U);
    assert(can_protocol_encode_command(&set_zero, &frame));
    assert(frame.data[2] == 4U);
    assert(can_protocol_decode_command(&frame, &decoded));
    assert(decoded.command == (can_protocol_command_id_t)4U);
    assert(can_protocol_encode_command(&toggle, &frame));
    assert(frame.data[2] == 5U);
    assert(can_protocol_decode_command(&frame, &decoded));
    assert(decoded.command == (can_protocol_command_id_t)5U);
    frame.data[0] = 3U;
    assert(!can_protocol_decode_command(&frame, &decoded));
}

static void test_status_encoding(void)
{
    const can_protocol_status_t status = {
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_STARTUP_LOCKED,
        0xFFFFU,
        false,
        true,
        false};
    can_protocol_frame_t frame = {0};
    can_protocol_status_t decoded = {0};

    assert(can_protocol_encode_status(&status, &frame));
    assert(frame.id == CAN_PROTOCOL_STATUS_ID);
    assert(frame.dlc == CAN_PROTOCOL_STATUS_DLC);
    assert(!frame.is_extended);
    assert(!frame.is_remote);
    assert(frame.data[0] == CAN_PROTOCOL_VERSION);
    assert(frame.data[1] == 31U);
    assert(frame.data[2] == CAN_PROTOCOL_STATE_STOP);
    assert(frame.data[3] == CAN_PROTOCOL_FAULT_STARTUP_LOCKED);
    assert(frame.data[4] == 0xFFU);
    assert(frame.data[5] == 0xFFU);
    assert(frame.data[6] == 0x02U);
    assert(frame.data[7] == 0U);
    assert(can_protocol_decode_status(&frame, &decoded));
    assert(decoded.last_sequence == status.last_sequence);
    assert(decoded.state == status.state);
    assert(decoded.fault == status.fault);
    assert(decoded.position == status.position);
    assert(decoded.is_calibrated == status.is_calibrated);
    assert(decoded.last_sequence_valid == status.last_sequence_valid);
    assert(decoded.rx_overflow_active == status.rx_overflow_active);

    {
        const can_protocol_status_t overflow_status = {
            0x42U,
            CAN_PROTOCOL_STATE_STOP,
            CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW,
            0xFFFFU,
            false,
            true,
            true};
        assert(can_protocol_encode_status(&overflow_status, &frame));
        assert(frame.data[0] == CAN_PROTOCOL_VERSION);
        assert(frame.data[6] == 0x06U);
        assert(can_protocol_decode_status(&frame, &decoded));
        assert(decoded.rx_overflow_active);
    }
}

static void test_invalid_encodings_preserve_output(void)
{
    const can_protocol_command_t invalid_command = {
        7U,
        (can_protocol_command_id_t)-1};
    const can_protocol_status_t invalid_status = {
        7U,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        0xFFFFU,
        false,
        true,
        false};
    can_protocol_frame_t frame = {0};

    frame.id = 0x0555U;
    frame.dlc = 3U;
    frame.data[0] = 0xCCU;
    assert(!can_protocol_encode_command(&invalid_command, &frame));
    assert(frame.id == 0x0555U);
    assert(frame.dlc == 3U);
    assert(frame.data[0] == 0xCCU);

    assert(!can_protocol_encode_status(&invalid_status, &frame));
    assert(frame.id == 0x0555U);
    assert(frame.dlc == 3U);
    assert(frame.data[0] == 0xCCU);
}

static void expect_event_result(
    uint8_t sequence,
    can_protocol_command_id_t command_id,
    bool last_sequence_valid,
    uint8_t last_sequence,
    can_protocol_command_id_t last_command,
    can_protocol_event_result_t expected)
{
    const can_protocol_command_t command = {sequence, command_id};

    assert(can_protocol_classify_event(
               &command,
               last_sequence_valid,
               last_sequence,
               last_command) == expected);
}

int main(void)
{
    can_protocol_frame_t frame;

    expect_valid_command(0U, CAN_PROTOCOL_CMD_STOP);
    expect_valid_command(10U, CAN_PROTOCOL_CMD_UP);
    expect_valid_command(11U, CAN_PROTOCOL_CMD_DOWN);
    expect_valid_command(255U, CAN_PROTOCOL_CMD_CLEAR_FAULT);
    test_demo_command_extension();

    frame = make_command_frame(1U, CAN_PROTOCOL_CMD_STOP);
    frame.id++;
    expect_invalid_command(&frame);

    frame = make_command_frame(1U, CAN_PROTOCOL_CMD_STOP);
    frame.is_extended = true;
    expect_invalid_command(&frame);

    frame = make_command_frame(1U, CAN_PROTOCOL_CMD_STOP);
    frame.is_remote = true;
    expect_invalid_command(&frame);

    frame = make_command_frame(1U, CAN_PROTOCOL_CMD_STOP);
    frame.dlc = CAN_PROTOCOL_COMMAND_DLC - 1U;
    expect_invalid_command(&frame);

    frame = make_command_frame(1U, CAN_PROTOCOL_CMD_STOP);
    frame.dlc = CAN_PROTOCOL_COMMAND_DLC + 1U;
    expect_invalid_command(&frame);

    frame = make_command_frame(1U, CAN_PROTOCOL_CMD_STOP);
    frame.data[0]++;
    expect_invalid_command(&frame);

    frame = make_command_frame(1U, 6U);
    expect_invalid_command(&frame);

    frame = make_command_frame(1U, CAN_PROTOCOL_CMD_STOP);
    frame.data[3] = 1U;
    expect_invalid_command(&frame);

    assert(!can_protocol_decode_command(NULL, NULL));

    expect_event_result(
        20U,
        CAN_PROTOCOL_CMD_STOP,
        false,
        0U,
        CAN_PROTOCOL_CMD_STOP,
        CAN_PROTOCOL_EVENT_BASELINE_STOP);
    expect_event_result(
        20U,
        CAN_PROTOCOL_CMD_UP,
        false,
        0U,
        CAN_PROTOCOL_CMD_STOP,
        CAN_PROTOCOL_EVENT_BASELINE_REQUIRED);
    expect_event_result(
        0U,
        CAN_PROTOCOL_CMD_STOP,
        true,
        255U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_NEW);
    expect_event_result(
        11U,
        CAN_PROTOCOL_CMD_DOWN,
        true,
        10U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_NEW);
    expect_event_result(
        10U,
        CAN_PROTOCOL_CMD_UP,
        true,
        10U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_REPEAT);
    expect_event_result(
        10U,
        CAN_PROTOCOL_CMD_STOP,
        true,
        10U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_CONFLICT);
    expect_event_result(
        138U,
        CAN_PROTOCOL_CMD_STOP,
        true,
        10U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_AMBIGUOUS);
    expect_event_result(
        139U,
        CAN_PROTOCOL_CMD_STOP,
        true,
        10U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_STALE);
    expect_event_result(
        11U,
        (can_protocol_command_id_t)6U,
        true,
        10U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_INVALID);
    expect_event_result(
        11U,
        (can_protocol_command_id_t)-1,
        true,
        10U,
        CAN_PROTOCOL_CMD_UP,
        CAN_PROTOCOL_EVENT_INVALID);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_STARTUP_LOCKED,
        0xFFFFU,
        0x00U);
    expect_valid_status(&frame);

    frame = make_status_frame(
        32U,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        0x0123U,
        0x03U);
    expect_valid_status(&frame);

    frame = make_status_frame(
        33U,
        CAN_PROTOCOL_STATE_DOWN,
        CAN_PROTOCOL_FAULT_NONE,
        1000U,
        0x03U);
    expect_valid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        0xFFFFU,
        0x00U);
    frame.id++;
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        0xFFFFU,
        0x00U);
    frame.is_extended = true;
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        0xFFFFU,
        0x00U);
    frame.is_remote = true;
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        0xFFFFU,
        0x00U);
    frame.dlc--;
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        0xFFFFU,
        0x00U);
    frame.data[0]++;
    expect_invalid_status(&frame);

    frame = make_status_frame(31U, 3U, CAN_PROTOCOL_FAULT_NONE, 0xFFFFU, 0U);
    expect_invalid_status(&frame);

    frame = make_status_frame(31U, CAN_PROTOCOL_STATE_STOP,
                             CAN_PROTOCOL_FAULT_MOTOR_LOCAL, 0xFFFFU, 0U);
    expect_valid_status(&frame);
    frame = make_status_frame(31U, CAN_PROTOCOL_STATE_STOP, 5U, 0xFFFFU, 0U);
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        0xFFFFU,
        0x04U);
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        0xFFFFU,
        0x04U);
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        0xFFFFU,
        0x00U);
    frame.data[7] = 1U;
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        100U,
        0x00U);
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_STOP,
        CAN_PROTOCOL_FAULT_NONE,
        1001U,
        0x01U);
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_COMM_TIMEOUT,
        0xFFFFU,
        0U);
    expect_invalid_status(&frame);

    frame = make_status_frame(
        31U,
        CAN_PROTOCOL_STATE_UP,
        CAN_PROTOCOL_FAULT_NONE,
        100U,
        0x01U);
    expect_invalid_status(&frame);

    assert(!can_protocol_decode_status(NULL, NULL));
    test_command_encoding();
    test_status_encoding();
    test_invalid_encodings_preserve_output();
    puts("CAN protocol checks passed");
    return 0;
}
