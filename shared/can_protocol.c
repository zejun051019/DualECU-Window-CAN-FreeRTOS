#include "can_protocol.h"

#include <stddef.h>

bool can_protocol_decode_command(
    const can_protocol_frame_t *frame,
    can_protocol_command_t *command_out)
{
    can_protocol_command_t decoded;
    uint8_t command_id;

    if ((frame == NULL) || (command_out == NULL))
    {
        return false;
    }

    if ((frame->id != CAN_PROTOCOL_COMMAND_ID) ||
        frame->is_extended ||
        frame->is_remote ||
        (frame->dlc != CAN_PROTOCOL_COMMAND_DLC))
    {
        return false;
    }

    command_id = frame->data[2];
    if ((frame->data[0] != CAN_PROTOCOL_VERSION) ||
        (command_id > CAN_PROTOCOL_CMD_DEMO_TOGGLE) ||
        (frame->data[3] != 0U))
    {
        return false;
    }

    decoded.sequence = frame->data[1];
    decoded.command = (can_protocol_command_id_t)command_id;
    *command_out = decoded;
    return true;
}

bool can_protocol_decode_status(
    const can_protocol_frame_t *frame,
    can_protocol_status_t *status_out)
{
    can_protocol_status_t decoded;
    uint8_t flags;
    uint16_t position;

    if ((frame == NULL) || (status_out == NULL))
    {
        return false;
    }

    if ((frame->id != CAN_PROTOCOL_STATUS_ID) ||
        frame->is_extended ||
        frame->is_remote ||
        (frame->dlc != CAN_PROTOCOL_STATUS_DLC))
    {
        return false;
    }

    flags = frame->data[6];
    position = (uint16_t)(((uint16_t)frame->data[5] << 8U) | frame->data[4]);
    if ((frame->data[0] != CAN_PROTOCOL_VERSION) ||
        (frame->data[2] > CAN_PROTOCOL_STATE_DOWN) ||
        (frame->data[3] > CAN_PROTOCOL_FAULT_MOTOR_LOCAL) ||
        ((flags & 0xF8U) != 0U) ||
        (frame->data[7] != 0U))
    {
        return false;
    }

    decoded.is_calibrated = ((flags & 0x01U) != 0U);
    if ((decoded.is_calibrated && (position > 1000U)) ||
        (!decoded.is_calibrated && (position != 0xFFFFU)) ||
        ((frame->data[3] != CAN_PROTOCOL_FAULT_NONE) &&
         (frame->data[2] != CAN_PROTOCOL_STATE_STOP)) ||
        (((flags & 0x04U) != 0U) &&
         (frame->data[3] != CAN_PROTOCOL_FAULT_CAN_RX_OVERFLOW)) ||
        ((frame->data[2] != CAN_PROTOCOL_STATE_STOP) &&
         ((flags & 0x02U) == 0U)))
    {
        return false;
    }

    decoded.last_sequence = frame->data[1];
    decoded.state = (can_protocol_state_t)frame->data[2];
    decoded.fault = (can_protocol_fault_t)frame->data[3];
    decoded.position = position;
    decoded.last_sequence_valid = ((flags & 0x02U) != 0U);
    decoded.rx_overflow_active = ((flags & 0x04U) != 0U);
    *status_out = decoded;
    return true;
}

bool can_protocol_encode_command(
    const can_protocol_command_t *command,
    can_protocol_frame_t *frame_out)
{
    can_protocol_frame_t encoded = {0};

    if ((command == NULL) || (frame_out == NULL) ||
        (command->command < CAN_PROTOCOL_CMD_STOP) ||
        (command->command > CAN_PROTOCOL_CMD_DEMO_TOGGLE))
    {
        return false;
    }

    encoded.id = CAN_PROTOCOL_COMMAND_ID;
    encoded.dlc = CAN_PROTOCOL_COMMAND_DLC;
    encoded.data[0] = CAN_PROTOCOL_VERSION;
    encoded.data[1] = command->sequence;
    encoded.data[2] = (uint8_t)command->command;
    *frame_out = encoded;
    return true;
}

bool can_protocol_encode_status(
    const can_protocol_status_t *status,
    can_protocol_frame_t *frame_out)
{
    can_protocol_frame_t encoded = {0};
    can_protocol_status_t validated;
    uint8_t flags = 0U;

    if ((status == NULL) || (frame_out == NULL) ||
        (status->state < CAN_PROTOCOL_STATE_STOP) ||
        (status->state > CAN_PROTOCOL_STATE_DOWN) ||
        (status->fault < CAN_PROTOCOL_FAULT_NONE) ||
        (status->fault > CAN_PROTOCOL_FAULT_MOTOR_LOCAL))
    {
        return false;
    }

    encoded.id = CAN_PROTOCOL_STATUS_ID;
    encoded.dlc = CAN_PROTOCOL_STATUS_DLC;
    encoded.data[0] = CAN_PROTOCOL_VERSION;
    encoded.data[1] = status->last_sequence;
    encoded.data[2] = (uint8_t)status->state;
    encoded.data[3] = (uint8_t)status->fault;
    encoded.data[4] = (uint8_t)(status->position & 0xFFU);
    encoded.data[5] = (uint8_t)(status->position >> 8U);
    if (status->is_calibrated)
    {
        flags |= 0x01U;
    }
    if (status->last_sequence_valid)
    {
        flags |= 0x02U;
    }
    if (status->rx_overflow_active)
    {
        flags |= 0x04U;
    }
    encoded.data[6] = flags;

    if (!can_protocol_decode_status(&encoded, &validated))
    {
        return false;
    }

    *frame_out = encoded;
    return true;
}

can_protocol_event_result_t can_protocol_classify_event(
    const can_protocol_command_t *command,
    bool last_sequence_valid,
    uint8_t last_sequence,
    can_protocol_command_id_t last_command)
{
    uint8_t sequence_delta;

    if ((command == NULL) ||
        (command->command < CAN_PROTOCOL_CMD_STOP) ||
        (command->command > CAN_PROTOCOL_CMD_DEMO_TOGGLE))
    {
        return CAN_PROTOCOL_EVENT_INVALID;
    }

    if (!last_sequence_valid)
    {
        return (command->command == CAN_PROTOCOL_CMD_STOP)
                   ? CAN_PROTOCOL_EVENT_BASELINE_STOP
                   : CAN_PROTOCOL_EVENT_BASELINE_REQUIRED;
    }

    if ((last_command < CAN_PROTOCOL_CMD_STOP) ||
        (last_command > CAN_PROTOCOL_CMD_DEMO_TOGGLE))
    {
        return CAN_PROTOCOL_EVENT_INVALID;
    }

    sequence_delta = (uint8_t)(command->sequence - last_sequence);
    if (sequence_delta == 0U)
    {
        return (command->command == last_command)
                   ? CAN_PROTOCOL_EVENT_REPEAT
                   : CAN_PROTOCOL_EVENT_CONFLICT;
    }

    if (sequence_delta < 128U)
    {
        return CAN_PROTOCOL_EVENT_NEW;
    }

    if (sequence_delta == 128U)
    {
        return CAN_PROTOCOL_EVENT_AMBIGUOUS;
    }

    return CAN_PROTOCOL_EVENT_STALE;
}
