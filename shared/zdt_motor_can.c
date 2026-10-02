#include "zdt_motor_can.h"

#include <stddef.h>
#include <string.h>

bool zdt_motor_can_encode_read_position(
    uint8_t address,
    can_protocol_frame_t *frame_out)
{
    can_protocol_frame_t frame = {0};

    if ((address == 0U) || (frame_out == NULL))
    {
        return false;
    }

    frame.id = ((uint32_t)address) << 8U;
    frame.dlc = 2U;
    frame.is_extended = true;
    frame.data[0] = ZDT_MOTOR_CAN_READ_POSITION_CODE;
    frame.data[1] = ZDT_MOTOR_CAN_CHECKSUM;

    *frame_out = frame;
    return true;
}

bool zdt_motor_can_decode_position_response(
    uint8_t address,
    const can_protocol_frame_t *frame,
    zdt_motor_position_t *position_out)
{
    zdt_motor_position_t position = {0};

    if ((address == 0U) || (frame == NULL) || (position_out == NULL) ||
        !frame->is_extended || frame->is_remote ||
        (frame->id != (((uint32_t)address) << 8U)) ||
        (frame->dlc != ZDT_MOTOR_CAN_POSITION_REPLY_DLC) ||
        (frame->data[0] != ZDT_MOTOR_CAN_READ_POSITION_CODE) ||
        (frame->data[1] > 1U) ||
        (frame->data[6] != ZDT_MOTOR_CAN_CHECKSUM))
    {
        return false;
    }

    position.negative = (frame->data[1] == 1U);
    position.raw_position = ((uint32_t)frame->data[2] << 24U) |
                            ((uint32_t)frame->data[3] << 16U) |
                            ((uint32_t)frame->data[4] << 8U) |
                            (uint32_t)frame->data[5];

    *position_out = position;
    return true;
}
