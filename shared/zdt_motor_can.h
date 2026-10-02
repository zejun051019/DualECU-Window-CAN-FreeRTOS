#ifndef SHARED_ZDT_MOTOR_CAN_H
#define SHARED_ZDT_MOTOR_CAN_H

#include "can_protocol.h"

#define ZDT_MOTOR_CAN_READ_POSITION_CODE (0x36U)
#define ZDT_MOTOR_CAN_CHECKSUM           (0x6BU)
#define ZDT_MOTOR_CAN_POSITION_REPLY_DLC (7U)
#define ZDT_MOTOR_CAN_DEFAULT_ADDRESS    (1U)

typedef struct
{
    bool negative;
    uint32_t raw_position;
} zdt_motor_position_t;

bool zdt_motor_can_encode_read_position(
    uint8_t address,
    can_protocol_frame_t *frame_out);

bool zdt_motor_can_decode_position_response(
    uint8_t address,
    const can_protocol_frame_t *frame,
    zdt_motor_position_t *position_out);

#endif /* SHARED_ZDT_MOTOR_CAN_H */
