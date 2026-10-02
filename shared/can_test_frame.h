#ifndef SHARED_CAN_TEST_FRAME_H
#define SHARED_CAN_TEST_FRAME_H

#include "can_protocol.h"
#include <stddef.h>

/* Task 2 probes use distinct IDs so simultaneous TX cannot share an ID. */
#define CAN_TEST_FRAME_F407_TO_G3507_ID (0x120U)
#define CAN_TEST_FRAME_G3507_TO_F407_ID (0x121U)
#define CAN_TEST_FRAME_COUNT            (1000U)

typedef enum
{
    CAN_TEST_T04_ACTION_INJECT_OVERFLOW = 1U,
    CAN_TEST_T04_ACTION_RELEASE_OVERFLOW = 2U
} can_test_t04_action_t;

static inline bool can_test_frame_encode_t04_control(
    can_test_t04_action_t action,
    can_protocol_frame_t *frame)
{
    if ((frame == NULL) ||
        ((action != CAN_TEST_T04_ACTION_INJECT_OVERFLOW) &&
         (action != CAN_TEST_T04_ACTION_RELEASE_OVERFLOW)))
    {
        return false;
    }

    frame->id = CAN_TEST_FRAME_F407_TO_G3507_ID;
    frame->dlc = 8U;
    frame->is_extended = false;
    frame->is_remote = false;
    frame->data[0] = 0x54U; /* 'T' */
    frame->data[1] = 0x30U; /* '0' */
    frame->data[2] = 0x34U; /* '4' */
    frame->data[3] = (uint8_t)action;
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        frame->data[i + 4U] = (uint8_t)~frame->data[i];
    }
    return true;
}

static inline bool can_test_frame_decode_t04_control(
    const can_protocol_frame_t *frame,
    can_test_t04_action_t *action_out)
{
    if ((frame == NULL) || (action_out == NULL) ||
        (frame->id != CAN_TEST_FRAME_F407_TO_G3507_ID) ||
        (frame->dlc != 8U) || frame->is_extended || frame->is_remote ||
        (frame->data[0] != 0x54U) || (frame->data[1] != 0x30U) ||
        (frame->data[2] != 0x34U) ||
        ((frame->data[3] != CAN_TEST_T04_ACTION_INJECT_OVERFLOW) &&
         (frame->data[3] != CAN_TEST_T04_ACTION_RELEASE_OVERFLOW)))
    {
        return false;
    }

    for (uint32_t i = 0U; i < 4U; ++i)
    {
        if (frame->data[i + 4U] != (uint8_t)~frame->data[i])
        {
            return false;
        }
    }

    *action_out = (can_test_t04_action_t)frame->data[3];
    return true;
}

static inline void can_test_frame_encode(uint16_t id,
                                         uint32_t counter,
                                         can_protocol_frame_t *frame)
{
    frame->id = id;
    frame->dlc = 8U;
    frame->is_extended = false;
    frame->is_remote = false;
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        frame->data[i] = (uint8_t)(counter >> (8U * i));
        frame->data[i + 4U] = (uint8_t)((~counter) >> (8U * i));
    }
}

static inline bool can_test_frame_decode(const can_protocol_frame_t *frame,
                                         uint16_t expected_id,
                                         uint32_t *counter_out)
{
    uint32_t counter = 0U;
    uint32_t inverted = 0U;

    if ((frame == NULL) || (counter_out == NULL) ||
        (frame->id != expected_id) || (frame->dlc != 8U) ||
        frame->is_extended || frame->is_remote)
    {
        return false;
    }
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        counter |= (uint32_t)frame->data[i] << (8U * i);
        inverted |= (uint32_t)frame->data[i + 4U] << (8U * i);
    }
    if ((counter == 0U) || (counter > CAN_TEST_FRAME_COUNT) ||
        (inverted != ~counter))
    {
        return false;
    }
    *counter_out = counter;
    return true;
}

#endif
