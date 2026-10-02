#include "../../shared/can_test_frame.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    can_protocol_frame_t frame = {0};
    uint32_t counter = 0U;
    can_test_t04_action_t action = CAN_TEST_T04_ACTION_INJECT_OVERFLOW;

    can_test_frame_encode(CAN_TEST_FRAME_G3507_TO_F407_ID, 1U, &frame);
    assert(frame.id == CAN_TEST_FRAME_G3507_TO_F407_ID);
    assert(can_test_frame_decode(
        &frame, CAN_TEST_FRAME_G3507_TO_F407_ID, &counter));
    assert(counter == 1U);

    frame.data[7] ^= 1U;
    assert(!can_test_frame_decode(
        &frame, CAN_TEST_FRAME_G3507_TO_F407_ID, &counter));

    assert(can_test_frame_encode_t04_control(
        CAN_TEST_T04_ACTION_INJECT_OVERFLOW, &frame));
    assert(can_test_frame_decode_t04_control(&frame, &action));
    assert(action == CAN_TEST_T04_ACTION_INJECT_OVERFLOW);

    assert(can_test_frame_encode_t04_control(
        CAN_TEST_T04_ACTION_RELEASE_OVERFLOW, &frame));
    assert(can_test_frame_decode_t04_control(&frame, &action));
    assert(action == CAN_TEST_T04_ACTION_RELEASE_OVERFLOW);

    frame.data[7] ^= 1U;
    assert(!can_test_frame_decode_t04_control(&frame, &action));
    assert(!can_test_frame_encode_t04_control(
        (can_test_t04_action_t)3U, &frame));

    puts("CAN test frame checks passed");
    return 0;
}
