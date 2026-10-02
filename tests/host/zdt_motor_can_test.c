#include "../../shared/zdt_motor_can.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CHECK(condition)                                                     \
    do                                                                       \
    {                                                                        \
        if (!(condition))                                                    \
        {                                                                    \
            (void)fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__,       \
                          #condition);                                       \
            return false;                                                    \
        }                                                                    \
    } while (0)

static bool test_frame_preserves_full_extended_id(void)
{
    can_protocol_frame_t frame = {0};
    const uint32_t expectedId = 0x1ABCDEFFU;

    frame.id = expectedId;
    CHECK((uint32_t)frame.id == expectedId);
    return true;
}

static bool test_position_query_matches_manual(void)
{
    can_protocol_frame_t frame = {0};

    CHECK(zdt_motor_can_encode_read_position(1U, &frame));
    CHECK(frame.id == 0x100U);
    CHECK(frame.is_extended);
    CHECK(!frame.is_remote);
    CHECK(frame.dlc == 2U);
    CHECK(frame.data[0] == 0x36U);
    CHECK(frame.data[1] == 0x6BU);
    return true;
}

static bool test_position_response_decodes_signed_raw_value(void)
{
    const can_protocol_frame_t frame = {
        .id = 0x100U,
        .dlc = 7U,
        .is_extended = true,
        .is_remote = false,
        .data = {0x36U, 0x01U, 0x00U, 0x00U, 0x00U, 0x10U, 0x6BU}};
    zdt_motor_position_t position = {0};

    CHECK(zdt_motor_can_decode_position_response(1U, &frame, &position));
    CHECK(position.negative);
    CHECK(position.raw_position == 16U);
    return true;
}

static bool test_position_response_rejects_wrong_format(void)
{
    can_protocol_frame_t frame = {
        .id = 0x100U,
        .dlc = 7U,
        .is_extended = true,
        .is_remote = false,
        .data = {0x36U, 0x00U, 0x00U, 0x00U, 0x00U, 0x10U, 0x6BU}};
    zdt_motor_position_t position = {0};

    frame.is_extended = false;
    CHECK(!zdt_motor_can_decode_position_response(1U, &frame, &position));
    frame.is_extended = true;
    frame.data[6] = 0x00U;
    CHECK(!zdt_motor_can_decode_position_response(1U, &frame, &position));
    frame.data[6] = 0x6BU;
    frame.dlc = 6U;
    CHECK(!zdt_motor_can_decode_position_response(1U, &frame, &position));
    frame.dlc = 7U;
    frame.data[1] = 0x02U;
    CHECK(!zdt_motor_can_decode_position_response(1U, &frame, &position));
    return true;
}

static bool test_position_query_rejects_broadcast_address(void)
{
    can_protocol_frame_t frame = {0};

    CHECK(!zdt_motor_can_encode_read_position(0U, &frame));
    return true;
}

int main(void)
{
    if (!test_frame_preserves_full_extended_id() ||
        !test_position_query_matches_manual() ||
        !test_position_response_decodes_signed_raw_value() ||
        !test_position_response_rejects_wrong_format() ||
        !test_position_query_rejects_broadcast_address())
    {
        return 1;
    }

    (void)puts("ZDT CAN position checks passed");
    return 0;
}
