#include "../../shared/mcan_timestamp.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

int main(void)
{
    assert(mcan_timestamp_frame_received_at_ms(100U, 123U, 123U) == 100U);
    assert(mcan_timestamp_frame_received_at_ms(100U, 124U, 123U) == 99U);
    assert(mcan_timestamp_frame_received_at_ms(100U, 157U, 124U) == 99U);
    assert(mcan_timestamp_frame_received_at_ms(100U, 158U, 124U) == 98U);
    assert(mcan_timestamp_frame_received_at_ms(1000U, 10000U, 0U) == 700U);

    /* 16-bit MCAN and 32-bit system time may wrap independently. */
    assert(mcan_timestamp_frame_received_at_ms(1000U, 5U, 65530U) == 999U);
    assert(mcan_timestamp_frame_received_at_ms(0U, 10U, 9U) == UINT32_MAX);

    puts("MCAN receive timestamp checks passed");
    return 0;
}
