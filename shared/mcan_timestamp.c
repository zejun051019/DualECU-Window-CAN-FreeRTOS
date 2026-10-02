#include "mcan_timestamp.h"

uint32_t mcan_timestamp_frame_received_at_ms(
    uint32_t processed_at_ms,
    uint16_t timestamp_now,
    uint16_t frame_timestamp)
{
    const uint32_t elapsed_ticks = (uint16_t)(timestamp_now - frame_timestamp);
    const uint32_t elapsed_ns = elapsed_ticks * MCAN_RX_TIMESTAMP_TICK_NS;
    const uint32_t elapsed_ms = (elapsed_ns + 999999U) / 1000000U;

    /* Round age up so a queued frame is never credited as newer than its SOF. */
    return processed_at_ms - elapsed_ms;
}
