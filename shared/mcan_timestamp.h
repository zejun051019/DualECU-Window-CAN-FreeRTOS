#ifndef SHARED_MCAN_TIMESTAMP_H
#define SHARED_MCAN_TIMESTAMP_H

#include <stdint.h>

/* 500 kbit/s CAN bit time multiplied by the configured 15-bit-time prescaler. */
#define MCAN_RX_TIMESTAMP_TICK_NS (30000U)

/*
 * Convert an MCAN 16-bit start-of-frame timestamp into the application's
 * millisecond clock. The caller must service a received element before the
 * 16-bit counter has completed a full wrap.
 */
uint32_t mcan_timestamp_frame_received_at_ms(
    uint32_t processed_at_ms,
    uint16_t timestamp_now,
    uint16_t frame_timestamp);

#endif /* SHARED_MCAN_TIMESTAMP_H */
