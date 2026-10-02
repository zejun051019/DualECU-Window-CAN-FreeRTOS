#ifndef CAN_PORT_MSPM0_H
#define CAN_PORT_MSPM0_H

#include "can_protocol.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAN_PORT_MSPM0_RX_EMPTY = 0U,
    CAN_PORT_MSPM0_RX_FRAME,
    CAN_PORT_MSPM0_RX_ACK_ERROR
} can_port_mspm0_rx_result_t;

bool can_port_mspm0_init(void);

/* Poll without waiting; true means bus-off recovery is still in progress. */
bool can_port_mspm0_service_bus_off(uint32_t now_ms);

bool can_port_mspm0_send_frame(const can_protocol_frame_t *frame);

can_port_mspm0_rx_result_t can_port_mspm0_receive(
    can_protocol_frame_t *frame_out,
    uint16_t *timestamp_out);

uint16_t can_port_mspm0_timestamp_now(void);

bool can_port_mspm0_rx_overflow_pending(void);

bool can_port_mspm0_clear_rx_overflow_if_drained(void);

#endif /* CAN_PORT_MSPM0_H */
