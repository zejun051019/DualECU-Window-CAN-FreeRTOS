#ifndef SHARED_WINDOW_REQUEST_H
#define SHARED_WINDOW_REQUEST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    WINDOW_REQUEST_NONE = 0U,
    WINDOW_REQUEST_UP,
    WINDOW_REQUEST_DOWN,
    WINDOW_REQUEST_STOP,
    WINDOW_REQUEST_CLEAR,
    WINDOW_REQUEST_DEMO_SET_ZERO,
    WINDOW_REQUEST_DEMO_TOGGLE
} window_request_command_t;

typedef struct
{
    window_request_command_t command;
    uint8_t sequence;
} window_request_t;

typedef struct
{
    window_request_t latest;
    window_request_t latched_stop;
    bool latest_valid;
    bool stop_latched;
    bool stop_acknowledged;
    uint32_t overwritten_count;
    uint32_t rejected_while_latched_count;
} window_request_queue_t;

void window_request_init(window_request_queue_t *queue);
bool window_request_command_is_valid(window_request_command_t command);
bool window_request_submit(window_request_queue_t *queue,
                           window_request_t request);
bool window_request_peek(const window_request_queue_t *queue,
                         window_request_t *request_out);
bool window_request_confirm_stop(window_request_queue_t *queue,
                                 uint8_t sequence);
bool window_request_revoke_stop_confirmation(window_request_queue_t *queue);
bool window_request_rearm(window_request_queue_t *queue);

#endif
