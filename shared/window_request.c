#include "window_request.h"

#include <stddef.h>

bool window_request_command_is_valid(window_request_command_t command)
{
    return (command == WINDOW_REQUEST_UP) ||
           (command == WINDOW_REQUEST_DOWN) ||
           (command == WINDOW_REQUEST_STOP) ||
           (command == WINDOW_REQUEST_CLEAR) ||
           (command == WINDOW_REQUEST_DEMO_SET_ZERO) ||
           (command == WINDOW_REQUEST_DEMO_TOGGLE);
}

void window_request_init(window_request_queue_t *queue)
{
    if (queue != NULL) {
        *queue = (window_request_queue_t){0};
    }
}

bool window_request_submit(window_request_queue_t *queue,
                           window_request_t request)
{
    if ((queue == NULL) || !window_request_command_is_valid(request.command)) {
        return false;
    }

    if (queue->stop_latched) {
        ++queue->rejected_while_latched_count;
        return false;
    }

    if (request.command == WINDOW_REQUEST_STOP) {
        if (queue->latest_valid) {
            ++queue->overwritten_count;
        }
        queue->latest = (window_request_t){0};
        queue->latest_valid = false;
        queue->latched_stop = request;
        queue->stop_latched = true;
        queue->stop_acknowledged = false;
        return true;
    }

    if (queue->latest_valid) {
        ++queue->overwritten_count;
    }
    queue->latest = request;
    queue->latest_valid = true;
    return true;
}

bool window_request_peek(const window_request_queue_t *queue,
                         window_request_t *request_out)
{
    if ((queue == NULL) || (request_out == NULL)) {
        return false;
    }

    if (queue->stop_latched) {
        /* Keep the same safe STOP alive even after application confirmation.
         * Confirmation permits explicit rearm; it does not stop supervision. */
        *request_out = queue->latched_stop;
        return true;
    }

    if (!queue->latest_valid) {
        return false;
    }
    *request_out = queue->latest;
    return true;
}

bool window_request_confirm_stop(window_request_queue_t *queue,
                                 uint8_t sequence)
{
    if ((queue == NULL) || !queue->stop_latched ||
        (queue->latched_stop.sequence != sequence)) {
        return false;
    }

    queue->stop_acknowledged = true;
    queue->latest_valid = false;
    return true;
}

bool window_request_revoke_stop_confirmation(window_request_queue_t *queue)
{
    if ((queue == NULL) || !queue->stop_latched ||
        !queue->stop_acknowledged) {
        return false;
    }

    queue->stop_acknowledged = false;
    return true;
}

bool window_request_rearm(window_request_queue_t *queue)
{
    if ((queue == NULL) || !queue->stop_latched ||
        !queue->stop_acknowledged) {
        return false;
    }

    queue->stop_latched = false;
    queue->stop_acknowledged = false;
    queue->latest_valid = false;
    queue->latest = (window_request_t){0};
    queue->latched_stop = (window_request_t){0};
    return true;
}
