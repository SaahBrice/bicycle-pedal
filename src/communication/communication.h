#ifndef PEDAL_COMMUNICATION_H
#define PEDAL_COMMUNICATION_H

typedef enum {
    PEDAL_MESSAGE_PLAIN,
    PEDAL_MESSAGE_SUCCESS,
    PEDAL_MESSAGE_WARNING,
    PEDAL_MESSAGE_ERROR,
} PedalMessageLevel;

// Prepare interrupt-driven UART transmission; returns 0 or negative errno.
int pedal_communication_init(void);

// Queue one leveled UART line without waiting for transmission; returns 0 or negative errno.
int pedal_communication_send(PedalMessageLevel level, const char *message);

// Sleep until queued bytes finish and runtime PM suspends UART, or timeout.
// Called from the reporting thread; timeout_ms must be positive.
int pedal_communication_wait_idle(int timeout_ms);

#endif // PEDAL_COMMUNICATION_H
