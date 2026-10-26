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

#endif // PEDAL_COMMUNICATION_H
