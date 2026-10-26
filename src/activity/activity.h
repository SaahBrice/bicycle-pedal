#ifndef PEDAL_ACTIVITY_H
#define PEDAL_ACTIVITY_H

// Configure LED0, LED1, and LED3 as activity indicators.
int pedal_activity_init(void);

// Blink LED0 while sample replay is running.
void pedal_activity_processing_start(void);
void pedal_activity_processing_stop(void);

// Pulse LED1 after a complete revolution.
void pedal_activity_revolution(void);

// Light LED3 while UART output is pending, then finish its visible pulse.
void pedal_activity_uart_start(void);
void pedal_activity_uart_idle(void);

#endif // PEDAL_ACTIVITY_H
