#ifndef PEDAL_ACTIVITY_H
#define PEDAL_ACTIVITY_H

#include <stdbool.h>

// Set to 0  to keep all LEDs off and disable their scheduled work.
#ifndef PEDAL_ACTIVITY_LEDS_ENABLED
#define PEDAL_ACTIVITY_LEDS_ENABLED 1
#endif

#if PEDAL_ACTIVITY_LEDS_ENABLED != 0 && PEDAL_ACTIVITY_LEDS_ENABLED != 1
#error "PEDAL_ACTIVITY_LEDS_ENABLED must be 0 or 1"
#endif

// Disconnect unused GPIOs, then configure LED0 through LED3.
int pedal_activity_init(void);

// Blink LED0 for standard or LED1 for simplified; the other mode LED stays off.
void pedal_activity_processing_start(bool simplified);
void pedal_activity_processing_stop(void);

// Pulse LED2 after a complete revolution.
void pedal_activity_revolution(void);

// Pulse LED3 once after a complete lookup table; independent of UART output.
void pedal_activity_table_complete(void);

#endif // PEDAL_ACTIVITY_H
