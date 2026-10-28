#ifndef PEDAL_BUTTON_H
#define PEDAL_BUTTON_H

// Configure button 0; a button already held at startup must be released first.
int pedal_button_init(void);

/*
 * Consume a pending interrupt/debounce event from the replay thread.
 * Returns: 1 once per debounced press, 0 otherwise, or a negative GPIO error.
 * Both press and release must remain stable for 30 ms; holding does not repeat.
 */
int pedal_button_poll(void);

// Block without polling until a debounced press is available.
void pedal_button_wait(void);

#endif
