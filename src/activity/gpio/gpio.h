#ifndef PEDAL_GPIO_H
#define PEDAL_GPIO_H

/*
 * Disconnect unused nRF54L15 DK GPIOs once at startup (no pull, input/output
 * disabled). Preserve LEDs, button 0, console UART pins, and crystal/NFC pins.
 * SWD, reset, and the high-frequency crystal use dedicated non-GPIO pins.
 * Returns: 0 or a negative device/configuration error.
 * This is for the bare DK; update the preserved pins before adding hardware.
 */
int pedal_gpio_cleanup(void);

#endif
