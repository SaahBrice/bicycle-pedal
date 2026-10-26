#include "activity/gpio/gpio.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/pinctrl/nrf-pinctrl.h>
#include <zephyr/sys/util.h>

static const struct gpio_dt_spec used_gpios[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
};

/* Preserve every pin in the selected UART's default pinctrl state. */
#define UART_PSEL(node, prop, idx) DT_PROP_BY_IDX(node, prop, idx),
#define UART_GROUP(node) DT_FOREACH_PROP_ELEM(node, psels, UART_PSEL)
static const uint32_t uart_psels[] = {
    DT_FOREACH_CHILD(DT_PHANDLE(DT_CHOSEN(zephyr_console), pinctrl_0), UART_GROUP)
};

struct pedal_gpio_port {
    const struct device *device;
    uint32_t number;
    gpio_port_pins_t available;
};
#define PEDAL_PORT(node) { DEVICE_DT_GET(node), DT_PROP(node, port), \
    GPIO_DT_PORT_PIN_MASK_NGPIOS_EXC(node, DT_PROP(node, ngpios)) }
static const struct pedal_gpio_port ports[] = {
    PEDAL_PORT(DT_NODELABEL(gpio0)),
    PEDAL_PORT(DT_NODELABEL(gpio1)),
    PEDAL_PORT(DT_NODELABEL(gpio2)),
};

static bool preserve_pin(const struct pedal_gpio_port *port, gpio_pin_t pin)
{
    /* nRF54L15: P1.00/01 are XL1/XL2; P1.02/03 are NFC1/NFC2.
     * Leave their special-function configuration untouched. */
    if (port->number == 1u && pin <= 3u) {
        return true;
    }
    for (size_t i = 0; i < ARRAY_SIZE(used_gpios); ++i) {
        if (used_gpios[i].port == port->device && used_gpios[i].pin == pin) {
            return true;
        }
    }
    for (size_t i = 0; i < ARRAY_SIZE(uart_psels); ++i) {
        const uint32_t absolute_pin = (uart_psels[i] >> NRF_PIN_POS) & NRF_PIN_MSK;
        if (absolute_pin == port->number * 32u + pin) {
            return true;
        }
    }
    return false;
}

int pedal_gpio_cleanup(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(ports); ++i) {
        const struct pedal_gpio_port *port = &ports[i];
        if (!device_is_ready(port->device)) {
            return -ENODEV;
        }
        for (gpio_pin_t pin = 0; pin < 32u; ++pin) {
            if ((port->available & BIT(pin)) == 0u || preserve_pin(port, pin)) {
                continue;
            }
            const int rc = gpio_pin_configure(port->device, pin, GPIO_DISCONNECTED);
            if (rc < 0) {
                return rc;
            }
        }
    }
    return 0;
}
