#include "activity/button/button.h"

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define PEDAL_BUTTON_DEBOUNCE_MS 30
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static int candidate_state;
static int stable_state;
static int64_t changed_ms;

int pedal_button_init(void)
{
    if (!gpio_is_ready_dt(&button)) {
        return -ENODEV;
    }
    int rc = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (rc < 0) {
        return rc;
    }
    rc = gpio_pin_get_dt(&button);
    if (rc < 0) {
        return rc;
    }
    candidate_state = stable_state = rc;
    changed_ms = k_uptime_get();
    return 0;
}

int pedal_button_poll(void)
{
    const int state = gpio_pin_get_dt(&button);
    if (state < 0) {
        return state;
    }
    const int64_t now = k_uptime_get();
    if (state != candidate_state) {
        candidate_state = state;
        changed_ms = now;
    }
    if (candidate_state != stable_state &&
        now - changed_ms >= PEDAL_BUTTON_DEBOUNCE_MS) {
        stable_state = candidate_state;
        return stable_state ? 1 : 0;
    }
    return 0;
}
