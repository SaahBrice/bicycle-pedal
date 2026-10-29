#include "activity/button/button.h"

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#define PEDAL_BUTTON_DEBOUNCE_MS 30
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
K_SEM_DEFINE(button_event, 0, 1);
static int stable_state;
static atomic_t pending_presses;
static atomic_t read_error;
static struct gpio_callback button_callback;

// Sample the logical pin value only after the contact has settled.
static void debounce(struct k_work *work)
{
    (void)work;
    const int state = gpio_pin_get_dt(&button);
    if (state < 0) {
        atomic_set(&read_error, state);
        return;
    }
    if (state != stable_state) {
        stable_state = state;
        if (state) {
            atomic_inc(&pending_presses);
            k_sem_give(&button_event);
        }
    }
}

K_WORK_DELAYABLE_DEFINE(debounce_work, debounce);

static void button_changed(const struct device *port, struct gpio_callback *cb,
                           gpio_port_pins_t pins)
{
    (void)port;
    (void)cb;
    (void)pins;
    // Each bounce restarts the settling interval; no periodic polling is needed.
    k_work_reschedule(&debounce_work, K_MSEC(PEDAL_BUTTON_DEBOUNCE_MS));
}

int pedal_button_init(void)
{
    if (!gpio_is_ready_dt(&button)) {
        return -ENODEV;
    }
    int configure_status = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (configure_status < 0) {
        return configure_status;
    }
    int button_state = gpio_pin_get_dt(&button);
    if (button_state < 0) {
        return button_state;
    }
    stable_state = button_state;
    atomic_clear(&pending_presses);
    atomic_clear(&read_error);
    gpio_init_callback(&button_callback, button_changed, BIT(button.pin));
    int callback_status = gpio_add_callback(button.port, &button_callback);
    if (callback_status < 0) {
        return callback_status;
    }
    int interrupt_status = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    if (interrupt_status < 0) {
        gpio_remove_callback(button.port, &button_callback);
    }
    return interrupt_status;
}

int pedal_button_poll(void)
{
    const int error = atomic_set(&read_error, 0);
    if (error < 0) {
        return error;
    }
    // Consume one event; an unpressed button costs no GPIO or clock reads.
    atomic_val_t count;
    do {
        count = atomic_get(&pending_presses);
        if (count == 0) {
            return 0;
        }
    } while (!atomic_cas(&pending_presses, count, count - 1));
    return 1;
}

void pedal_button_wait(void)
{
    k_sem_take(&button_event, K_FOREVER);
}
