#include "activity/activity.h"
#include "activity/gpio/gpio.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define PEDAL_PROCESSING_BLINK_MS 100u // [ms] Selected mode LED half-period during replay.
#define PEDAL_REVOLUTION_PULSE_MS 120u // [ms] Visible LED2 revolution pulse.
#define PEDAL_TABLE_PULSE_MS 120u       // [ms] LED3 pulse after a complete table.

static const struct gpio_dt_spec standard_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec simplified_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec *processing_led = &standard_led;
static const struct gpio_dt_spec revolution_led = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec table_led = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static bool processing_led_on;
static bool initialized;
static struct k_work_sync processing_stop_sync;

/* ------------------------------------------------------------------ */
/* Creates: static void processing_blink(struct k_work *work)         */
/*                                                                    */
/* Description:                                                       */
/*   Toggles the selected mode LED during replay and schedules its next edge. */
/*                                                                    */
/* Input: work - owning delayable work item                           */
/*                                                                    */
/* Output: void - updates the mode LED and its work schedule                  */
/* ------------------------------------------------------------------ */
static void processing_blink(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(processing_work, processing_blink);

static void processing_blink(struct k_work *work)
{
    (void)work;
    processing_led_on = !processing_led_on;
    gpio_pin_set_dt(processing_led, processing_led_on);
    k_work_reschedule(&processing_work, K_MSEC(PEDAL_PROCESSING_BLINK_MS));
}

/* ------------------------------------------------------------------ */
/* Creates: static void revolution_pulse_end(struct k_work *work)    */
/*                                                                    */
/* Description:                                                       */
/*   Turns LED2 off after the visible revolution pulse.              */
/*                                                                    */
/* Input: work - owning delayable work item                           */
/*                                                                    */
/* Output: void - turns LED2 off                                      */
/* ------------------------------------------------------------------ */
static void revolution_pulse_end(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(revolution_work, revolution_pulse_end);

static void revolution_pulse_end(struct k_work *work)
{
    (void)work;
    gpio_pin_set_dt(&revolution_led, 0);
}

/* ------------------------------------------------------------------ */
/* Creates: static void table_pulse_end(struct k_work *work)          */
/*                                                                    */
/* Description:                                                       */
/*   Turns LED3 off after the table-completion pulse.      */
/*                                                                    */
/* Input: work - owning delayable work item                           */
/*                                                                    */
/* Output: void - turns LED3 off                                      */
/* ------------------------------------------------------------------ */
static void table_pulse_end(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(table_work, table_pulse_end);

static void table_pulse_end(struct k_work *work)
{
    (void)work;
    gpio_pin_set_dt(&table_led, 0);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_activity_init(void)                             */
/*                                                                    */
/* Description:                                                       */
/*   Checks and configures all four board LEDs as inactive outputs.  */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: int - 0 or a negative GPIO/device error                    */
/* ------------------------------------------------------------------ */
int pedal_activity_init(void)
{
    int cleanup_status = pedal_gpio_cleanup();
    if (cleanup_status < 0) {
        return cleanup_status;
    }
    if (!gpio_is_ready_dt(&standard_led) ||
        !gpio_is_ready_dt(&simplified_led) ||
        !gpio_is_ready_dt(&revolution_led) ||
        !gpio_is_ready_dt(&table_led)) {
        return -ENODEV;
    }

    // Power profiling never uses LEDs. Disconnect their pins so no LED GPIO
    // remains driven while the CPU waits for the next sample or button press.
    if (IS_ENABLED(CONFIG_PEDAL_POWER_BUILD)) {
        const struct gpio_dt_spec *leds[] = {
            &standard_led, &simplified_led, &revolution_led, &table_led,
        };
        for (size_t index = 0; index < ARRAY_SIZE(leds); ++index) {
            int led_status = gpio_pin_configure_dt(leds[index], GPIO_DISCONNECTED);
            if (led_status < 0) {
                return led_status;
            }
        }
        initialized = false;
        return 0;
    }

    int standard_led_status = gpio_pin_configure_dt(&standard_led, GPIO_OUTPUT_INACTIVE);
    if (standard_led_status < 0) {
        return standard_led_status;
    }
    int simplified_led_status = gpio_pin_configure_dt(&simplified_led, GPIO_OUTPUT_INACTIVE);
    if (simplified_led_status < 0) {
        return simplified_led_status;
    }
    int revolution_led_status = gpio_pin_configure_dt(&revolution_led, GPIO_OUTPUT_INACTIVE);
    if (revolution_led_status < 0) {
        return revolution_led_status;
    }
    int table_led_status = gpio_pin_configure_dt(&table_led, GPIO_OUTPUT_INACTIVE);
    if (table_led_status < 0) {
        return table_led_status;
    }
    initialized = true;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_processing_start(bool simplified)                */
/*                                                                    */
/* Description:                                                       */
/*   Blinks LED0 for standard or LED1 for simplified replay.           */
/*                                                                    */
/* Input: simplified - true for LED1; false for LED0                   */
/*                                                                    */
/* Output: void - selects the mode LED and schedules blinking          */
/* ------------------------------------------------------------------ */
void pedal_activity_processing_start(bool simplified)
{
    if (!PEDAL_ACTIVITY_LEDS_ENABLED || !initialized) {
        return;
    }
    pedal_activity_processing_stop();
    processing_led = simplified ? &simplified_led : &standard_led;
    processing_led_on = true;
    gpio_pin_set_dt(processing_led, 1);
    k_work_reschedule(&processing_work, K_MSEC(PEDAL_PROCESSING_BLINK_MS));
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_processing_stop(void)                 */
/*                                                                    */
/* Description:                                                       */
/*   Stops mode blinking when replay ends or switches mode.                    */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: void - cancels blinking and turns both mode LEDs off                 */
/* ------------------------------------------------------------------ */
void pedal_activity_processing_stop(void)
{
    if (!PEDAL_ACTIVITY_LEDS_ENABLED || !initialized) {
        return;
    }
    k_work_cancel_delayable_sync(&processing_work, &processing_stop_sync);
    processing_led_on = false;
    gpio_pin_set_dt(&standard_led, 0);
    gpio_pin_set_dt(&simplified_led, 0);
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_revolution(void)                      */
/*                                                                    */
/* Description:                                                       */
/*   Starts or extends a visible LED2 pulse for a completed turn.    */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: void - lights LED2 and schedules its turn-off              */
/* ------------------------------------------------------------------ */
void pedal_activity_revolution(void)
{
    if (!PEDAL_ACTIVITY_LEDS_ENABLED || !initialized) {
        return;
    }
    gpio_pin_set_dt(&revolution_led, 1);
    k_work_reschedule(&revolution_work, K_MSEC(PEDAL_REVOLUTION_PULSE_MS));
}

/*
 * Pulse LED3 once for a completed lookup table, independently of UART traffic.
 * The next replay may start while this short completion pulse is visible.
 */
void pedal_activity_table_complete(void)
{
    if (!PEDAL_ACTIVITY_LEDS_ENABLED || !initialized) {
        return;
    }
    gpio_pin_set_dt(&table_led, 1);
    k_work_reschedule(&table_work, K_MSEC(PEDAL_TABLE_PULSE_MS));
}
