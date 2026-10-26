#include "activity/activity.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define PEDAL_PROCESSING_BLINK_MS 100u // [ms] LED0 half-period during replay.
#define PEDAL_REVOLUTION_PULSE_MS 120u // [ms] Visible LED1 revolution pulse.
#define PEDAL_UART_PULSE_MS 120u       // [ms] Minimum LED3 pulse after UART drains.

static const struct gpio_dt_spec processing_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec revolution_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec uart_led = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static bool processing;
static bool processing_led_on;
static bool initialized;
static struct k_work_sync processing_stop_sync;

/* ------------------------------------------------------------------ */
/* Creates: static void processing_blink(struct k_work *work)         */
/*                                                                    */
/* Description:                                                       */
/*   Toggles LED0 while replay is active and schedules its next edge. */
/*                                                                    */
/* Input: work - owning delayable work item                           */
/*                                                                    */
/* Output: void - updates LED0 and its work schedule                  */
/* ------------------------------------------------------------------ */
static void processing_blink(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(processing_work, processing_blink);

static void processing_blink(struct k_work *work)
{
    (void)work;
    if (!processing) {
        return;
    }
    processing_led_on = !processing_led_on;
    gpio_pin_set_dt(&processing_led, processing_led_on);
    k_work_reschedule(&processing_work, K_MSEC(PEDAL_PROCESSING_BLINK_MS));
}

/* ------------------------------------------------------------------ */
/* Creates: static void revolution_pulse_end(struct k_work *work)    */
/*                                                                    */
/* Description:                                                       */
/*   Turns LED1 off after the visible revolution pulse.              */
/*                                                                    */
/* Input: work - owning delayable work item                           */
/*                                                                    */
/* Output: void - turns LED1 off                                      */
/* ------------------------------------------------------------------ */
static void revolution_pulse_end(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(revolution_work, revolution_pulse_end);

static void revolution_pulse_end(struct k_work *work)
{
    (void)work;
    gpio_pin_set_dt(&revolution_led, 0);
}

/* ------------------------------------------------------------------ */
/* Creates: static void uart_pulse_end(struct k_work *work)          */
/*                                                                    */
/* Description:                                                       */
/*   Turns LED3 off shortly after the UART output queue drains.      */
/*                                                                    */
/* Input: work - owning delayable work item                           */
/*                                                                    */
/* Output: void - turns LED3 off                                      */
/* ------------------------------------------------------------------ */
static void uart_pulse_end(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(uart_work, uart_pulse_end);

static void uart_pulse_end(struct k_work *work)
{
    (void)work;
    gpio_pin_set_dt(&uart_led, 0);
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_activity_init(void)                             */
/*                                                                    */
/* Description:                                                       */
/*   Checks and configures the three board LEDs as inactive outputs.  */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: int - 0 or a negative GPIO/device error                    */
/* ------------------------------------------------------------------ */
int pedal_activity_init(void)
{
    if (!gpio_is_ready_dt(&processing_led) ||
        !gpio_is_ready_dt(&revolution_led) ||
        !gpio_is_ready_dt(&uart_led)) {
        return -ENODEV;
    }
    int rc = gpio_pin_configure_dt(&processing_led, GPIO_OUTPUT_INACTIVE);
    if (rc < 0) {
        return rc;
    }
    rc = gpio_pin_configure_dt(&revolution_led, GPIO_OUTPUT_INACTIVE);
    if (rc < 0) {
        return rc;
    }
    rc = gpio_pin_configure_dt(&uart_led, GPIO_OUTPUT_INACTIVE);
    if (rc < 0) {
        return rc;
    }
    initialized = true;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_processing_start(void)                */
/*                                                                    */
/* Description:                                                       */
/*   Starts a visible LED0 blink across the sample replay.           */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: void - lights LED0 and schedules blinking                  */
/* ------------------------------------------------------------------ */
void pedal_activity_processing_start(void)
{
    if (!initialized) {
        return;
    }
    processing = true;
    processing_led_on = true;
    gpio_pin_set_dt(&processing_led, 1);
    k_work_reschedule(&processing_work, K_MSEC(PEDAL_PROCESSING_BLINK_MS));
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_processing_stop(void)                 */
/*                                                                    */
/* Description:                                                       */
/*   Stops the LED0 blink when sample replay ends.                    */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: void - cancels blinking and turns LED0 off                 */
/* ------------------------------------------------------------------ */
void pedal_activity_processing_stop(void)
{
    if (!initialized) {
        return;
    }
    processing = false;
    k_work_cancel_delayable_sync(&processing_work, &processing_stop_sync);
    processing_led_on = false;
    gpio_pin_set_dt(&processing_led, 0);
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_revolution(void)                      */
/*                                                                    */
/* Description:                                                       */
/*   Starts or extends a visible LED1 pulse for a completed turn.    */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: void - lights LED1 and schedules its turn-off              */
/* ------------------------------------------------------------------ */
void pedal_activity_revolution(void)
{
    if (!initialized) {
        return;
    }
    gpio_pin_set_dt(&revolution_led, 1);
    k_work_reschedule(&revolution_work, K_MSEC(PEDAL_REVOLUTION_PULSE_MS));
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_uart_start(void)                      */
/*                                                                    */
/* Description:                                                       */
/*   Lights LED3 when output is queued for interrupt-driven UART.    */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: void - cancels turn-off and lights LED3                    */
/* ------------------------------------------------------------------ */
void pedal_activity_uart_start(void)
{
    if (!initialized) {
        return;
    }
    k_work_cancel_delayable(&uart_work);
    gpio_pin_set_dt(&uart_led, 1);
}

/* ------------------------------------------------------------------ */
/* Creates: void pedal_activity_uart_idle(void)                       */
/*                                                                    */
/* Description:                                                       */
/*   Keeps LED3 visible briefly after the UART software queue drains. */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: void - schedules LED3 turn-off                             */
/* ------------------------------------------------------------------ */
void pedal_activity_uart_idle(void)
{
    if (initialized) {
        k_work_reschedule(&uart_work, K_MSEC(PEDAL_UART_PULSE_MS));
    }
}
