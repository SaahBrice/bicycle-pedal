#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "algorithms/algorithms.h"
#include "activity/activity.h"
#include "activity/button/button.h"
#include "configuration/configuration.h"
#include "configuration/constants.h"
#include "sensors/sensors.h"
#ifndef CONFIG_PEDAL_POWER_BUILD
#include "communication/communication.h"
#include "revolution/revolution.h"
#include "timing/timing.h"
#endif

static const struct gpio_dt_spec marker = GPIO_DT_SPEC_GET(DT_NODELABEL(marker), gpios);
// Keep computed outputs observable in the silent build too.
static volatile PedalResult last_result;
#ifdef CONFIG_PEDAL_POWER_BUILD
enum mode { SLEEP, STANDARD_CONTINUOUS, SIMPLIFIED_CONTINUOUS,
            STANDARD_DUTY, SIMPLIFIED_DUTY, BOTH_DUTY, MODE_COUNT };
static volatile enum mode current_mode;

static int run(const PedalCalibration *cal)
{
    PedalSensorSource source;
    PedalSimplifiedCoefficients coeff;
    PedalSensorSample sample;
    PedalResult result;
    bool restart = true;
    int64_t deadline = 0;
    while (true) {
        int rc = pedal_button_poll();
        if (rc < 0) { return rc; }
        if (rc > 0) {
            gpio_pin_set_dt(&marker, 0);
            current_mode = (current_mode + 1) % MODE_COUNT;
            restart = true;
        }
        if (current_mode == SLEEP) {
            // Block until a debounced press; no periodic application wakeup.
            pedal_button_wait();
            continue;
        }
        const bool both = current_mode == BOTH_DUTY;
        const bool simplified = current_mode == SIMPLIFIED_CONTINUOUS ||
                                current_mode == SIMPLIFIED_DUTY || both;
        const bool duty = current_mode >= STANDARD_DUTY;
        if (restart) {
            pedal_sensors_reset(&source);
            deadline = k_uptime_get();
            gpio_pin_set_dt(&marker, !duty);
            if (simplified && (rc = pedal_simplified_prepare(cal, &coeff)) < 0) { return rc; }
            restart = false;
        }
        rc = pedal_sensors_next(&source, &sample);
        if (rc < 0) { return rc; }
        if (rc == 0) {
            // Preserve cadence across wrap; prepare coefficients once per replay.
            pedal_sensors_reset(&source);
            if (simplified && (rc = pedal_simplified_prepare(cal, &coeff)) < 0) { return rc; }
            rc = pedal_sensors_next(&source, &sample);
            if (rc <= 0) { return -ENODATA; }
        }
        if (duty) { gpio_pin_set_dt(&marker, 1); }
        rc = (simplified && !both) ? pedal_simplified_process(&sample, &coeff, &result) :
                          pedal_standard_process(&sample, cal, &result);
        if (duty) { gpio_pin_set_dt(&marker, 0); }
        if (rc < 0) { return rc; }
        last_result = result;
        if (both) {
            // Same input, second pulse: simplified follows standard without a delay.
            gpio_pin_set_dt(&marker, 1);
            rc = pedal_simplified_process(&sample, &coeff, &result);
            gpio_pin_set_dt(&marker, 0);
            if (rc < 0) { return rc; }
            last_result = result;
        }
        if (duty) {
            deadline += PEDAL_SAMPLE_PERIOD_MS;
            k_sleep(K_TIMEOUT_ABS_MS(deadline));
        }
    }
}
#else
struct difference { double sum, absolute, squares, maximum; };
static void add_difference(struct difference *s, double d)
{
    s->sum += d;
    s->absolute += fabs(d);
    s->squares += d*d;
    if (fabs(d) > s->maximum) { s->maximum = fabs(d); }
}

// Report only after both timed calls have finished.
static int report(uint32_t revolution, const PedalTimingStats timing[2],
                  const struct difference differences[3])
{
    char line[240];
    snprintf(line, sizeof(line), "Revolution %u: %u samples; differences = simplified - standard",
             revolution, timing[0].calls);
    int rc = pedal_communication_send(PEDAL_MESSAGE_PLAIN, line);
    if (rc < 0) { return rc; }
    for (int i = 0; i < 2; ++i) {
        PedalTimingSummary s;
        rc = pedal_timing_summarize(&timing[i], &s);
        if (rc < 0) { return rc; }
        snprintf(line, sizeof(line), "%s: sample %.2f cycles, %.3f us; revolution %llu cycles, %.3f us",
                 i ? "simplified" : "standard", s.average_cycles, s.average_us,
                 (unsigned long long)s.total_cycles, s.total_us);
        rc = pedal_communication_send(PEDAL_MESSAGE_PLAIN, line);
        if (rc < 0) { return rc; }
    }
    const char *names[] = {"torque [Nm]", "power [W]", "omega [rad/s]"};
    for (int i = 0; i < 3; ++i) {
        const struct difference *d = &differences[i];
        const double n = timing[0].calls;
        snprintf(line, sizeof(line), "%s: bias %.8f, MAE %.8f, RMSE %.8f, max absolute %.8f",
                 names[i], d->sum/n, d->absolute/n, sqrt(d->squares/n), d->maximum);
        rc = pedal_communication_send(PEDAL_MESSAGE_PLAIN, line);
        if (rc < 0) { return rc; }
    }
    // One empty line separates completed revolution reports.
    rc = pedal_communication_send(PEDAL_MESSAGE_PLAIN, "");
    if (rc < 0) { return rc; }
    return pedal_communication_wait_idle(1000);
}

static int run(const PedalCalibration *cal)
{
    int rc = pedal_communication_init();
    if (rc < 0) { return rc; }
    rc = pedal_timing_init();
    if (rc < 0) { return rc; }
    while (true) {
        PedalSimplifiedCoefficients coeff;
        PedalSensorSource source;
        PedalSensorSample sample;
        PedalRevolutionTracker tracker;
        PedalSummary completed;
        PedalTimingStats timing[2] = {0};
        struct difference differences[3] = {0};
        rc = pedal_simplified_prepare(cal, &coeff);
        if (rc < 0) { return rc; }
        pedal_sensors_reset(&source);
        pedal_revolution_reset(&tracker);
        while ((rc = pedal_sensors_next(&source, &sample)) > 0) {
            PedalResult results[2];
            // Two scope pulses per sample: standard first, simplified second.
            for (int i = 0; i < 2; ++i) {
                gpio_pin_set_dt(&marker, 1);
                PedalTimingStamp stamp = pedal_timing_begin();
                rc = i ? pedal_simplified_process(&sample, &coeff, &results[i]) :
                         pedal_standard_process(&sample, cal, &results[i]);
                const uint32_t cycles = pedal_timing_end(stamp, &timing[i]);
                gpio_pin_set_dt(&marker, 0);
                if (rc < 0) { return rc; }
                if (!cycles) { return -ENOTSUP; }
            }
            last_result = results[1];
            add_difference(&differences[0], (double)results[1].torque_Nm-(double)results[0].torque_Nm);
            add_difference(&differences[1], (double)results[1].power_W-(double)results[0].power_W);
            add_difference(&differences[2], (double)results[1].omega_rad_s-(double)results[0].omega_rad_s);
            // Shared standard-derived boundaries keep the compared sample sets equal.
            rc = pedal_revolution_add(&tracker, &sample, &results[0], &completed);
            if (rc < 0) { return rc; }
            pedal_timing_suspend();
            if (rc > 0) {
                rc = report(tracker.complete_revolutions, timing, differences);
                if (rc < 0) { return rc; }
                timing[0] = timing[1] = (PedalTimingStats){0};
                for (int i = 0; i < 3; ++i) { differences[i] = (struct difference){0}; }
            }
        }
        if (rc < 0) { return rc; }
        // Add two empty lines to the last report's separator: three at table end.
        rc = pedal_communication_send(PEDAL_MESSAGE_PLAIN, "\r\n");
        if (rc < 0) { return rc; }
        rc = pedal_communication_wait_idle(1000);
        if (rc < 0) { return rc; }
        // An incomplete final revolution is discarded at table wrap.
    }
}
#endif

int main(void)
{
    PedalCalibration cal;
    int rc = pedal_activity_init();
    if (rc == 0) { rc = pedal_button_init(); }
    if (rc == 0 && !gpio_is_ready_dt(&marker)) { rc = -ENODEV; }
    if (rc == 0) { rc = gpio_pin_configure_dt(&marker, GPIO_OUTPUT_INACTIVE); }
    if (rc == 0) { rc = pedal_configuration_load(&cal); }
    if (rc == 0) { rc = run(&cal); }
    gpio_pin_set_dt(&marker, 0);
    return rc;
}
