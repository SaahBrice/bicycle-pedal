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

// Compile out all P1.11 writes when CONFIG_PEDAL_SCOPE_MARKER=n.
static inline void marker_set(int value)
{
    if (IS_ENABLED(CONFIG_PEDAL_SCOPE_MARKER)) {
        gpio_pin_set_dt(&marker, value);
    }
}

static int initialize(PedalCalibration *calibration);
#ifdef CONFIG_PEDAL_POWER_BUILD
static int run_power_profile(const PedalCalibration *calibration);
#else
static int run_comparison(const PedalCalibration *calibration);
#endif

// Entry point: initialize the board, then run the selected build's workflow.
int main(void)
{
    PedalCalibration calibration;
    int initialization_status = initialize(&calibration);
    if (initialization_status < 0) {
        marker_set(0);
        return initialization_status;
    }

#ifdef CONFIG_PEDAL_POWER_BUILD
    // Silent build: Button 0 selects the algorithm and its sampling schedule.
    int run_status = run_power_profile(&calibration);
#else
    // Compare build: process both algorithms and report per revolution.
    int run_status = run_comparison(&calibration);
#endif
    marker_set(0); // Leave P1.11 low if a workflow stops with an error.
    return run_status;
}

#ifdef CONFIG_PEDAL_POWER_BUILD
// Button 0 advances through these modes in order, then returns to sleep.
enum mode { SLEEP, STANDARD_CONTINUOUS, SIMPLIFIED_CONTINUOUS,
            STANDARD_DUTY, SIMPLIFIED_DUTY, BOTH_DUTY };
static volatile enum mode current_mode = SLEEP;

// A button mode chooses one algorithm (or both) and a sampling schedule.
// These two choices are stored once per mode, outside the sample loop.
enum power_algorithm { POWER_STANDARD, POWER_SIMPLIFIED, POWER_BOTH };

struct power_replay {
    PedalSensorSource source;
    PedalSimplifiedCoefficients coeff;
    PedalSensorSample sample;
    PedalResult result;
    int64_t deadline;
    enum power_algorithm algorithm;
    bool duty_cycled;
};

// This switch runs only after a button press, outside the timed sample path.
static enum mode next_mode(enum mode mode)
{
    switch (mode) {
    case SLEEP:                 return STANDARD_CONTINUOUS;
    case STANDARD_CONTINUOUS:   return SIMPLIFIED_CONTINUOUS;
    case SIMPLIFIED_CONTINUOUS: return STANDARD_DUTY;
    case STANDARD_DUTY:        return SIMPLIFIED_DUTY;
    case SIMPLIFIED_DUTY:      return BOTH_DUTY;
    case BOTH_DUTY:            return SLEEP;
    default:                   return SLEEP;
    }
}

// A button press selects the next mode and starts its lookup at sample zero.
static int advance_power_mode(struct power_replay *replay, const PedalCalibration *cal)
{
    marker_set(0); // Finish the previous mode's scope indication.
    current_mode = next_mode(current_mode);
    if (current_mode == SLEEP) { return 0; }

    // Decode the new mode once. The sample loop uses these choices directly.
    switch (current_mode) {
    case STANDARD_CONTINUOUS:
        replay->algorithm = POWER_STANDARD;
        replay->duty_cycled = false;
        break;
    case SIMPLIFIED_CONTINUOUS:
        replay->algorithm = POWER_SIMPLIFIED;
        replay->duty_cycled = false;
        break;
    case STANDARD_DUTY:
        replay->algorithm = POWER_STANDARD;
        replay->duty_cycled = true;
        break;
    case SIMPLIFIED_DUTY:
        replay->algorithm = POWER_SIMPLIFIED;
        replay->duty_cycled = true;
        break;
    case BOTH_DUTY:
        replay->algorithm = POWER_BOTH;
        replay->duty_cycled = true;
        break;
    default:
        return -EINVAL;
    }

    // Every press starts again with the first lookup sample.
    pedal_sensors_reset(&replay->source);
    replay->deadline = k_uptime_get();
    // Continuous modes hold the marker high; duty modes pulse per calculation.
    marker_set(!replay->duty_cycled);
    // These coefficients remain fixed for the whole simplified table replay.
    if (replay->algorithm != POWER_STANDARD) {
        int preparation_status = pedal_simplified_prepare(cal, &replay->coeff);
        if (preparation_status < 0) { return preparation_status; }
    }
    return 0;
}

// Return 1 with the next lookup sample, or a negative error.
// A full table repeats without resetting the absolute 5 ms deadline.
static int read_lookup_sample(struct power_replay *replay, const PedalCalibration *cal)
{
    int sample_status = pedal_sensors_next(&replay->source, &replay->sample);
    if (sample_status != 0) { return sample_status; }

    // End of table: rewind and refresh simplified constants once per replay.
    pedal_sensors_reset(&replay->source);

    /* [TODO: COmmented out so it does not restart recalculation of constants at end of table]
    if (replay->algorithm != POWER_STANDARD) {
        int preparation_status = pedal_simplified_prepare(cal, &replay->coeff);
        if (preparation_status < 0) { return preparation_status; }
    }
    */
    sample_status = pedal_sensors_next(&replay->source, &replay->sample);
    return sample_status > 0 ? sample_status : -ENODATA;
}

// Run exactly one algorithm on the loaded sample, with one scope pulse.
static int calculate_algorithm(struct power_replay *replay, const PedalCalibration *cal,
                               enum power_algorithm algorithm)
{
    // The marker brackets calculation.
    if (replay->duty_cycled) { marker_set(1); }
    int calculation_status;
    switch (algorithm) {
    case POWER_SIMPLIFIED:
        calculation_status = pedal_simplified_process(&replay->sample, &replay->coeff,
                                                       &replay->result);
        break;
    case POWER_STANDARD:
        calculation_status = pedal_standard_process(&replay->sample, cal,
                                                    &replay->result);
        break;
    default:
        calculation_status = -EINVAL;
        break;
    }
    if (replay->duty_cycled) { marker_set(0); }
    if (calculation_status < 0) { return calculation_status; }
    last_result = replay->result;
    return 0;
}

// Choose one calculation, or run both sequentially on the same sample.
static int calculate_selected_algorithms(struct power_replay *replay,
                                          const PedalCalibration *cal)
{
    if (replay->algorithm == POWER_BOTH) {
        int standard_status = calculate_algorithm(replay, cal, POWER_STANDARD);
        if (standard_status < 0) { return standard_status; }
        return calculate_algorithm(replay, cal, POWER_SIMPLIFIED);
    }
    return calculate_algorithm(replay, cal, replay->algorithm);
}

// Block the main thread until the next 200 Hz sample is due.
// Zephyr idles the CPU while no thread is ready, then wakes it for this deadline.
static void sleep_until_next_sample(struct power_replay *replay)
{
    replay->deadline += PEDAL_SAMPLE_PERIOD_MS;
    k_sleep(K_TIMEOUT_ABS_MS(replay->deadline));
}

// Read, calculate, then wait for the next sample only in 200 Hz modes.
static int run_one_power_sample(struct power_replay *replay, const PedalCalibration *cal)
{
    int sample_status = read_lookup_sample(replay, cal);
    if (sample_status < 0) { return sample_status; }
    int calculation_status = calculate_selected_algorithms(replay, cal);
    if (calculation_status < 0) { return calculation_status; }

    if (replay->duty_cycled) {
        sleep_until_next_sample(replay);
    }
    return 0;
}

static int run_power_profile(const PedalCalibration *cal)
{
    struct power_replay replay = {0};
    while (true) {
        // 1. Handle a button press, if one occurred.
        int button_press = pedal_button_poll();
        if (button_press < 0) { return button_press; }
        if (button_press > 0) {
            int mode_status = advance_power_mode(&replay, cal);
            if (mode_status < 0) { return mode_status; }
        }

        // 2. Sleep mode runs no samples and waits for Button 0.
        if (current_mode == SLEEP) {
            pedal_button_wait();
            continue;
        }

        // 3. Active mode reads and calculates one sample, then sleeps if needed.
        int sample_status = run_one_power_sample(&replay, cal);
        if (sample_status < 0) { return sample_status; }
    }
}
#else
struct difference { double sum, absolute, squares, maximum; };
struct compare_replay {
    PedalSimplifiedCoefficients coeff;
    PedalSensorSource source;
    PedalSensorSample sample;
    PedalRevolutionTracker tracker;
    PedalSummary completed;
    PedalTimingStats timing[2];
    struct difference differences[3];
};

static void add_difference(struct difference *statistics, double error)
{
    statistics->sum += error;
    statistics->absolute += fabs(error);
    statistics->squares += error*error;
    if (fabs(error) > statistics->maximum) { statistics->maximum = fabs(error); }
}

// Report only after both timed calls have finished.
static int report_revolution(uint32_t revolution, const PedalTimingStats timing[2],
                             const struct difference differences[3])
{
    char line[240];
    snprintf(line, sizeof(line), "Revolution %u: %u samples; differences = simplified - standard",
             revolution, timing[0].calls);
    int send_status = pedal_communication_send(PEDAL_MESSAGE_PLAIN, line);
    if (send_status < 0) { return send_status; }
    for (int algorithm_index = 0; algorithm_index < 2; ++algorithm_index) {
        PedalTimingSummary summary;
        int timing_status = pedal_timing_summarize(&timing[algorithm_index], &summary);
        if (timing_status < 0) { return timing_status; }
        snprintf(line, sizeof(line), "%s: sample %.2f cycles, %.3f us; revolution %llu cycles, %.3f us",
                 algorithm_index ? "simplified" : "standard", summary.average_cycles,
                 summary.average_us, (unsigned long long)summary.total_cycles,
                 summary.total_us);
        send_status = pedal_communication_send(PEDAL_MESSAGE_PLAIN, line);
        if (send_status < 0) { return send_status; }
    }
    const char *names[] = {"torque [Nm]", "power [W]", "omega [rad/s]"};
    for (int quantity_index = 0; quantity_index < 3; ++quantity_index) {
        const struct difference *error = &differences[quantity_index];
        const double sample_count = timing[0].calls;
        snprintf(line, sizeof(line), "%s: bias %.8f, MAE %.8f, RMSE %.8f, max absolute %.8f",
                 names[quantity_index], error->sum/sample_count,
                 error->absolute/sample_count, sqrt(error->squares/sample_count),
                 error->maximum);
        send_status = pedal_communication_send(PEDAL_MESSAGE_PLAIN, line);
        if (send_status < 0) { return send_status; }
    }
    // One empty line separates completed revolution reports.
    send_status = pedal_communication_send(PEDAL_MESSAGE_PLAIN, "");
    if (send_status < 0) { return send_status; }
    return pedal_communication_wait_idle(1000);
}

// Time both algorithms on replay->sample and accumulate their differences.
// Marker writes bracket the measurements; completed revolutions are reported.
// Returns 0 or a processing, timing, revolution, or communication error.
static int compare_sample(struct compare_replay *replay, const PedalCalibration *cal)
{
    PedalResult results[2];
    // The same stored sample goes to standard, then simplified.
    for (int algorithm_index = 0; algorithm_index < 2; ++algorithm_index) {
        marker_set(1);
        PedalTimingStamp stamp = pedal_timing_begin();
        int algorithm_status = algorithm_index ?
            pedal_simplified_process(&replay->sample, &replay->coeff,
                                     &results[algorithm_index]) :
            pedal_standard_process(&replay->sample, cal, &results[algorithm_index]);
        const uint32_t cycles = pedal_timing_end(stamp, &replay->timing[algorithm_index]);
        marker_set(0);
        if (algorithm_status < 0) { return algorithm_status; }
        if (!cycles) { return -ENOTSUP; }
    }
    last_result = results[1];

    // Store simplified-minus-standard differences for this revolution.
    add_difference(&replay->differences[0],
                   (double)results[1].torque_Nm-(double)results[0].torque_Nm);
    add_difference(&replay->differences[1],
                   (double)results[1].power_W-(double)results[0].power_W);
    add_difference(&replay->differences[2],
                   (double)results[1].omega_rad_s-(double)results[0].omega_rad_s);

    // Standard output sets the revolution boundary for both algorithms.
    int revolution_status = pedal_revolution_add(&replay->tracker, &replay->sample,
                                                  &results[0], &replay->completed);
    if (revolution_status < 0) { return revolution_status; }
    pedal_timing_suspend();
    if (revolution_status > 0) {
        // UART reporting happens only after both timed calls have finished.
        int report_status = report_revolution(replay->tracker.complete_revolutions,
                                              replay->timing, replay->differences);
        if (report_status < 0) { return report_status; }
        replay->timing[0] = replay->timing[1] = (PedalTimingStats){0};
        for (int quantity_index = 0; quantity_index < 3; ++quantity_index) {
            replay->differences[quantity_index] = (struct difference){0};
        }
    }
    return 0;
}

// Replay one full lookup table, reporting each completed revolution.
// Returns 0 at table end; the caller then starts another full replay.
static int compare_table(const PedalCalibration *cal)
{
    struct compare_replay replay = {0};
    int preparation_status = pedal_simplified_prepare(cal, &replay.coeff);
    if (preparation_status < 0) { return preparation_status; }
    pedal_sensors_reset(&replay.source);
    pedal_revolution_reset(&replay.tracker);
    int sample_status;
    while ((sample_status = pedal_sensors_next(&replay.source, &replay.sample)) > 0) {
        int comparison_status = compare_sample(&replay, cal);
        if (comparison_status < 0) { return comparison_status; }
    }
    if (sample_status < 0) { return sample_status; }

    // An incomplete last revolution is dropped; add extra space at table end.
    int send_status = pedal_communication_send(PEDAL_MESSAGE_PLAIN, "\r\n");
    if (send_status < 0) { return send_status; }
    return pedal_communication_wait_idle(1000);
}

static int run_comparison(const PedalCalibration *cal)
{
    int communication_status = pedal_communication_init();
    if (communication_status < 0) { return communication_status; }
    int timing_status = pedal_timing_init();
    if (timing_status < 0) { return timing_status; }
    while (true) {
        int table_status = compare_table(cal);
        if (table_status < 0) { return table_status; }
    }
}
#endif

// Configure board GPIO, Button 0, scope marker, and fixed calibration.
static int initialize(PedalCalibration *calibration)
{
    int activity_status = pedal_activity_init();
    if (activity_status < 0) { return activity_status; }

    int button_status = pedal_button_init();
    if (button_status < 0) { return button_status; }

    if (IS_ENABLED(CONFIG_PEDAL_SCOPE_MARKER)) {
        if (!gpio_is_ready_dt(&marker)) { return -ENODEV; }
        int marker_status = gpio_pin_configure_dt(&marker, GPIO_OUTPUT_INACTIVE);
        if (marker_status < 0) { return marker_status; }
    }
    return pedal_configuration_load(calibration);
}
