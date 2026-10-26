#include <errno.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#include "algorithms/algorithms.h"
#include "activity/activity.h"
#include "activity/button/button.h"
#include "communication/communication.h"
#include "configuration/configuration.h"
#include "configuration/constants.h"
#include "revolution/revolution.h"
#include "sensors/sensors.h"
#include "telemetry/telemetry.h"
#include "timing/timing.h"

/*
 * Replays the lookup table continuously at 200 Hz, starting in standard mode.
 * Button 0 selects the other algorithm and restarts from the first sample.
 * Each completed replay reports only its selected algorithm. Simplified
 * coefficients are prepared once at the start of every simplified replay.
 * Output: returns a negative error code if initialization or replay fails.
 */
int main(void)
{
    PedalCalibration cal;
    bool simplified_mode = false;
    bool announce_switch = false;
    int rc = pedal_communication_init();
    if (rc < 0) {
        return rc;
    }
    rc = pedal_activity_init();
    if (rc < 0) {
        pedal_telemetry_error("LED activity", 0u, rc);
        return rc;
    }
    rc = pedal_button_init();
    if (rc < 0) {
        pedal_telemetry_error("button", 0u, rc);
        return rc;
    }
    rc = pedal_timing_init();
    if (rc < 0) {
        pedal_telemetry_error("cycle counter", 0u, rc);
        return rc;
    }
    rc = pedal_configuration_load(&cal);
    if (rc < 0) {
        pedal_telemetry_error("configuration", 0u, rc);
        return rc;
    }

    while (true) {
        PedalSimplifiedCoefficients coeff;
        PedalSensorSource source;
        PedalRevolutionTracker tracker;
        PedalSensorSample sample = {0};
        PedalResult result;
        PedalSummary summary;
        PedalTimingStats total = {0};
        PedalTimingStats turn = {0};
        PedalTimingStats full_revs = {0};
        PedalTimingStats setup = {0};
        bool restart = false;
        const char *error_component = "sensors";

        rc = pedal_button_poll();
        if (rc < 0) {
            pedal_telemetry_error("button", 0u, rc);
            return rc;
        }
        if (rc > 0) {
            simplified_mode = !simplified_mode;
            announce_switch = true;
        }
        const char *algorithm = simplified_mode ? "simplified" : "standard";
        if (announce_switch) {
            rc = pedal_telemetry_status(PEDAL_MESSAGE_PLAIN,
                simplified_mode ? "Switched to simplified algorithm" :
                                  "Switched to standard algorithm");
            if (rc < 0) {
                return rc;
            }
            announce_switch = false;
        }
        pedal_activity_processing_start(simplified_mode);
        if (simplified_mode) {
            const PedalTimingStamp stamp = pedal_timing_begin();
            rc = pedal_simplified_prepare(&cal, &coeff);
            const uint32_t cycles = pedal_timing_end(stamp, &setup);
            if (rc < 0 || cycles == 0u) {
                pedal_activity_processing_stop();
                rc = rc < 0 ? rc : -ENOTSUP;
                pedal_telemetry_error("algorithm setup", 0u, rc);
                return rc;
            }
        }
        pedal_sensors_reset(&source);
        pedal_revolution_reset(&tracker);
        const int64_t replay_start_ms = k_uptime_get();
        uint32_t replay_step = 0u;
        while ((rc = pedal_sensors_next(&source, &sample)) > 0) {
            const int64_t deadline_ms = replay_start_ms +
                (int64_t)replay_step * PEDAL_SAMPLE_PERIOD_MS;
            k_sleep(K_TIMEOUT_ABS_MS(deadline_ms));
            replay_step++;

            rc = pedal_button_poll();
            if (rc < 0) {
                error_component = "button";
                break;
            }
            if (rc > 0) {
                simplified_mode = !simplified_mode;
                announce_switch = true;
                restart = true;
                break;
            }

            uint32_t cycles;
            if (simplified_mode) {
                const PedalTimingStamp stamp = pedal_timing_begin();
                rc = pedal_simplified_process(&sample, &coeff, &result);
                cycles = pedal_timing_end(stamp, &total);
            } else {
                const PedalTimingStamp stamp = pedal_timing_begin();
                rc = pedal_standard_process(&sample, &cal, &result);
                cycles = pedal_timing_end(stamp, &total);
            }
            if (cycles == 0u) {
                error_component = "cycle counter";
                rc = -ENOTSUP;
                break;
            }
            if (rc < 0) {
                error_component = algorithm;
                break;
            }
            pedal_timing_record(&turn, cycles);
            rc = pedal_revolution_add(&tracker, &sample, &result, &summary);
            if (rc < 0) {
                error_component = "revolution";
                break;
            }
            if (rc > 0) {
                pedal_activity_revolution();
                pedal_timing_merge(&full_revs, &turn);
                turn = (PedalTimingStats){0};
            }
        }
        pedal_activity_processing_stop();
        if (restart) {
            continue;
        }
        if (rc < 0) {
            pedal_telemetry_error(error_component, sample.sample_index, rc);
            return rc;
        }
        pedal_activity_table_complete();
        rc = pedal_telemetry_final(algorithm, tracker.total_samples,
                                   tracker.complete_revolutions, &total,
                                   &full_revs, simplified_mode ? &setup : NULL);
        if (rc < 0) {
            return rc;
        }
    }
}
