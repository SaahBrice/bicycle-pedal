#include <errno.h>
#include <zephyr/kernel.h>

#include "algorithms/algorithms.h"
#include "activity/activity.h"
#include "communication/communication.h"
#include "configuration/configuration.h"
#include "configuration/constants.h"
#include "revolution/revolution.h"
#include "sensors/sensors.h"
#include "telemetry/telemetry.h"
#include "timing/timing.h"

/* ------------------------------------------------------------------ */
/* Creates: int main(void)                                             */
/*                                                                    */
/* Description:                                                       */
/*   Replays simulator samples at the configured 200 Hz rate. Times   */
/*   each algorithm call with the DWT counter. Once replay finishes,  */
/*   reports sample and revolution counts and average computation     */
/*   costs for the standard and simplified algorithms.                */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: int - 0 after a complete replay, or negative errno code   */
/* ------------------------------------------------------------------ */
int main(void)
{
    PedalCalibration cal;
    PedalSimplifiedCoefficients coeff;
    PedalSensorSource source;
    PedalRevolutionTracker tracker;
    PedalSensorSample sample;
    PedalResult standard;
    PedalResult simplified;
    PedalSummary summary;
    PedalTimingStats standard_total = {0};
    PedalTimingStats simplified_total = {0};
    PedalTimingStats standard_turn = {0};
    PedalTimingStats simplified_turn = {0};
    PedalTimingStats standard_full_revs = {0};
    PedalTimingStats simplified_full_revs = {0};
    int rc = pedal_communication_init();
    if (rc < 0) {
        return rc;
    }
    rc = pedal_activity_init();
    if (rc < 0) {
        pedal_telemetry_error("LED activity", 0u, rc);
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
    rc = pedal_simplified_prepare(&cal, &coeff);
    if (rc < 0) {
        pedal_telemetry_error("algorithm setup", 0u, rc);
        return rc;
    }
    pedal_sensors_reset(&source);
    pedal_revolution_reset(&tracker);
    pedal_activity_processing_start();
    const int64_t replay_start_ms = k_uptime_get();
    uint32_t replay_step = 0u;
    while ((rc = pedal_sensors_next(&source, &sample)) > 0) {
        const int64_t deadline_ms = replay_start_ms +
            (int64_t)replay_step * PEDAL_SAMPLE_PERIOD_MS;
        k_sleep(K_TIMEOUT_ABS_MS(deadline_ms));
        replay_step++;

        PedalTimingStamp stamp = pedal_timing_begin();
        rc = pedal_standard_process(&sample, &cal, &standard);
        const uint32_t standard_cycles = pedal_timing_end(stamp, &standard_total);
        if (standard_cycles == 0u) {
            pedal_activity_processing_stop();
            pedal_telemetry_error("standard cycle counter", sample.sample_index, -ENOTSUP);
            return -ENOTSUP;
        }
        if (rc < 0) {
            pedal_activity_processing_stop();
            pedal_telemetry_error("standard algorithm", sample.sample_index, rc);
            return rc;
        }
        pedal_timing_record(&standard_turn, standard_cycles);
        stamp = pedal_timing_begin();
        rc = pedal_simplified_process(&sample, &coeff, &simplified);
        const uint32_t simplified_cycles = pedal_timing_end(stamp, &simplified_total);
        if (simplified_cycles == 0u) {
            pedal_activity_processing_stop();
            pedal_telemetry_error("simplified cycle counter", sample.sample_index, -ENOTSUP);
            return -ENOTSUP;
        }
        if (rc < 0) {
            pedal_activity_processing_stop();
            pedal_telemetry_error("simplified algorithm", sample.sample_index, rc);
            return rc;
        }
        pedal_timing_record(&simplified_turn, simplified_cycles);
        rc = pedal_revolution_add(&tracker, &sample, &simplified, &summary);
        if (rc < 0) {
            pedal_activity_processing_stop();
            pedal_telemetry_error("revolution", sample.sample_index, rc);
            return rc;
        }
        if (rc > 0) {
            pedal_activity_revolution();
            pedal_timing_merge(&standard_full_revs, &standard_turn);
            pedal_timing_merge(&simplified_full_revs, &simplified_turn);
            standard_turn = (PedalTimingStats) {0};
            simplified_turn = (PedalTimingStats) {0};
        }
    }
    pedal_activity_processing_stop();
    if (rc < 0) {
        pedal_telemetry_error("sensors", sample.sample_index, rc);
        return rc;
    }
    return pedal_telemetry_final(tracker.total_samples,
                                 tracker.complete_revolutions,
                                 &standard_total, &standard_full_revs,
                                 &simplified_total, &simplified_full_revs);
}
