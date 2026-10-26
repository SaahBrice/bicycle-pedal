#include "revolution/revolution.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#define PEDAL_TURN_RAD 6.28318530717958647692f // [rad] Full revolution.

/* ------------------------------------------------------------------ */
/* Creates: void pedal_revolution_reset(PedalRevolutionTracker *tracker) */
/*                                                                    */
/* Description:                                                       */
/*   Clears all turn and total accumulators when a destination exists. */
/*                                                                    */
/* Input:                                                             */
/*   tracker - state to clear                                          */
/*                                                                    */
/* Output:                                                            */
/*   void - tracker is reset in place                                  */
/* ------------------------------------------------------------------ */
void pedal_revolution_reset(PedalRevolutionTracker *tracker)
{
    if (tracker != NULL) {
        memset(tracker, 0, sizeof(*tracker));
    }
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_revolution_add(PedalRevolutionTracker *tracker,  */
/*                                  const PedalSensorSample *sample,  */
/*                                  const PedalResult *result,         */
/*                                  PedalSummary *completed)           */
/*                                                                    */
/* Description:                                                       */
/*   Validates sample order, integrates angular travel, and creates  */
/*   a sample mean when a full revolution is completed.               */
/*                                                                    */
/* Input:                                                             */
/*   tracker - state accumulated from previous samples                */
/*   sample - index and timestamp of this reading                     */
/*   result - torque, power, and angular velocity of this reading     */
/*   completed - destination when a revolution is completed           */
/*                                                                    */
/* Output:                                                            */
/*   int - 1 complete, 0 incomplete, or -EINVAL                       */
/* ------------------------------------------------------------------ */
int pedal_revolution_add(PedalRevolutionTracker *tracker,
                         const PedalSensorSample *sample,
                         const PedalResult *result, PedalSummary *completed)
{
    if (tracker == NULL || sample == NULL || result == NULL || completed == NULL ||
        !isfinite(sample->time_s) || sample->time_s < 0.0f ||
        !isfinite(result->omega_rad_s) || !isfinite(result->torque_Nm) ||
        !isfinite(result->power_W)) {
        return -EINVAL;
    }
    if (tracker->has_previous) {
        if (sample->sample_index <= tracker->previous_index ||
            sample->time_s <= tracker->previous_time_s) {
            return -EINVAL;
        }
        const float increment = 0.5f *
            (tracker->previous_omega_rad_s + result->omega_rad_s) *
            (sample->time_s - tracker->previous_time_s);
        if (!isfinite(increment) || fabsf(increment) >= PEDAL_TURN_RAD) {
            return -EINVAL;
        }
        const float corrected = fabsf(increment) - tracker->travel_comp_rad;
        const float updated = tracker->travel_rad + corrected;
        tracker->travel_comp_rad =
            (updated - tracker->travel_rad) - corrected;
        tracker->travel_rad = updated;
    }
    tracker->has_previous = true;
    tracker->previous_index = sample->sample_index;
    tracker->previous_time_s = sample->time_s;
    tracker->previous_omega_rad_s = result->omega_rad_s;
    tracker->turn_samples++;
    tracker->turn_torque_Nm += result->torque_Nm;
    tracker->turn_power_W += result->power_W;
    tracker->total_samples++;
    tracker->total_torque_Nm += result->torque_Nm;
    tracker->total_power_W += result->power_W;

    if (tracker->travel_rad + 1e-5f >=
        (tracker->complete_revolutions + 1u) * PEDAL_TURN_RAD) {
        completed->samples = tracker->turn_samples;
        completed->average_torque_Nm = tracker->turn_torque_Nm / tracker->turn_samples;
        completed->average_power_W = tracker->turn_power_W / tracker->turn_samples;
        tracker->complete_revolutions++;
        tracker->turn_samples = 0;
        tracker->turn_torque_Nm = 0.0;
        tracker->turn_power_W = 0.0;
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_revolution_summary(const PedalRevolutionTracker */
/*                                      *tracker, bool partial,        */
/*                                      PedalSummary *summary)          */
/*                                                                    */
/* Description:                                                       */
/*   Calculates a sample mean for all readings or the unfinished turn. */
/*                                                                    */
/* Input:                                                             */
/*   tracker - accumulated replay state                                */
/*   partial - true selects unfinished turn; false selects all data   */
/*   summary - destination for mean torque and power                   */
/*                                                                    */
/* Output:                                                            */
/*   int - 0 on success, -ENODATA if empty, or -EINVAL                */
/* ------------------------------------------------------------------ */
int pedal_revolution_summary(const PedalRevolutionTracker *tracker,
                             bool partial, PedalSummary *summary)
{
    if (tracker == NULL || summary == NULL) {
        return -EINVAL;
    }
    const uint32_t count = partial ? tracker->turn_samples : tracker->total_samples;
    if (count == 0u) {
        return -ENODATA;
    }
    summary->samples = count;
    summary->average_torque_Nm =
        (partial ? tracker->turn_torque_Nm : tracker->total_torque_Nm) / count;
    summary->average_power_W =
        (partial ? tracker->turn_power_W : tracker->total_power_W) / count;
    return 0;
}
