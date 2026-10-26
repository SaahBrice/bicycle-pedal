#ifndef PEDAL_REVOLUTION_H
#define PEDAL_REVOLUTION_H

#include <stdbool.h>
#include "data_types/pedal_types.h"

typedef struct {
    bool has_previous;
    uint32_t previous_index;
    float previous_time_s;      // [s] Previous timestamp.
    float previous_omega_rad_s; // [rad/s] Previous angular velocity.
    float travel_rad;           // [rad] Accumulated absolute travel.
    float travel_comp_rad;      // [rad] Compensation for float summation.
    uint32_t complete_revolutions;
    uint32_t turn_samples;
    float turn_torque_Nm;       // [N*m] Sum of turn sample torques.
    float turn_power_W;         // [W] Sum of turn sample powers.
    uint32_t total_samples;
    float total_torque_Nm;      // [N*m] Sum of all sample torques.
    float total_power_W;        // [W] Sum of all sample powers.
} PedalRevolutionTracker;

// Clear turn and total accumulators before a replay.
void pedal_revolution_reset(PedalRevolutionTracker *tracker);

// Add a timed result; return 1 when a turn completes, 0 otherwise, or -EINVAL.
int pedal_revolution_add(PedalRevolutionTracker *tracker,
                         const PedalSensorSample *sample,
                         const PedalResult *result, PedalSummary *completed);

// Return the mean for all samples or the unfinished final turn.
int pedal_revolution_summary(const PedalRevolutionTracker *tracker,
                             bool partial, PedalSummary *summary);

#endif // PEDAL_REVOLUTION_H
