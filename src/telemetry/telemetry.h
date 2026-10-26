#ifndef PEDAL_TELEMETRY_H
#define PEDAL_TELEMETRY_H

#include "communication/communication.h"
#include "data_types/pedal_types.h"
#include "timing/timing.h"

// Send a readable message with a plain, success, warning, or error state.
int pedal_telemetry_status(PedalMessageLevel level, const char *message);

// Report a component error with its sample index and negative errno code.
int pedal_telemetry_error(const char *component, uint32_t sample_index,
                          int code);

// Report one simplified algorithm result with the sample index and time.
int pedal_telemetry_sample(const PedalSensorSample *sample,
                           const PedalResult *result);

// Report a labeled full turn, partial turn, or total sample mean.
int pedal_telemetry_summary(const char *label, uint32_t turns,
                            const PedalSummary *summary);

// Report the largest numerical differences between standard and simplified results.
int pedal_telemetry_comparison(float maximum_delta_Nm, float maximum_delta_W);

// Report total and average computation cost for an algorithm and replay scope.
int pedal_telemetry_timing(const char *scope, uint32_t revolution,
                           const char *algorithm,
                           const PedalTimingSummary *summary);

// Report standard and simplified timing for the same replay segment.
int pedal_telemetry_timing_pair(const char *scope, uint32_t revolution,
                                const PedalTimingStats *standard,
                                const PedalTimingStats *simplified);

// Report how many samples missed their 200 Hz release deadline.
int pedal_telemetry_pacing(uint32_t late_samples, uint32_t total_samples);

// Report one selected algorithm; optional setup timing is separate from samples.
int pedal_telemetry_final(const char *algorithm, uint32_t samples,
                          uint32_t revolutions, const PedalTimingStats *all,
                          const PedalTimingStats *full,
                          const PedalTimingStats *setup);

#endif // PEDAL_TELEMETRY_H
