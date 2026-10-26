#ifndef PEDAL_LOOKUP_SAMPLES_H
#define PEDAL_LOOKUP_SAMPLES_H

#include <stddef.h>
#include "data_types/pedal_types.h"

// Generated simulator readings in chronological order.
extern const PedalSensorSample pedal_lookup_samples[];
// Number of readings available to the lookup source.
extern const size_t pedal_lookup_count;

#endif // PEDAL_LOOKUP_SAMPLES_H
