#ifndef PEDAL_SENSORS_H
#define PEDAL_SENSORS_H

#include "data_types/pedal_types.h"

typedef struct {
    unsigned int next_index;
} PedalSensorSource;

// Rewind the lookup sample source to its first sample.
void pedal_sensors_reset(PedalSensorSource *source);

// Copy the next sample; return 1 for data, 0 for normal end, or -EINVAL.
int pedal_sensors_next(PedalSensorSource *source, PedalSensorSample *sample);

#endif // PEDAL_SENSORS_H
