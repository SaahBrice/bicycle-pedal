#ifndef PEDAL_CONFIGURATION_H
#define PEDAL_CONFIGURATION_H

#include "data_types/pedal_types.h"

// Load fixed simulator calibration into cal; returns 0 or -EINVAL.
int pedal_configuration_load(PedalCalibration *cal);

#endif // PEDAL_CONFIGURATION_H
