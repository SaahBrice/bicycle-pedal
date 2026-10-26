#ifndef PEDAL_ALGORITHMS_H
#define PEDAL_ALGORITHMS_H

#include "data_types/pedal_types.h"

typedef struct {
    float coeff_b1_Nm_per_uV; // [N*m/uV] Bridge 1 torque coefficient.
    float coeff_b2_Nm_per_uV; // [N*m/uV] Bridge 2 torque coefficient.
    float torque_offset_Nm;   // [N*m] Combined zero and thermal correction.
    float axis_x;             // [1] Gyro X projection coefficient.
    float axis_y;             // [1] Gyro Y projection coefficient.
    float axis_z;             // [1] Gyro Z projection coefficient.
    float excitation_V;       // [V] Expected constant excitation.
} PedalSimplifiedCoefficients;

// Calculate torque, power, and angular velocity with the full bridge chain.
int pedal_standard_process(const PedalSensorSample *sample,
                           const PedalCalibration *cal, PedalResult *result);

// Precompute the fixed coefficients for the simplified calculation.
int pedal_simplified_prepare(const PedalCalibration *cal,
                             PedalSimplifiedCoefficients *coeff);

// Calculate torque, power, and angular velocity with fixed coefficients.
int pedal_simplified_process(const PedalSensorSample *sample,
                             const PedalSimplifiedCoefficients *coeff,
                             PedalResult *result);

#endif // PEDAL_ALGORITHMS_H
