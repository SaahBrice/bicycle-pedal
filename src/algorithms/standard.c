#include "algorithms/algorithms.h"

#include <errno.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Creates: int pedal_standard_process(const PedalSensorSample *sample, */
/*                                     const PedalCalibration *cal,     */
/*                                     PedalResult *result)             */
/*                                                                    */
/* Description:                                                       */
/*   Mirrors standard_algorithm.py. Half-bridge inputs are deviations */
/*   from Vex/2, so normalization only converts [uV] to [V] and      */
/*   divides by excitation [V]. The result is written only when valid. */
/*                                                                    */
/* Input:                                                             */
/*   sample - bridge and gyroscope readings                            */
/*   cal - simulator-derived geometry and offsets                      */
/*   result - calculation destination                                  */
/*                                                                    */
/* Output:                                                            */
/*   int - 0 on success or -EINVAL for invalid inputs                  */
/* ------------------------------------------------------------------ */
int pedal_standard_process(const PedalSensorSample *sample,
                           const PedalCalibration *cal, PedalResult *result)
{
    if (sample == NULL || cal == NULL || result == NULL ||
        !isfinite(sample->excitation_V) || sample->excitation_V <= 0.0f ||
        !isfinite(cal->k_cal_b1) || !isfinite(cal->k_cal_b2) ||
        cal->k_cal_b1 <= 0.0f || cal->k_cal_b2 <= 0.0f ||
        cal->delta_b1_m <= 0.0f || cal->delta_b2_m <= 0.0f) {
        return -EINVAL;
    }
    for (unsigned int i = 0; i < 4; ++i) {
        if (!isfinite(sample->half_bridge_uV[i])) {
            return -EINVAL;
        }
    }
    for (unsigned int i = 0; i < 3; ++i) {
        if (!isfinite(sample->gyro_rad_s[i])) {
            return -EINVAL;
        }
    }

    const float scale = 1e-6f / sample->excitation_V;
    const float q1 = (sample->half_bridge_uV[0] - sample->half_bridge_uV[1]) * scale -
                     cal->offset_zero_b1 - cal->offset_temp_b1;
    const float q2 = (sample->half_bridge_uV[2] - sample->half_bridge_uV[3]) * scale -
                     cal->offset_zero_b2 - cal->offset_temp_b2;
    const float f1_N = q1 / (cal->k_cal_b1 * cal->delta_b1_m);
    const float f2_N = q2 / (cal->k_cal_b2 * cal->delta_b2_m);
    const float tangential_N = f1_N * cosf(cal->alpha_rad) -
                               f2_N * sinf(cal->alpha_rad);
    const float omega_rad_s =
        -sinf(cal->imu_pitch_rad) * sample->gyro_rad_s[0] +
        cosf(cal->imu_pitch_rad) * sinf(cal->imu_roll_rad) * sample->gyro_rad_s[1] +
        cosf(cal->imu_pitch_rad) * cosf(cal->imu_roll_rad) * sample->gyro_rad_s[2];
    const float torque_Nm = tangential_N * cal->crank_length_m;
    if (!isfinite(torque_Nm) || !isfinite(omega_rad_s) ||
        !isfinite(torque_Nm * omega_rad_s)) {
        return -EINVAL;
    }
    *result = (PedalResult) {
        .torque_Nm = torque_Nm,
        .power_W = torque_Nm * omega_rad_s,
        .omega_rad_s = omega_rad_s,
    };
    return 0;
}
