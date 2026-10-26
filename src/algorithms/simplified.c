#include "algorithms/algorithms.h"

#include <errno.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Creates: int pedal_simplified_prepare(const PedalCalibration *cal,  */
/*                                       PedalSimplifiedCoefficients   */
/*                                       *coeff)                        */
/*                                                                    */
/* Description:                                                       */
/*   Mirrors the constant folding in simplified_algorithm.py. This   */
/*   removes divisions and trigonometric functions from each sample. */
/*                                                                    */
/* Input:                                                             */
/*   cal - fixed simulator calibration                                 */
/*   coeff - destination for the coefficients                          */
/*                                                                    */
/* Output:                                                            */
/*   int - 0 on success or -EINVAL for invalid inputs                  */
/* ------------------------------------------------------------------ */
int pedal_simplified_prepare(const PedalCalibration *cal,
                             PedalSimplifiedCoefficients *coeff)
{
    if (cal == NULL || coeff == NULL || cal->k_cal_b1 <= 0.0f ||
        cal->k_cal_b2 <= 0.0f || cal->delta_b1_m <= 0.0f ||
        cal->delta_b2_m <= 0.0f || cal->excitation_V <= 0.0f ||
        !isfinite(cal->crank_length_m) || !isfinite(cal->alpha_rad) ||
        !isfinite(cal->imu_roll_rad) || !isfinite(cal->imu_pitch_rad)) {
        return -EINVAL;
    }
    const float raw1 = cal->crank_length_m * cosf(cal->alpha_rad) /
                       (cal->k_cal_b1 * cal->delta_b1_m);
    const float raw2 = -cal->crank_length_m * sinf(cal->alpha_rad) /
                       (cal->k_cal_b2 * cal->delta_b2_m);
    *coeff = (PedalSimplifiedCoefficients) {
        .coeff_b1_Nm_per_uV = raw1 * 1e-6f / cal->excitation_V,
        .coeff_b2_Nm_per_uV = raw2 * 1e-6f / cal->excitation_V,
        .torque_offset_Nm = -raw1 * (cal->offset_zero_b1 + cal->offset_temp_b1) -
                            raw2 * (cal->offset_zero_b2 + cal->offset_temp_b2),
        .axis_x = -sinf(cal->imu_pitch_rad),
        .axis_y = cosf(cal->imu_pitch_rad) * sinf(cal->imu_roll_rad),
        .axis_z = cosf(cal->imu_pitch_rad) * cosf(cal->imu_roll_rad),
        .excitation_V = cal->excitation_V,
    };
    return isfinite(coeff->coeff_b1_Nm_per_uV) &&
           isfinite(coeff->coeff_b2_Nm_per_uV) &&
           isfinite(coeff->torque_offset_Nm) ? 0 : -EINVAL;
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_simplified_process(const PedalSensorSample       */
/*                                      *sample, const                 */
/*                                      PedalSimplifiedCoefficients    */
/*                                      *coeff, PedalResult *result)   */
/*                                                                    */
/* Description:                                                       */
/*   Applies the fixed coefficients to bridge differences and gyro    */
/*   readings. Rejects excitation changes because its reciprocal was */
/*   folded into the coefficients.                                     */
/*                                                                    */
/* Input:                                                             */
/*   sample - one sensor reading                                       */
/*   coeff - prepared fixed coefficients                               */
/*   result - calculation destination                                  */
/*                                                                    */
/* Output:                                                            */
/*   int - 0 on success or -EINVAL for invalid inputs                  */
/* ------------------------------------------------------------------ */
int pedal_simplified_process(const PedalSensorSample *sample,
                             const PedalSimplifiedCoefficients *coeff,
                             PedalResult *result)
{
    if (sample == NULL || coeff == NULL || result == NULL ||
        !isfinite(sample->excitation_V) ||
        fabsf(sample->excitation_V - coeff->excitation_V) > 1e-6f) {
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
    const float torque_Nm =
        coeff->coeff_b1_Nm_per_uV *
            (sample->half_bridge_uV[0] - sample->half_bridge_uV[1]) +
        coeff->coeff_b2_Nm_per_uV *
            (sample->half_bridge_uV[2] - sample->half_bridge_uV[3]) +
        coeff->torque_offset_Nm;
    const float omega_rad_s = coeff->axis_x * sample->gyro_rad_s[0] +
                              coeff->axis_y * sample->gyro_rad_s[1] +
                              coeff->axis_z * sample->gyro_rad_s[2];
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
