#include "algorithms/algorithms.h"
#include "configuration/constants.h"

#include <errno.h>
#include <math.h>

static const float TAYLOR_HALF = 0.5f;
static const float TAYLOR_SIXTH = 0.166666667f;
static const float TAYLOR_TWENTY_FOURTH = 0.041666667f;
static const float TAYLOR_ONE_HUNDRED_TWENTIETH = 0.008333333f;

/* Reduce to |r| <= pi/4, then use cos(r) through r^4 and sin(r) through r^5.
 * The quarter-turn mapping restores the signs and swaps sin/cos if needed.
 */
static void approximate_sincos(float angle, float *sine, float *cosine)
{
    // Wrap into [-pi, pi], then choose the nearest multiple of pi/2.
    const float wrapped = remainderf(angle, 2.0f * PEDAL_PI);
    const int quarter_turns = (int)roundf(wrapped * (2.0f / PEDAL_PI));
    // The residual angle lies within approximately [-pi/4, pi/4].
    const float r = wrapped - quarter_turns * (PEDAL_PI * 0.5f);
    const float r2 = r * r;
    // Evaluate the even cosine and odd sine Taylor polynomials in r^2.
    const float cos_r = 1.0f + r2 * (-TAYLOR_HALF + r2 * TAYLOR_TWENTY_FOURTH);
    const float sin_r = r * (1.0f + r2 * (-TAYLOR_SIXTH +
                                         r2 * TAYLOR_ONE_HUNDRED_TWENTIETH));

    // Restore the quadrant of the original angle.
    switch ((quarter_turns % 4 + 4) % 4) {
    case 0:
        *cosine = cos_r;
        *sine = sin_r;
        break;
    case 1:
        *cosine = -sin_r;
        *sine = cos_r;
        break;
    case 2:
        *cosine = -cos_r;
        *sine = -sin_r;
        break;
    default:
        *cosine = sin_r;
        *sine = -cos_r;
        break;
    }
}

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
    // verify if fixed geometry and excitation are usable
    if (cal == NULL || coeff == NULL || cal->k_cal_b1 <= 0.0f ||
        cal->k_cal_b2 <= 0.0f || cal->delta_b1_m <= 0.0f ||
        cal->delta_b2_m <= 0.0f || cal->excitation_V <= 0.0f ||
        !isfinite(cal->crank_length_m) || !isfinite(cal->alpha_rad) ||
        !isfinite(cal->imu_roll_rad) || !isfinite(cal->imu_pitch_rad)) {
        return -EINVAL;
    }
    // Approximate each fixed installation angle .
    float sin_alpha, cos_alpha, sin_pitch, cos_pitch, sin_roll, cos_roll;
    approximate_sincos(cal->alpha_rad, &sin_alpha, &cos_alpha);
    approximate_sincos(cal->imu_pitch_rad, &sin_pitch, &cos_pitch);
    approximate_sincos(cal->imu_roll_rad, &sin_roll, &cos_roll);

    // Combine crank length, bridge calibration, and force direction for each bridge.
    const float raw1 = cal->crank_length_m * cos_alpha /
                       (cal->k_cal_b1 * cal->delta_b1_m);
    const float raw2 = -cal->crank_length_m * sin_alpha /
                       (cal->k_cal_b2 * cal->delta_b2_m);
    // Convert uV to V/Vo with one reciprocal of the fixed excitation voltage.
    const float inverse_excitation = 1.0f / cal->excitation_V;
    const float bridge_scale = 1e-6f * inverse_excitation;
    // Store bridge gains, the combined zero/temperature torque correction,
    // and the crank-axis direction used to project gyro readings.
    *coeff = (PedalSimplifiedCoefficients) {
        .coeff_b1_Nm_per_uV = raw1 * bridge_scale,
        .coeff_b2_Nm_per_uV = raw2 * bridge_scale,
        .torque_offset_Nm = -raw1 * (cal->offset_zero_b1 + cal->offset_temp_b1) -
                            raw2 * (cal->offset_zero_b2 + cal->offset_temp_b2),
        .axis_x = -sin_pitch,
        .axis_y = cos_pitch * sin_roll,
        .axis_z = cos_pitch * cos_roll,
        .excitation_V = cal->excitation_V,
    };
    // Reject coefficients that would produce invalid torque values.
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
    //  bridge gains are valid if false.
    if (sample == NULL || coeff == NULL || result == NULL ||
        !isfinite(sample->excitation_V) ||
        fabsf(sample->excitation_V - coeff->excitation_V) > 1e-6f) {
        return -EINVAL;
    }
    // Reject non-finite bridge and gyro readings.
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
    // Multiply each bridge difference by its torque gain and add the offset.
    const float torque_Nm =
        coeff->coeff_b1_Nm_per_uV *
            (sample->half_bridge_uV[0] - sample->half_bridge_uV[1]) +
        coeff->coeff_b2_Nm_per_uV *
            (sample->half_bridge_uV[2] - sample->half_bridge_uV[3]) +
        coeff->torque_offset_Nm;
    // Project the gyro vector onto the prepared crank axis to get rad/s.
    const float omega_rad_s = coeff->axis_x * sample->gyro_rad_s[0] +
                              coeff->axis_y * sample->gyro_rad_s[1] +
                              coeff->axis_z * sample->gyro_rad_s[2];
    // Keep invalid or overflowing calculations out of the result.
    if (!isfinite(torque_Nm) || !isfinite(omega_rad_s) ||
        !isfinite(torque_Nm * omega_rad_s)) {
        return -EINVAL;
    }
    // Mechanical power in watts is torque times angular speed.
    *result = (PedalResult) {
        .torque_Nm = torque_Nm,
        .power_W = torque_Nm * omega_rad_s,
        .omega_rad_s = omega_rad_s,
    };
    return 0;
}
