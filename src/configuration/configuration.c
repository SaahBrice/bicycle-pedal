#include "configuration/configuration.h"
#include "configuration/constants.h"

#include <errno.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Creates: int pedal_configuration_load(PedalCalibration *cal)       */
/*                                                                    */
/* Description:                                                       */
/*   Converts the simulator's geometry into the bridge calibration    */
/*   used by both algorithms. The zero offsets and IMU rotation match */
/*   the supplied ideal simulator configuration.                       */
/*                                                                    */
/* Input:                                                             */
/*   cal - destination for calibrated values                          */
/*                                                                    */
/* Output:                                                            */
/*   int - 0 on success or -EINVAL for invalid geometry               */
/* ------------------------------------------------------------------ */
int pedal_configuration_load(PedalCalibration *cal)
{
    if (cal == NULL || PEDAL_SHAFT_INNER_DIAMETER_M < 0.0f ||
        PEDAL_SHAFT_OUTER_DIAMETER_M <= PEDAL_SHAFT_INNER_DIAMETER_M ||
        PEDAL_YOUNG_MODULUS_PA <= 0.0f || PEDAL_GAUGE_FACTOR <= 0.0f) {
        return -EINVAL;
    }

    const float section_modulus_m3 =
        PEDAL_PI * (powf(PEDAL_SHAFT_OUTER_DIAMETER_M, 4.0f) -
                    powf(PEDAL_SHAFT_INNER_DIAMETER_M, 4.0f)) /
        (32.0f * PEDAL_SHAFT_OUTER_DIAMETER_M);
    const float k_cal = PEDAL_GAUGE_FACTOR /
        (2.0f * PEDAL_YOUNG_MODULUS_PA * section_modulus_m3);

    *cal = (PedalCalibration) {
        .crank_length_m = PEDAL_CRANK_LENGTH_M,
        .k_cal_b1 = k_cal,
        .k_cal_b2 = k_cal,
        .delta_b1_m = PEDAL_BRIDGE_1_SPACING_M,
        .delta_b2_m = PEDAL_BRIDGE_2_SPACING_M,
        .alpha_rad = PEDAL_ALPHA_DEG * PEDAL_PI / 180.0f,
        .offset_zero_b1 = PEDAL_OFFSET_ZERO_B1,
        .offset_zero_b2 = PEDAL_OFFSET_ZERO_B2,
        .offset_temp_b1 = PEDAL_OFFSET_TEMP_B1,
        .offset_temp_b2 = PEDAL_OFFSET_TEMP_B2,
        .imu_roll_rad = PEDAL_IMU_ROLL_RAD,
        .imu_pitch_rad = PEDAL_IMU_PITCH_RAD,
        .excitation_V = PEDAL_EXCITATION_V,
    };
    return 0;
}
