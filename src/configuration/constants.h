#ifndef PEDAL_CONSTANTS_H
#define PEDAL_CONSTANTS_H

// Fixed inputs from simulator/configs/bicicletta.yaml, perno.yaml, and installazione.yaml.
#define PEDAL_PI 3.14159265358979323846f // [1] Circle constant.
#define PEDAL_CRANK_LENGTH_M 0.1725f       // [m] Crank length.
#define PEDAL_SHAFT_OUTER_DIAMETER_M 0.012f // [m] Shaft outer diameter.
#define PEDAL_SHAFT_INNER_DIAMETER_M 0.0072f // [m] Shaft inner diameter.
#define PEDAL_YOUNG_MODULUS_PA 196000000000.0f // [Pa] Shaft Young's modulus.
#define PEDAL_GAUGE_FACTOR 2.05f          // [1] Strain gauge factor.
#define PEDAL_BRIDGE_1_SPACING_M 0.010f   // [m] First gauge pair spacing.
#define PEDAL_BRIDGE_2_SPACING_M 0.010f   // [m] Second gauge pair spacing.
#define PEDAL_RADIAL_DIRECTION_DEG 5.0f   // [deg] Installed radial direction.
#define PEDAL_FIRST_AXIS_DEG 0.0f         // [deg] First bridge axis.
#define PEDAL_ALPHA_DEG (-90.0f - (PEDAL_RADIAL_DIRECTION_DEG - PEDAL_FIRST_AXIS_DEG)) // [deg] Force projection.
#define PEDAL_EXCITATION_V 3.0f           // [V] Fixed bridge excitation.
#define PEDAL_OFFSET_ZERO_B1 0.0f         // [1] Bridge 1 zero offset.
#define PEDAL_OFFSET_ZERO_B2 0.0f         // [1] Bridge 2 zero offset.
#define PEDAL_OFFSET_TEMP_B1 0.0f         // [1] Bridge 1 thermal offset.
#define PEDAL_OFFSET_TEMP_B2 0.0f         // [1] Bridge 2 thermal offset.
#define PEDAL_IMU_ROLL_RAD 0.0f           // [rad] IMU installation roll.
#define PEDAL_IMU_PITCH_RAD 0.0f          // [rad] IMU installation pitch.

#define PEDAL_SAMPLE_RATE_HZ 200u         // [Hz] Simulator sample rate.
#define PEDAL_SAMPLE_PERIOD_MS (1000u / PEDAL_SAMPLE_RATE_HZ) // [ms] Replay interval.

#if 1000u % PEDAL_SAMPLE_RATE_HZ != 0u
#error "The sample rate must divide 1000 for millisecond scheduling"
#endif

#endif // PEDAL_CONSTANTS_H
