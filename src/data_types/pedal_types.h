#ifndef PEDAL_TYPES_H
#define PEDAL_TYPES_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t sample_index;
    float time_s;             // [s] Time since start.
    float dt_s;               // [s] Simulator sample interval.
    float half_bridge_uV[4];  // [uV] B1.1, B1.2, B2.1, B2.2 relative to Vex/2.
    float excitation_V;       // [V] Bridge excitation.
    float accel_m_s2[3];      // [m/s^2] X, Y, Z acceleration.
    float gyro_rad_s[3];      // [rad/s] X, Y, Z angular velocity.
} PedalSensorSample;

typedef struct {
    float crank_length_m;     // [m] Crank length.
    float k_cal_b1;           // [1/(N*m)] Bridge 1 calibration factor.
    float k_cal_b2;           // [1/(N*m)] Bridge 2 calibration factor.
    float delta_b1_m;         // [m] Bridge 1 gauge spacing.
    float delta_b2_m;         // [m] Bridge 2 gauge spacing.
    float alpha_rad;          // [rad] Force projection angle.
    float offset_zero_b1;     // [1] Normalized bridge 1 zero offset.
    float offset_zero_b2;     // [1] Normalized bridge 2 zero offset.
    float offset_temp_b1;     // [1] Normalized bridge 1 thermal offset.
    float offset_temp_b2;     // [1] Normalized bridge 2 thermal offset.
    float imu_roll_rad;       // [rad] IMU installation roll.
    float imu_pitch_rad;      // [rad] IMU installation pitch.
    float excitation_V;       // [V] Fixed expected excitation.
} PedalCalibration;

typedef struct {
    float torque_Nm;          // [N*m] Tangential crank torque.
    float power_W;            // [W] Instantaneous mechanical power.
    float omega_rad_s;        // [rad/s] Signed crank angular velocity.
} PedalResult;

typedef struct {
    uint32_t samples;
    float average_torque_Nm;  // [N*m] Sample mean torque.
    float average_power_W;    // [W] Sample mean power.
} PedalSummary;

#endif // PEDAL_TYPES_H
