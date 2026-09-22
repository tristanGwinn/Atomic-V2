#pragma once

#include "deprecate/exitCondition.hpp"
#include "controllers/pid.hpp"

#include "controllers/ramsete.hpp"
#include "trajectory/kinematics.hpp"

#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/MotorGroup.hpp"

#include "units/Angle.hpp"
#include "units/units.hpp"

#include <functional>

namespace config {

// extern const RameseteFollowSettings ramsete_settings;

inline PID angular_pid = PID(0, 0, 0, 0, false);
inline PID lateral_pid = PID(0, 0, 0, 0, false);

// Physical robot variables
constexpr Length track_width = 11.50_in;
constexpr Length wheel_diameter = 2.75_in;
constexpr AngularVelocity max_rpm = 450 * rpm;

constexpr LinearVelocity max_vel = 64.8_inps;           // max_vel = wheel_diameter * PI * max_rpm / 60_sec
constexpr LinearAcceleration max_accel = 3.40_mps2;     // max_accel = drivetrain force at max rpm / robot mass

inline DifferentialKinematics robot_kinematics
       = DifferentialKinematics (
                                    track_width,
                                    max_vel,
                                    max_accel,
                                    2.0         // friction coefficient
                                );

}