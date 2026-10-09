#pragma once

#include "deprecate/exitCondition.hpp"
#include "controllers/pid.hpp"

#include "trajectory/kinematics.hpp"

#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/MotorGroup.hpp"

#include "units/Angle.hpp"
#include "units/units.hpp"

#include <functional>

namespace config {

const int path_resolution = 100;    // this is the number of samples used for approximating path lengths

// NEED TUNED
constexpr double ramsete_beta = 45.0;
constexpr double ramsete_zeta = 0.4;

constexpr units::Pose ramsete_tolerance(2_in, 2_in, 1.5_stDeg);

constexpr Time default_timeout = 3.0_sec;
constexpr Length default_tolerance = 2_in;
constexpr Angle angle_tolerance = 2_stDeg;
inline PID angular_pid = PID(0, 0, 0, 0, false);    // these are just here until old motions are removed
inline PID lateral_pid = PID(0, 0, 0, 0, false);

// Physical robot variables

constexpr Number lift_slope = 6.368 / 1.308;

constexpr Length track_width = 11.50_in;
constexpr Length wheel_diameter = 2.75_in;
constexpr AngularVelocity max_rpm = 450 * rpm;

constexpr LinearVelocity max_vel = toLinear<AngularVelocity>(max_rpm, wheel_diameter);
constexpr AngularVelocity max_angular_vel = toAngular<LinearVelocity>(max_vel, M_PI * track_width);

constexpr LinearAcceleration max_accel = 2.76_mps2;     // max_accel = drivetrain force at max rpm / robot mass
constexpr double friction_coefficient = 2.0;

inline DifferentialKinematics *robot_kinematics
       = new DifferentialKinematics (
                                    track_width,
                                    max_vel,
                                    max_accel,
                                    2.0         // friction coefficient
                                );

}