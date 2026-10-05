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

inline PID angular_pid = PID(0, 0, 0, 0, false);    // these are just here until old motions are removed
inline PID lateral_pid = PID(0, 0, 0, 0, false);

// Physical robot variables

constexpr Number lift_slope = 6.368 / 1.308;

constexpr Length track_width = 11.50_in;
constexpr Length wheel_diameter = 2.75_in;
constexpr AngularVelocity max_rpm = 450 * rpm;

constexpr LinearVelocity max_vel = 5_inps; // toLinear<AngularVelocity>(max_rpm, wheel_diameter);
constexpr LinearAcceleration max_accel = 3.40_mps2;     // max_accel = drivetrain force at max rpm / robot mass
constexpr double friction_coefficient = 2.0;

inline DifferentialKinematics *robot_kinematics
       = new DifferentialKinematics (
                                    track_width,
                                    max_vel,
                                    max_accel,
                                    2.0         // friction coefficient
                                );

}