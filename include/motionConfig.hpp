#pragma once

#include "exitCondition.hpp"
#include "controllers/pid.hpp"

#include "controllers/ramsete.hpp"
#include "trajectory/kinematics.hpp"

#include "chassis/trackingWheel.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/MotorGroup.hpp"

#include "units/Pose.hpp"
#include "units/Angle.hpp"

#include <functional>

// controller objects

// extern const PID angular_pid;
// extern const PID lateral_pid;

// extern const RameseteFollowSettings ramsete_settings;

extern const RamseteController ramsete_controller;
extern DifferentialKinematics robot_kinematics;

extern const std::function<units::Pose()> pose_getter;

// Physical robot variables
extern const Length track_width;
extern const Length wheel_diameter;
extern const AngularVelocity max_rpm;
extern const Mass robot_mass;

extern const Number drift_compensation;

// chassis electronics
extern MotorGroup left_motors;
extern MotorGroup right_motors;

extern V5InertialSensor imu;

// PID variables
// extern const ExitConditionGroup<AngleRange> angular_exit_conditions;
// extern const ExitConditionGroup<Length> lateral_exit_conditions;

// extern const Number angular_slew;
// extern const Number lateral_slew;
