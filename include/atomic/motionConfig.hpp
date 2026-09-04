#pragma once

#include "atomic/exitCondition.hpp"
#include "atomic/pid.hpp"

#include "atomic/ramsete/ramseteController.hpp"
#include "atomic/trajectory/kinematics.hpp"

#include "atomic/chassis/trackingWheel.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Motor/MotorGroup.hpp"

#include "units/Pose.hpp"
#include "units/Angle.hpp"

#include <functional>

// controller objects

extern const atomic::PID angular_pid;
extern const atomic::PID lateral_pid;

// extern const atomic::RameseteFollowSettings ramsete_settings;

extern const atomic::RamseteController ramsete_controller;
extern atomic::DifferentialKinematics robot_kinematics;

extern const std::function<units::Pose()> pose_getter;

// Physical robot variables
extern const Length track_width;
extern const Length wheel_diameter;
extern const AngularVelocity max_rpm;
extern const Mass robot_mass;

extern const Number drift_compensation;

// chassis electronics
extern atomic::MotorGroup left_motors;
extern atomic::MotorGroup right_motors;

extern atomic::V5InertialSensor imu;

// PID variables
extern const atomic::ExitConditionGroup<AngleRange> angular_exit_conditions;
extern const atomic::ExitConditionGroup<Length> lateral_exit_conditions;

extern const Number angular_slew;
extern const Number lateral_slew;
