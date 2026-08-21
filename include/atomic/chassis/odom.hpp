#pragma once

#include "atomic/chassis/chassis.hpp"
#include "atomic/pose.hpp"

// the odometry modeling uses the pilons' algorithm, and the code is mostly copied from lemlib
namespace atomic {
/**
 * @brief Set the sensors to be used for odometry
 *
 * @param sensors the sensors to be used
 * @param drivetrain drivetrain to be used
 */
void setSensors(atomic::OdomSensors sensors, atomic::Drivetrain drivetrain);

/**
 * @brief Get the pose of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return Pose
 */
Pose getPose(bool radians = false);

/**
 * @brief Set the Pose of the robot
 *
 * @param pose the new pose
 * @param radians true if theta is in radians, false if in degrees. False by default
 */
void setPose(Pose pose, bool radians = false);

/**
 * @brief Get the speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return atomic::Pose
 */
Pose getSpeed(bool radians = false);

/**
 * @brief Get the local speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return atomic::Pose
 */
Pose getLocalSpeed(bool radians = false);

/**
 * @brief Estimate the pose of the robot after a certain amount of time
 *
 * @param time time in seconds
 * @param radians False for degrees, true for radians. False by default
 * @return atomic::Pose
 */
Pose estimatePose(float time, bool radians = false);

/**
 * @brief Update the pose of the robot
 *
 */
void update();

/**
 * @brief Initialize the odometry system
 *
 */
void init();

}   // namespace atomic
