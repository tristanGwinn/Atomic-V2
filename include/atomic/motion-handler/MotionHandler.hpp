#pragma once

#include <functional>
#include "units/units.hpp"
#include "units/Pose.hpp"

namespace atomic::motion_handler {

/**
 * @brief run a motion algorithm
 *
 * @note this algorithm is currently very similar to
 *       the one found in lemlib
 * 
 * @param f the motion function
 */
void move(std::function<void(void)> f);

/**
 * @brief check if the robot is in a motion
 * @return if the robot is executing a motion
 */
bool isMoving();

/**
 * @brief cancel the currently running motion, if it exists
 */
void cancel();

/**
 * @brief wait until the robot is within a radius of a given point
 */
void waitUntilPoint(units::V2Position target, Length radius, std::function<units::Pose()> poseGetter);

/**
 * @brief wait until the robot has traveled a given distance from where this function was called
 */
void waitUntilDistance(Length dist, std::function<units::Pose()> poseGetter);

} // namespace atomic::motion_handler