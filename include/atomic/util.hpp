#pragma once

#include <cmath>
#include <vector>
#include "atomic/chassis/chassis.hpp"
#include "atomic/pose.hpp"

namespace atomic{

/**
 * @brief AngularDirection
 *
 * When turning, the user may want to specify the direction the robot should turn in.
 * This enum class has 3 values: CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, and AUTO
 * AUTO will make the robot turn in the shortest direction, and will be the most used value
 */
enum class AngularDirection { CW_CLOCKWISE, CCW_COUNTERCLOCKWISE };


/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 *
 */
float slew(float target, float current, float maxChange);

/**
 * @brief returns a random float between two values
 * 
 * @param min lower bound
 * @param max upper bound
 * @return float - the random value
 * 
 */
float random_float(float min, float max);

/**
 * @brief Return the sign of a number
 *
 * @param x the number to get the sign of
 * @return int - -1 if negative, 1 if positive
 *
 */
template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }

/**
 * @brief Return the average of a vector of numbers
 * 
 * @param values
 * @return float 
 * 
 */
float avg(std::vector<float> values);

/**
 * @brief Exponential moving average
 *
 * @param current current measurement
 * @param previous previous output
 * @param smooth smoothing factor (0-1). 1 means no smoothing, 0 means no change
 * @return float - the smoothed output
 *
 */
float ema(float current, float previous, float smooth);


/**
 * @brief a cheap polynomial normal pdf approximation, where sigma = 1
 * 
 * @param x x value
 * @return const float - approximation
 * 
 */
float cheap_norm_pdf(const float x);

/**
 * @brief the sinc function, sin(x)/x
 * 
 * @param x x value
 * @return float - output
 * 
 */
constexpr double sinc(double x){
    if (std::abs(x) < 1e-9) return 1.0 - 1.0 / 6.0 * x * x;
    else return std::sin(x) / x;
}

/**
 * @brief Convert degrees to radians
 *
 * @param deg degrees
 * @return float radians
 *
 */
constexpr float deg_to_rad(float angle_deg) { return angle_deg * M_PI / 180; }

/**
 * @brief Convert radians to degrees
 *
 * @param rad radians
 * @return float degrees
 *
 */
constexpr float rad_to_deg(float angle_rad) { return angle_rad * 180 / M_PI; }


// TODO: add function info, i.e. briefs and params, to the following funtions

/**
 * 
 */
float clamp(float input, float min, float max);

/**
 * 
 */
constexpr float clamp_min_voltage(float drive_output, float drive_min_voltage);

/**
 * 
 */
constexpr float percent_to_volts(float percent);

/**
 * 
 */
constexpr float volts_to_percent(float volts);


/**
 * @brief Calculate the error between two angles
 *
 * @param target the target angle
 * @param position the current angle
 * @param direction which direction the robot should turn in. Defaults to AUTO
 *
 * @return Angle the error between the two angles
 * 
 */
float angleError(float target, float position, bool radians = true,
                 AngularDirection direction = AngularDirection::AUTO);

/**
 * @brief Sanitize an angle so its positive and within the range of 0 to 2pi or 0 to 360
 *
 * @param angle the angle to sanitize
 * @param radians whether the angle is in radians or no. True by default
 * @return constexpr float
 */
constexpr float sanitizeAngle(float angle, bool radians = true);

/**
 * 
 */
constexpr float reduce_0_to_360(float angle);

/**
 * 
 */
constexpr float reduce_negative_pi_to_pi(float angle);

/**
 * 
 */
constexpr float reduce_negative_180_to_180(float angle);

/**
 * 
 */
constexpr float reduce_negative_90_to_90(float angle);

}   // namespace atomic