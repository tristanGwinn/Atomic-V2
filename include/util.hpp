#pragma once

#include <cmath>
#include <vector>
#include "motion-handler/MotionHandler.hpp"

/**
 * @brief this macro can be used to greatly simplify passing motion algorithms to the motion handler
 *
 * @b Example:
 * @code {.cpp}
 * void autonomous() {
 *   MOVE_CUSTOM(simpleMotion())
 * }
 * @endcode
 */
#define MOVE_CUSTOM(f) motion_handler::move([&] { f; });

// wait until a condition is met
#define WAIT_UNTIL(condition)                                                  \
  do {                                                                         \
    pros::delay(5);                                                            \
  } while (!(condition))


using namespace units;

struct DriveVelocities {
    LinearVelocity v = 0_mps;
    AngularVelocity omega = 0_radps;
};

struct DriveOutputs {
        Number left;
        Number right;
};

/**
 * @brief AngularDirection
 *
 * AUTO will make the robot turn in the shortest direction, and will be the most used value
 */
enum class AngularDirection {
    CW_CLOCKWISE, /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO /** turn in the direction with the shortest distance to target */
};

/**
 * @brief SlewDirection
 *
 * Slew may only need to be applied when the value being slewed is increasing, decreasing.
 * This enum class help make calls to slew more readable
 */
enum class SlewDirection { INCREASING, DECREASING, ALL };

/**
 * @brief calculate desaturated motor outputs
 *
 * Given a lateral output and an angular output, calculate the left and right side drivetrain velocities such that the
 * drivetrain output does not exceed 1
 *
 * @param lateralOutput
 * @param angularOutput
 *
 * @return DriveOutputs
 */
DriveOutputs desaturate(Number lateralOutput, Number angularOutput);

/**
 * @brief constrain the change in a value over time
 *
 * @param target the requested new value of the changing value
 * @param current the value to be constrained
 * @param maxChangeRate the maximum rate of change
 * @param deltaTime the change in time since the last iteration
 * @param directionLimit in which direction to restrict the change. All directions by default
 *
 * @return Number the value with the constrained change
 */
Number slew(Number target, Number current, Number maxChangeRate, Time deltaTime,
            SlewDirection restrictDirection = SlewDirection::ALL);

Curvature getSignedTangentArcCurvature(units::Pose start, units::V2Position end);

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
/*template <typename T>*/ constexpr Number sgn(Number value) { return value < 0.0 ? -1.0 : 1.0; }

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
constexpr float ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}

/**
 * @brief a cheap polynomial normal pdf approximation, where sigma = 1
 * 
 * @param x x value
 * @return const float - approximation
 * 
 */
// float cheap_norm_pdf(const float x);

/**
 * @brief the sinc function, sin(x)/x
 * 
 * @param x x value
 * @return float - output
 * 
 */
constexpr Number sinc(Number x){
    if (std::abs(x) < 1e-9) return 1.0 - 1.0 / 6.0 * x * x;
    else return std::sin(x) / x;
}

// TODO: add function info, i.e. briefs and params, to the following funtions

/**
 * 
 */
Number clamp(Number input, Number min, Number max);

// float clamp_min_voltage(float drive_output, float drive_min_voltage);

/**
 * @brief convert a percentage, -1 to 1, to a voltage value, -127 to 127 
 */
Number percent_to_volts(Number percent);

/**
 * @brief convert a voltage value, -127 to 127, to a percentage, -1 to 1
 */
Number volts_to_percent(Number volts);

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
Angle angleError(Angle target, Angle position,
                 std::optional<AngularDirection> direction = std::nullopt);

/**
 * @brief Sanitize an angle so its positive and within the range of 0 to 2pi or 0 to 360
 *
 * @param angle the angle to sanitize
 * @param radians whether the angle is in radians or no. True by default
 * @return constexpr Angle
 */
constexpr Angle sanitizeAngle(Angle angle, bool radians = true);