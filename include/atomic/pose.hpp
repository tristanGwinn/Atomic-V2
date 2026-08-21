#pragma once

#include <string>

namespace atomic {

/**
 * @brief A pose in 2D space
 *
 * A pose is a position and heading in 2D space.
 * Representing a pose as a single object makes it easier to work with,
 * especially in motion algorithms and position tracking.
 *
 * The Pose class overloads operators so you can easily add, subtract, multiply, etc.
 * 
 * - from lemlib
 */
class Pose {
    public:
        float x;
        float y;
        float theta;

        /**
         * @brief Create a new pose
         *
         * @note Pose is unitless. It is up to the user to ensure that the units are consistent.
         *
         * @param x component
         * @param y component
         * @param theta heading. Defaults to 0
         *
         */
        Pose(float x, float y, float theta = 0);

        /**
         * @brief Checks equality between this Pose and another Pose
         *
         * @param other The other object.
         * @return Whether the two objects are equal.
         */
        bool operator==(const Pose& other) const;

        /**
         * @brief Add a pose to this pose
         *
         * @note heading is not modified, and is taken from this pose
         *
         * @param other other pose
         * @return Pose
         *
         */
        Pose operator+(const Pose& other) const;

        /**
         * @brief Subtract a pose from this pose
         *
         * @note heading is not modified, and is taken from this pose
         *
         * @param other other pose
         * @return Pose
         *
         */
        Pose operator-(const Pose& other) const;

        /**
         * @brief Multiply a pose by this pose (dot product)
         *
         * @note heading is not considered in this operation
         *
         * @param other other pose
         * @return Pose
         *
         */
        float operator*(const Pose& other) const;

        /**
         * @brief Multiply a pose by a float
         *
         * @note heading is not considered in this operation
         *
         * @param other float
         * @return Pose
         *
         */
        Pose operator*(const float& other) const;

        /**
         * @brief Divide a pose by a float
         *
         * @note heading is not considered in this operation
         *
         * @param other float
         * @return Pose
         *
         */
        Pose operator/(const float& other) const;

        /**
         * @brief Linearly interpolate between two poses
         *
         * @note heading is not considered in this operation
         *
         * @param other the other pose
         * @param t t value
         * @return Pose
         *
         */
        Pose lerp(Pose other, float t) const;

        /**
         * @brief Get the distance between two poses
         *
         * @param other the other pose
         * @return float
         *
         */
        float distance(Pose other) const;

        /**
         * @brief Get the angle between two poses
         *
         * @note heading is not considered in this operation
         *
         * @param other the other pose
         * @return float in radians
         *
         */
        float angle(Pose other) const;

        /**
         * @brief Rotate a pose by an angle
         *
         * @note heading is not considered in this operation
         *
         * @param angle angle in radians
         * @return Pose
         *
         */
        Pose rotate(float angle) const;
};

/**
 * @brief Format a pose
 *
 * @param pose
 * @return std::string
 * 
 */
std::string format_as(const Pose& pose);
}   // namespace atomic
