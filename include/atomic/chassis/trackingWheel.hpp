#pragma once

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

namespace atomic {
    class TrackingWheel {
    public:
        /**
         * @brief Create a new tracking wheel
         *
         * @param encoder the optical shaft encoder to use
         * @param wheelDiameter the diameter of the wheel
         * @param distance distance between the tracking wheel and the center of rotation in inches
         * @param gearRatio gear ratio of the tracking wheel, defaults to 1
         *
         */
        TrackingWheel(pros::adi::Encoder* encoder, float wheelDiameter, float distance, float gearRatio = 1);
        
        /**
         * @brief Create a new tracking wheel
         *
         * @param encoder the v5 rotation sensor to use
         * @param wheelDiameter the diameter of the wheel
         * @param distance distance between the tracking wheel and the center of rotation in inches
         * @param gearRatio gear ratio of the tracking wheel, defaults to 1
         *
         */
        TrackingWheel(pros::Rotation* encoder, float wheelDiameter, float distance, float gearRatio = 1);
        
        /**
         * @brief Create a new tracking wheel
         *
         * @param motors the motor group to use
         * @param wheelDiameter the diameter of the wheel
         * @param distance half the track width of the drivetrain in inches
         * @param rpm theoretical maximum rpm of the drivetrain wheels
         *
         */
        TrackingWheel(pros::MotorGroup* motors, float wheelDiameter, float distance, float rpm);
        
        /**
         * @brief Reset the tracking wheel position to 0
         *
         */
        void reset();

        /**
         * @brief Get the distance traveled by the tracking wheel
         *
         * @return float distance traveled in inches
         *
         */
        float getDistanceTraveled();

        /**
         * @brief Get the offset of the tracking wheel from the center of rotation
         *
         * @return float offset in inches
         *
         */
        float getOffset();
        
        /**
         * @brief Get the type of tracking wheel
         *
         * @return int - 1 if motor group, 0 otherwise
         *
         */
        int getType();
    
    private:
        float diameter;
        float distance;
        float rpm;
        pros::adi::Encoder* encoder = nullptr;
        pros::Rotation* rotation = nullptr;
        pros::MotorGroup* motors = nullptr;
        float gearRatio = 1;
};
}   // atomic