#pragma once

#include "atomic/command/command.h"
#include "atomic/command/runCommand.h"

#include "hardware/Motor/MotorGroup.hpp"

class DriveSubsystem : public Subsystem {
    private:
        atomic::MotorGroup left_motors;
        atomic::MotorGroup right_motors;

        float deadband = 0;     // no deadband by default
        float minOutput = 0;    // no min output by default
        float curve = 1.0;      // no curve by default

        std::optional<double> pct;
        std::optional<atomic::ExpoDriveCurve> driveCurve;

    public:
        explicit DriveSubsystem(atomic::MotorGroup &leftmotors, atomic::MotorGroup &rightmotors,
                                float deadband, float minOutput, float curve)
        : left_motors(leftmotors), right_motors(rightmotors), deadband(deadband), minOutput(minOutput), curve(curve) 
        {
            resetAngle(); 
        }

        explicit DriveSubsystem(atomic::MotorGroup &leftmotors, atomic::MotorGroup &rightmotors)
        : left_motors(leftmotors), right_motors(rightmotors)
        {
            resetAngle();  
        }


        void periodic() override {
            // nothing here yet
        }

        void resetAngle() {
            left_motors.setAngle(0_stDeg);
            right_motors.setAngle(0_stDeg);
        }

        /**
         * Move the drivetrain motors at a signed percentage of voltage
         */
        void setPct(const double pct_left, const double pct_right) {
            this->left_motors.move(pct_left);
            this->right_motors.move(pct_right);
            pros::lcd::print(0, "left percent: %f", pct_left);
            pros::lcd::print(1, "right percent: %f", pct_right);
        }

        RunCommand* pctCommand(const double left, const double right) {
            // Create a new RunCommand
            // The lambda body is called at every update, in this case setting the intake percentage
            return new RunCommand(
                [this, left, right] () // Capture "this" and the percentage request
                {
                    this->setPct(left, right);
                },
                {this}
            );
        }

        RunCommand *arcade(pros::Controller &controller) {
            return new RunCommand(
                [this, controller]() mutable {
                    this->setPct((controller.get_analog(ANALOG_LEFT_Y) + controller.get_analog(ANALOG_RIGHT_X)) / 127.0,
                                 (controller.get_analog(ANALOG_LEFT_Y) - controller.get_analog(ANALOG_RIGHT_X)) / 127.0);
                },
                {this});
        }

        ~DriveSubsystem() override = default;
};
