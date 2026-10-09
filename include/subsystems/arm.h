#pragma once

#include "command/command.h"
#include "command/runCommand.h"

#include "controllers/pid.hpp"

#include "hardware/Motor/MotorGroup.hpp"
#include "pros/imu.hpp"

// this is a subsystem class for the chain bar
class ArmSubsystem : public Subsystem {
    private:
        MotorGroup motor;

        // im not using hadrware wrappers here bcuz they dont return roll or yaw.
        pros::Imu imu;
        pros::Imu chassis_imu;

        double max_init_delta = 0.001;    // this should be pretty low
        double prev_position = 0;

        std::optional<double> pos_offset;
        double position = 0;

        PID pid;

        std::optional<double> pct;
        std::optional<double> voltage;
        std::optional<double> target;

    public:
        explicit ArmSubsystem(MotorGroup &motors, pros::Imu &inertial, pros::Imu &chassis_inertial, const PID &pid)
         : motor(motors), imu(inertial), chassis_imu(chassis_inertial), pid(pid) {
            calibrateSensors();
        }

        void calibrateSensors() {
            imu.tare();
            motor.setAngle(0_stDeg);
        }

        void periodic() override {
            position = this->getPosition() - pos_offset.value_or(0);
            
            // this is a really gross way to do this, but wtv
            if (!pos_offset.has_value() && (fabs(position - prev_position) < max_init_delta && position > 26.0)) {
                pos_offset = position;
            }
            prev_position = position;

            if (!voltage.has_value() && target.has_value()) {
                const auto control_out = pid.update(position);
                // printf("arm position: %f \n", position);
                // printf("arm error: %f \n", position.value() - target.value());
                // printf("arm control output: %f \n", control_out.internal());
                motor.move(control_out / 100);
            }
        }

        double getPosition() const {
            double arm_roll = imu.get_roll();
            arm_roll -= chassis_imu.get_roll() + 5;    // account for the roll of the chassis imu.

            // convert the roll into a rotation value (where 180 is the facing up)
            auto pos = (arm_roll < 0) ?
                        360 + arm_roll : arm_roll; 
            return pos;    
        }

        double getTarget() {
            return target.value_or(-1);
        }

        /**
         * Move the lift motors at a signed percentage of voltage
         */
        void setPct(const double pct) {
            this->motor.move(pct);
        }

        void setTarget(double target) {
            this->target = clamp(target, 0, 260);
            pid.setTarget(
                this->target.value()
            );
            voltage = std::nullopt;
            pid.setTarget(target);
            this->target = target;
            voltage = std::nullopt;
        }

        void brakeMotors(BrakeMode brake_mode) {
            voltage = std::nullopt;
            target = std::nullopt;
            motor.setBrakeMode(brake_mode);
            motor.brake();
        }

        void stopAndHold() {
            brakeMotors(BrakeMode::HOLD);
        }

        FunctionalCommand *positionCommand(double angle, double threshold = 3.0) {
            std::cout << "The arm target position is set to " << angle 
                      << "° with a tolerance of " << threshold << "°" << std::endl;
        Time start_time = from_msec(pros::millis());
        return new FunctionalCommand(
            [this, angle]() { this->setTarget(angle);
                                     }, [this, angle]() { 
                                         this->setTarget(angle);
                                     }, [this](bool _) {
                                         printf("The arm is within tolerence of its target.\n");
                                         // this->target = std::nullopt;
                                     }, [this, threshold, angle]() {
                                         return abs(this->getPosition() - angle) < threshold;
                                     }, {this});
        }

        FunctionalCommand *holdPositionCommand() {
        return new FunctionalCommand([this]() {
                                         this->stopAndHold();
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
        }

        FunctionalCommand *dropArmCommand() {
        return new FunctionalCommand([this]() {
                                         this->brakeMotors(BrakeMode::COAST);
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
        }

        /**
         * this command was take from the annotated example provided in echo's documentation
         * I left the annotations in for future reference in writing other commnads
         */
        RunCommand* pctCommand(const double pct) {
            // Create a new RunCommand
            // The lambda body is called at every update, in this case setting the lift percentage
            return new RunCommand(
                [this, pct] () // Capture "this" and the percentage request
                {
                    this->setPct(pct); // Set the percentage of the lift to the request
                },
                {this}  // Add "this", the pointer to this subsystem that is currently running.
                        // It is important to ensure that all subsystems that are being utilized in a command are properly
                        // Freed to allow that command to run.
            );
        }

        ~ArmSubsystem() override = default;
};
