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

        double max_init_delta = 0.1;    // this should be somewhere between 0.6 and 1
        double prev_position = 0;

        std::optional<double> pos_offset;
        std::optional<double> position;

        std::optional<double> pct;

        PID pid;

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
            
            if (!pos_offset.has_value() && (fabs(position.value() - prev_position) < max_init_delta && position.value() > 17.0)) {
                pos_offset = position;
            }
            prev_position = position.value();

            printf("arm roll: %f \n", imu.get_roll());
            printf("arm position: %f \n", position);
            printf("arm position offset: %f \n", pos_offset.value_or(0));

            if (!voltage.has_value() && target.has_value()) {
                 const auto control_out = pid.update(position.value());
                 printf("arm error: %f \n", position.value() - target.value());
                 printf("arm control output: %f \n", control_out);
                 motor.move(control_out);
            }
        }

        double getPosition() const {
            double arm_roll = imu.get_roll();
            arm_roll -= chassis_imu.get_roll();    // account for the roll of the chassis imu

            // convert the roll into a rotation value (where 180 is the facing up)
            auto pos = (arm_roll < 0) ?
                        fabs(arm_roll) : 360 - arm_roll; 
            return pos;    
        }

        /**
         * Move the lift motors at a signed percentage of voltage
         */
        void setPct(const double pct) {
            this->motor.move(pct);
        }

        void setTarget(double target) {
            pid.setTarget(target);
            this->target = target;
            voltage = std::nullopt;
        }

        void brakeMotors(BrakeMode brake_mode) {
            motor.setBrakeMode(brake_mode);
            motor.brake();
        }

        void stopAndHold() {
            voltage = std::nullopt;
            target = std::nullopt;
            brakeMotors(BrakeMode::HOLD);
        }

        FunctionalCommand *positionCommand(double angle, double threshold = 8.0) {
        return new FunctionalCommand(
            [this, angle]() { this->setTarget(angle);
                                     }, [this, angle]() { this->setTarget(angle); }, [](bool _) {
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
