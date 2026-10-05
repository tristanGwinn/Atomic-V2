#pragma once

#include "command/command.h"
#include "trajectory/trajectory.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"
#include "util.hpp"

class Ramsete : public Command {
    private:
        DriveSubsystem *drivetrain;

        float zeta;
        float beta;

        Time startTime = 0.0_msec;

        Trajectory *trajectory;

        LinearVelocity lastLeft = 0_mps, lastRight = 0_mps;

        DriveVelocities lastVelocities{0_mps, 0_radps};

    public:
        Ramsete(DriveSubsystem *drivetrain, Trajectory *trajectory,
                const float zeta = 0.0, const float beta = 0.0) :
            drivetrain(drivetrain), trajectory(trajectory), zeta(zeta), beta(beta) {}

        void initialize() override { startTime = from_msec(pros::millis()); printf("\nFollowing a trajectory with ramsete.\nFollowing Data:\n"); }

        void execute() override {
            const DriveVelocities controller_outputs = calculate();
            drivetrain->setDriveVelocities(controller_outputs);
            lastVelocities = controller_outputs;
        }

        DriveVelocities calculate(){
            const auto targetState = trajectory->sample(from_msec(pros::millis()) - startTime);
            auto currentPose = drivetrain->getPose();
            auto desiredPose = targetState.pose;

            const auto desiredOmega = targetState.angularVelocity;
            const auto desiredV = targetState.linearVelocity;

            const auto poseError = units::Pose(
                        desiredPose.x - currentPose.x,
                        desiredPose.y - currentPose.y,
                        desiredPose.orientation - currentPose.orientation
                    );

            const auto k =
                    2.0 * zeta * 
                    units::sqrt(square(desiredOmega).internal() + 
                    beta * square(desiredV).internal());

            
            const auto v_output = desiredV * units::cos(poseError.orientation) + k * poseError.x.internal() * mps;

            const auto omega_output = (desiredOmega.internal() + k * poseError.orientation.internal() +
                                       beta * desiredV.internal() * sinc(poseError.orientation.internal()) * poseError.y.internal())
                                       * radps;


            printf("\nTime: %f sec\n", (pros::millis() - to_msec(startTime)) / 1000.0);
            std::cout << "Current Pose: ( " 
                      << to_in(currentPose.x) << "in, "
                      << to_in(currentPose.y) << "in, " 
                      << to_stDeg(currentPose.orientation) << "deg )" <<
            std::endl;
            std::cout << "Desired Pose: ( " 
                      << to_in(desiredPose.x) << "in, "
                      << to_in(desiredPose.y) << "in, " 
                      << to_stDeg(desiredPose.orientation) << "deg )" <<
            std::endl;

            return DriveVelocities{v_output, omega_output};
        }

        void end(bool interrupted) override { std::cout << "DONE" << std::endl; }

        bool isFinished() override { return trajectory->totalTime() < from_msec(pros::millis()) - startTime; }

        std::vector<Subsystem *> getRequirements() override { return {drivetrain}; }
};
