#pragma once

#include "command/command.h"
#include "subsystems/drivetrain.h"
#include "units/units.hpp"
#include "config.hpp"
#include "util.hpp"

class MoveToPoint : public Command {
    private:
        DriveSubsystem *drivetrain;

        V2Position target;
        Time timeout;
        Length tolerance;
        bool reversed;

        Time startTime = 0.0_msec;
        Angle initialAngle = 0_stDeg;
        std::optional<bool> prevSide = std::nullopt;
        LinearVelocity prevLateralOut = 0_mps;
        AngularVelocity prevAngularOut = 0_rps;

    public:
        MoveToPoint(DriveSubsystem *drivetrain, V2Position target, Length tolerance = config::default_tolerance, 
                    Time timeout = config::default_timeout, bool reversed = false) :
            drivetrain(drivetrain), target(target), tolerance(tolerance), timeout(timeout), reversed(reversed) {}

        void initialize() override { 
            startTime = from_msec(pros::millis()); 
            std::cout << "Moving to (" << target.x << ", " << target.y << ") ..." << std::endl; 

            initialAngle = drivetrain->getPose().angleTo(target);
        }

        void execute() override {
            // get pose
            const Pose pose = drivetrain->getPose();

            // calculate error
            const Length lateralError = pose.distanceTo(target) * cos(angleError(pose.orientation, pose.angleTo(target)));
            const Angle angularError = [&] {
                const Angle adjustedTheta = reversed ? pose.orientation + 180_stDeg : pose.orientation;
                return angleError(adjustedTheta, pose.angleTo(target));
            }();

            // check exit conditions
            /*if (settings.exitConditions.update(lateralError) && close) break;
            {
                const bool side = (pose.y - target.y) * -sin(initialAngle) <=
                                  (pose.x - target.x) * cos(initialAngle) + params.earlyExitRange;
                if (prevSide == std::nullopt) prevSide = side;
                const bool sameSide = side == prevSide;
                // exit if close
                if (!sameSide && params.minLateralSpeed != 0) break;
                prevSide = side;
            }*/

            // get lateral and angular outputs
            const LinearVelocity lateralOut = [&] -> LinearVelocity {
                // get raw output from PID
                auto out = config::lateral_pid.update(to_m(lateralError));

                // apply restrictions on maximum speed
                out = clamp(out, -config::max_vel, config::max_vel);
                out *= reversed ? -1 : 1;

                // slew except when settling
                // out = close ? out : slew(out, prevLateralOut, params.lateralSlew, helper.getDelta());

                // apply restrictions on minimum speed
                // if (!close && params.reversed) out = clamp(out, -params.maxLateralSpeed, -params.minLateralSpeed);
                // else if (!close && !params.reversed) out = clamp(out, params.minLateralSpeed, params.maxLateralSpeed);

                // update previous value
                prevLateralOut = out*mps;
                return out*mps;
            }();

            const AngularVelocity angularOut = [&] -> AngularVelocity {
                // if settling, disable turning
                // if (close) return 0;
                
                // get raw output from PID
                auto out = config::angular_pid.update(to_stRad(angularError));
                
                // apply restrictions on maximum speed
                out = clamp(out, -config::max_angular_vel, config::max_angular_vel);
                
                // slew except when settling
                // out = slew(out, prevAngularOut, params.angularSlew, helper.getDelta());
                
                // update previous value
                prevAngularOut = out*rps;
                return out*rps;
            }();

            // move the drivetrain
            drivetrain->setDriveVelocities({lateralOut, angularOut});
        }

        void end(bool interrupted) override { std::cout << "DONE" << std::endl; }


        bool isFinished() override { 

            // TODO

            return false;
        }

        std::vector<Subsystem *> getRequirements() override { return {drivetrain}; }
};
