#pragma once

#include <utility>

#include "units/units.hpp"

#include "command/command.h"
#include "command/commandScheduler.h"
#include "command/parallelCommandGroup.h"
#include "command/sequence.h"

#include "subsystems/lift.h"
#include "subsystems/arm.h"

class ResetArmLift : public Command {
    private:
        // subsystems
        LiftSubsystem *lift;
        ArmSubsystem *arm;

    public:
        ResetArmLift(LiftSubsystem *lift, ArmSubsystem *arm)
        : lift(lift), arm(arm) {}

        void initialize() override {
            lift->setTarget(lift->getPosition() - 4_in);

            // dont move the arm unless needed
            if (abs(arm->getPosition()) < 60.0 ) arm->brakeMotors(BrakeMode::COAST);
            else arm->setTarget(10);
        }

        void execute() override {
            // no-op
        }

        bool isFinished() override {
            return 
                abs(lift->getPosition() - lift->getTarget()) < 1_in &&
                abs(arm->getPosition() - arm->getTarget()) < 20;
        }

        void end(bool interupted) override {
            lift->brakeMotors(BrakeMode::COAST);
            arm->brakeMotors(BrakeMode::COAST);
            printf("Reset Arm and Lift positions\n");
        }

        std::vector<Subsystem *> getRequirements() override { return {lift, arm}; }

        ~ResetArmLift() override = default;
};
