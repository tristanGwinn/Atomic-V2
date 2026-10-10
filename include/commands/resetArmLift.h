#pragma once

#include <utility>

#include "units/units.hpp"

#include "command/command.h"
#include "command/commandScheduler.h"
#include "command/parallelCommandGroup.h"
#include "command/sequence.h"

#include "subsystems/lift.h"
#include "subsystems/arm.h"
#include "subsystems/claw.h"

class ResetArmLift : public Command {
    private:
        // subsystems
        LiftSubsystem *lift;
        ArmSubsystem *arm;
        ClawSubsystem *claw;

    public:
        ResetArmLift(LiftSubsystem *lift, ArmSubsystem *arm, ClawSubsystem* claw)
        : lift(lift), arm(arm), claw(claw) {}

        void initialize() override {
            printf("Resetting the Arm and Lift positions ...\n");
            claw->setLevel(true);
            lift->setTarget(lift->getPosition() - 4_in);
            arm->setTarget(0);
        }

        void execute() override {
            // no-op
            if ((arm->getPosition() - arm->getTarget()) < 100)
                arm->setPct(-0.2);
        }

        bool isFinished() override {
            return 
                abs(lift->getPosition() - lift->getTarget()) < 1_in &&
                (arm->getTarget() == -1) ? true : abs(arm->getPosition() - arm->getTarget()) < 3.0;
        }

        void end(bool interupted) override {
            lift->brakeMotors(BrakeMode::COAST);
            arm->brakeMotors(BrakeMode::COAST);
            printf("DONE\n");
            claw->setLevel(false);
        }

        std::vector<Subsystem *> getRequirements() override { return {lift, arm}; }

        ~ResetArmLift() override = default;
};
