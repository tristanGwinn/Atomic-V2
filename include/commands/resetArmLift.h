#pragma once

#include <utility>

#include "units/units.hpp"

#include "command/command.h"
#include "command/commandScheduler.h"
#include "command/parallelCommandGroup.h"
#include "command/sequence.h"

#include "subsystems/lift.h"
#include "subsystems/arm.h"
/**
 * this command will wait until the claw detects
 * an object, proceed to grab it, and then move the
 * arm (chain bar) and cascade lift to the desired position
 */
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
            arm->setTarget(10);
        }

        // before writing this, move to position functionality to the cascade lift subsystem
        void execute() override {
            lift->setTarget(lift->getPosition() - 4_in);
            arm->setTarget(10);
        }

        bool isFinished() override {
            return 
                units::abs(lift->getPosition() - lift->getTarget()) < 1_in &&
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
