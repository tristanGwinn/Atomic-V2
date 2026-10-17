#pragma once

#include <utility>

#include "command/command.h"
#include "command/commandScheduler.h"
#include "command/waitCommand.h"
#include "command/parallelCommandGroup.h"
#include "command/sequence.h"

#include "subsystems/subsystems.h"

/**
 * this command will wait until the claw detects
 * an object, proceed to grab it, and then move the
 * arm (chain bar) and cascade lift to the desired position
 */
class Score : public Command {
    private:
        // subsystems
        LiftSubsystem *lift;
        ArmSubsystem *arm;

        // Score positions for the arm and lift
        // angle (degrees), height (inches)
        std::array<double, 2> score_positions;  // it might be better to make this a struct or some other data type

        // maybe a bool or enum to determine order of arm and lift movement
        // may also want to motion profile or smth; problem for future me
    public:
        Score(LiftSubsystem *lift, ArmSubsystem *arm, std::array<double, 2> positions)
        : lift(lift), arm(arm), score_positions(std::move(positions)) {}

        void initialize() override {
            lift->setTarget(from_in(score_positions[1]));
            arm->setTarget(score_positions[0]);
            printf("Moving arm and lift to scoring positions ...\n");
        }

        // before writing this, move to position functionality to the cascade lift subsystem
        void execute() override { 
            lift->setTarget(from_in(score_positions[1]));
            arm->setTarget(score_positions[0]);
        }

        bool arePositionsTolerable() {
            return (
                (abs(lift->getPosition() - lift->getTarget()) < 3.0_cm) && (abs(arm->getPosition() - arm->getTarget()) < 3.0)
            );
        }

        bool isFinished() override { return arePositionsTolerable(); }

        void end(bool interupted) override { 
            // lift->brakeMotors(BrakeMode::HOLD);
            // arm->brakeMotors(BrakeMode::HOLD);
            std::cout << "Score command finished!\n" << std::endl;    
        }

        std::vector<Subsystem *> getRequirements() override { return {lift, arm}; }

        ~Score() override = default;
};
