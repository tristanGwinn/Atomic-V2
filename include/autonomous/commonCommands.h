#pragma once


#include "config.hpp"
#include "subsystems/subsystems.h"

/**
 * This class stores common commands used in autonomous.
 */
class CommonCommands {
public:
	/**
	 * Create a motion profiled trajectory from a set of points
	 */
    static Trajectory* profiledPath(units::V2Position p0, units::V2Position p1, units::V2Position p2, units::V2Position p3, Length deltaD = 0.7_in){
  	    TrajectoryGenerator generator(config::robot_kinematics, deltaD);
  	    generator.generateTrajectory(new CubicBezier(p0, p1, p2, p3));
	    return new Trajectory(generator.getTrajectory());
    }

	static Command* ScoreAtLevel(int level) {
		Command* scorePosition;
		switch(level){
			case 0:
				scorePosition = scorePos0;
			case 1:
				scorePosition = scorePos1;
			case 2:
				scorePosition = scorePos2;
			case 3:
				scorePosition = scorePos3;
			default:
        		return new InstantCommand([]() { std::cout << "INVALID SCORING POSITION" << std::endl; }, {});
		}
		return new Sequence({
            claw->levelCommand(true),
            scorePosition,
            (arm->holdPositionCommand()->with(lift->holdPositionCommand())),
			claw->levelCommand(false),
            new WaitCommand(150_msec)
		});
	}

	static Command* DropAndReturn() {
		return new Sequence({
            claw->levelCommand(false), new WaitCommand(7_sec),
            claw->levelCommand(true),
			resetArmLift
		});
	}

};