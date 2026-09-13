/** 
 * 
 * 45434A VEX Atomic Codebase
 * Created by Tristan Gwinn and other members of 45434A Atomic for the 2026-2027 VEX Override Season.
 * 
 * Date Updated: 9/12/26
 * Updated By: Tristan Gwinn
 * 
 * ToDo: 
 *  - Clean up code and directory (High Prority)
 *  - Add functionality to read json files as trajectories (High Priority)
 * 	- Test RAMSETE functionality (Med Priority)
 *  - Update Atomic logo to new pink version (Low Priority)
 * 	- Driver control recording/playback (Low Priority)
 * 	
 * What's New:
 *  - Temporary command based control of the lift has been added
 *  - Add a subsystem / action command scheduler to handle subsystem use (Current Focus)
 */

/////
//
//  To download code on linux, run sudo chmod a+rw after connecting brain / controller
//
/////

#include "main.h"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "atomic/driveCurve.hpp"
#include "atomic/motionConfig.hpp"
#include "atomic/chassis/odom.hpp"

#include "atomic/motions/ramseteTrajectoryFollower.hpp"

#include "atomic/trajectory/kinematics.hpp"
#include "atomic/trajectory/trajectoryGenerator.hpp"

#include "atomic/command/commandController.h"

#include "atomic/subsystems/lift.h"
#include "atomic/subsystems/drivetrain.h"

#include "pros/llemu.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
atomic::ExpoDriveCurve driveCurve(0.0, 1.0, 1.12);

// Physical robot variables
const Length track_width = 11.50_in;
const Length wheel_diameter = 2.75_in;
const AngularVelocity max_rpm = 450_rpm;

const LinearVelocity max_vel = 64.8_inps;           // max_vel = wheel_diameter * PI * max_rpm / 60_sec
const LinearAcceleration max_accel = 3.40_mps2;     // max_accel = drivetrain force at max rpm / robot mass

atomic::DifferentialKinematics robot_kinematics(
                                    track_width,
                                    max_vel,
                                    max_accel,
                                    2.0         // friction coefficient
                                );

atomic::MotorGroup left_motors({-11, -12}, 450_rpm);
atomic::MotorGroup right_motors({20, 19}, 450_rpm);

atomic::V5InertialSensor imu(2);

atomic::TrackingWheel vertical_tracker(
                                atomic::ReversibleSmartPort(20),    // tracking port
                                wheel_diameter,                     // diameter
                                track_width / 2                     // offset
                            );

atomic::TrackingWheel horizontal_tracker(
                                atomic::ReversibleSmartPort(18),    // tracking port
                                2.0_in,                             // diameter
                                -3.0_in                             // offset
                            );

atomic::Odometry odom({&imu}, {&vertical_tracker}, {&horizontal_tracker});

// PID variables
extern const atomic::PID angular_pid(0.05, 0, 0);
extern const atomic::PID lateral_pid(0.05, 0, 0);

const atomic::ExitConditionGroup<AngleRange> angular_exit_conditions({atomic::ExitCondition<AngleRange>(0.5_cDeg, 1000_msec)});
const atomic::ExitConditionGroup<Length> lateral_exit_conditions({atomic::ExitCondition(1.0_in, 2000_msec)});

const Number angular_slew = 1.0;
const Number lateral_slew = 1.0;

CommandController primary(pros::E_CONTROLLER_MASTER);   // set the controller for command triggers

atomic::MotorGroup lift_motors({-13, 7}, 600_rpm);  // lift motors

// Subsystem Objects
LiftSubsystem *lift;
DriveSubsystem *drivetrain;

// brain image stuff
LV_IMAGE_DECLARE(logo);
bool logoOnBrain = false;


/**
 * @brief This function runs the update scheduler at each frame with a consistent schedule
 *
 * @warning This function or alternative similar to it must be running to ensure the \refitem CommandScheduler is run
 */
[[noreturn]] void update_loop() {
	// Loop forever
	while (true) {
		// Store the start time
		auto start_time = pros::millis();

		// Run the command scheduler
		// This might be an expensive(Time wise) computation
		CommandScheduler::run();

		// Use delay until if this computation ends up being expensive, keeping loop time in check
		pros::c::task_delay_until(&start_time, 10);
	}
}


void initialize() {
	if (logoOnBrain){
		lv_obj_t *img = lv_image_create(lv_screen_active());
		lv_image_set_src(img, &logo);
		lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
	}
    else pros::lcd::initialize();   // to get rid of pros screen run: pros::lcd::shutdown();

    // Start the command scheduler task
    pros::Task commandSchedulerTask(update_loop);

    // add components to subsystems
    lift = new LiftSubsystem(lift_motors);
    drivetrain = new DriveSubsystem(left_motors, right_motors);

    // register subsystems
    CommandScheduler::registerSubsystem(drivetrain, drivetrain->arcade(primary));
    CommandScheduler::registerSubsystem(lift, lift->pctCommand(0.0));

    // primary.getTrigger(ANALOG_LEFT_Y)->whileTrue(lift->pctCommand(1.0));

    primary.getTrigger(DIGITAL_R1)->whileTrue(lift->pctCommand(1.0));
    primary.getTrigger(DIGITAL_L1)->whileTrue(lift->pctCommand(-1.0));

}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}
