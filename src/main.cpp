/** 
 * 
 * 45434A VEX Atomic Codebase
 * Created by Tristan Gwinn and other members of 45434A Atomic for the 2026-2027 VEX Override Season.
 * 
 * Date Updated: 9/22/26
 * Updated By: Tristan Gwinn
 * 
 * Completed:
 *  - Rewrote odom math to work inside of the drivetrain periodic
 *  - Rewrote drive curve functionality to be inside of drivetrain.h
 *  - modified pid.hpp and removed pid.cpp
 *  - Move robot config variables out of main.cpp
 *  - Got rid of trackingWheel.cpp
 *  - Create dedicated file for Command and subsystem initalization
 * 
 *  - Removed namespace "atomic" and "include/atomic/"
 *  - moved lose files to "deprecate/" in both "src/" and "include/"
 *  - created arm subsystem
 * 
 * Main to-do: 
 *  - Add functionality to read json files as trajectories (High Priority)
 *  - Rewrite motions as commands
 *  - Update Atomic logo to new pink version (Low Priority)
 * 	- Driver control recording/playback (Low Priority)
 * 
 */

/////
//
//  To download code on linux, run sudo chmod a+rw after connecting brain / controller
//
/////

#include "main.h"
#include "pros/imu.hpp"
#include "config.hpp"
#include "subsystems/subsystems.h"

#include "pros/llemu.hpp"

// brain image stuff
LV_IMAGE_DECLARE(logo);
bool logoOnBrain = false;

void initialize() {
	if (logoOnBrain){
		lv_obj_t *img = lv_image_create(lv_screen_active());
		lv_image_set_src(img, &logo);
		lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
	}else pros::lcd::initialize();   // to get rid of pros screen run: pros::lcd::shutdown();

    // Start the command scheduler task
    pros::Task commandSchedulerTask(update_loop);

    // setup and register subsystems and triggers
    initializeSubsystems();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}
