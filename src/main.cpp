/** 
 * 
 * 45434A VEX Atomic Codebase
 * Created by Tristan Gwinn and other members of 45434A Atomic for the 2026-2027 VEX Override Season.
 * 
 * Date Updated: 9/23/26
 * Updated By: Tristan Gwinn
 * 
 * What's New:
 *  - Fixed odometry Pose tracking!!
 *  - Added temporary arm (chain bar) control
 * 
 * Main to-do: 
 *  - Add functionality to read json files as trajectories (High Priority)
 *  - Rewrite motions as commands
 *  - Update Atomic logo to new pink version (Low Priority)
 * 	- Driver control recording/playback (Low Priority)
 * 
 */

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

    // setup and register subsystems and setup triggers
    initializeSubsystems();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}
