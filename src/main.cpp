/** 
 * 
 * 45434A VEX Atomic Framework and Codebase
 * Created by Tristan Gwinn and other members of 45434A Atomic for the 2026-2027 VEX Override Season.
 * 
 * Date Updated: 8/21/26
 * Updated By: Tristan Gwinn
 * 
 * Note:
 * 	As of now, nothing has been tested,
 *  and some functionality may be broken/or unintentionally left unfinished.
 * 
 * ToDo: 
 * 	It's okay to reference old code and other templates/frameworks to implement features
 *  - Realtime motion-profiled trajectory generation
 *  - Trajectory formatting
 * 	- The Ramsete path follow cannot currently read paths
 *  - Add simple movement logic
 * 		- Turn to face point
 *  	- Move to Pose via boomerang
 *  	- Turn to face heading
 *  	- Swing to face point
 *  	- Swing to face heading
 * 	- Many files need to be consistently formatted and organized
 * 	- Create a formatting guide
 * 	- Driver control recording/playback
 * 	
 * What's New:
 * 	- Many features were brought over from lemlib, examples below
 *  	- Chassis class
 * 		- Motion handler
 * 		- Units
 * 		- PID class
 * 		- Pose class
 * 	- Ramsete controller (untested)
 * 	- Trajectory following is partially implemented
 * 	- Logo can display on brainscreen
 * 
 */

#include "main.h"
#include "subsystems.hpp"

bool logoOnBrain = false;
LV_IMAGE_DECLARE(logo);

void initialize() {
	if (logoOnBrain){
		lv_obj_t *img = lv_image_create(lv_screen_active());
		lv_image_set_src(img, &logo);
		lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
	}



}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}