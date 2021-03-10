#include "main.h"
#include "Robot.h"
#include "PurePursuit.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	delay(100);
	// Robot::start_task("DISPLAY", Robot::display);
	Robot::driver = true;
	Robot::start_task("SENSORCHECKING", Robot::balls_checking);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	if(Robot::get_driver_type() == 1) {
		Robot::start_task("TUNEDRIVER", Robot::drive_tune);
		skills_driver_auton();
	} else {
		Robot::start_task("DRIVE", Robot::drive);
	}

}
