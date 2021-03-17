#include "main.h"
#include "Robot.h"
#include "PurePursuit.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	Robot::brake("hold");
	lcd::initialize();
	delay(100);
	// Robot::start_task("DISPLAY", Robot::display);
	Robot::driver = true;
	Robot::start_task("SENSORCHECKING", Robot::balls_checking);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("RECORD", Robot::record_thread);
	if(Robot::get_driver_type() == 1) {
		Robot::start_task("TUNEAUTON", Robot::save_point);
	} else {
		Robot::start_task("DRIVE", Robot::drive);
	}

}
