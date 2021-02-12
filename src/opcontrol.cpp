#include "main.h"
#include "Robot.h"
#include "PurePursuit.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	delay(100);
	lcd::initialize();
	delay(100);
	// Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("SENSORCHECKING", Robot::balls_checking);

	Robot::start_task("DRIVE", Robot::drive);
	Robot::start_task("FPS", Robot::fps);
}
