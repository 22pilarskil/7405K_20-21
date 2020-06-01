#include "main.h"
#include "Robot.h"
using namespace pros;

void opcontrol() {
	lcd::initialize();
	delay(100);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("DRIVE", Robot::drive);
	Robot::start_task("FPS", Robot::fps);
}


	