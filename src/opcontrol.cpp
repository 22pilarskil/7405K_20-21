#include "main.h"
#include "Robot.h"
#include "PurePursuit.h"
using namespace pros;

void opcontrol() {
	lcd::initialize();
	delay(100);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("DRIVE", Robot::drive);
	Robot::start_task("FPS", Robot::fps);
  Robot::start_task("VISION", Robot::vis_sense);
}


	
