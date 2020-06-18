#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::move_to(1000, 1000, 45);
	delay(100);
	Robot::move_to(1000, 0, -45);
	delay(10000);
	Robot::move_to(0, 0, 0);

}
