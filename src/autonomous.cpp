#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::move_to(1000, 1000, 45);
	Robot::intake(1);
	delay(300);
	Robot::move_to(2000, 0, 45, 80);
	delay(300);
	Robot::intake(0);
	Robot::move_to(2000, 1000, 90);
	delay(300);
	Robot::intake(1);
	Robot::move_to(2000, -1000, 90, 80);
	delay(300);
	Robot::intake(0);
	Robot::move_to(0, 0, 0);

}
