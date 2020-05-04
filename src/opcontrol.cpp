#include "main.h"
#include "Robot.h"
using namespace pros;

void opcontrol() {
	lcd::initialize();
	delay(100);
	Robot::start_tasks();
}


	