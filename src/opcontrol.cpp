#include "main.h"
#include "Robot.h"
#include "PurePursuit.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	delay(100);
	Robot::start_task("DISPLAY", Robot::display);

	Robot::start_task("SENSORUPDATING", Robot::BallsUpdating);
	Robot::start_task("SENSORCHECKING", Robot::BallsChecking);

	Robot::start_task("DRIVE", Robot::drive);
	Robot::start_task("FPS", Robot::fps);


	// Robot::intake(-1, "intakes", false, false);
	// delay(600);
	// Robot::intake(0, "intakes");

	// while(Robot::UF.get_value() > 80) delay(1);
	// for(int i; i<2; i++) Robot::intake(1, "intakes", false, false);
	// delay(100);
	// Robot::intake(0, "intakes");

}
