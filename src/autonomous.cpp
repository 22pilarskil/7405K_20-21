#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);

	std::vector<std::vector<double>> path {{.1,.1}, {1000, 0}, {1000, 1000}, {2000, 1000.01}};
	Robot::move_to_pure_pursuit(path, 1);

	/* test 1
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	delay(300);
	Robot::move_to(2000, 1000, 90);
	delay(300);
	Robot::intake(1);
	Robot::move_to(2000, -1000, 90, .7);
	delay(300);
	Robot::intake(0);
	Robot::move_to(0, 0, 0);
	*/

}
