#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	delay(100);
	std::vector<std::vector<double>> points {{0, 0}, {800, 0.1}, {700, 470}, {365, 760}};//, {700, 470}, {160, 970}};
	Robot::move_to_pure_pursuit(points, 1, 1);
	Robot::intake(1, false, true);
	delay(500);
	Robot::intake(0, false, true);
	//Robot::move_to(211, 211, -45);
	//Robot::move_to(0, -1000, 0);
	/*Robot::move_to(500, 0, 0, 1);
	delay(300);
    Robot::move_to(525, 525, -1s31);
    delay(300);
    Robot::move_to(360, 750, -131);
    delay(300);
	Robot::intake(1, false, true);
	delay(2000);
	Robot::intake(0, false, true);*/


}

// Power PID Testing
// Robot::move_to(500, 0, 0);
// delay(300);
// Robot::move_to(0, 0, 0);
// delay(300);
// Robot::move_to(1000, 0, 0);
// delay(300);;
// Robot::move_to(0, 0, 0);
// delay(300);
// Robot::move_to(1500, 0, 0);
// delay(300);
// Robot::move_to(0,0,0);

// Turn PID Testing
// Robot::move_to(0, 0, 90);
// delay(300);
// Robot::move_to(0, 0, 45);
// delay(300);
// Robot::move_to(0, 0, 180);
// delay(300);
// Robot::move_to(0, 0, 225);
// delay(300);
// Robot::move_to(0, 0, 0);

// Strafe PID Testing
// Robot::move_to(0, 1000, 0);
// delay(300);
// Robot::move_to(0, -1000, 0);
// delay(300);
// Robot::move_to(0, 0, 0);
