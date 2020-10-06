#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("SENSORS", Robot::sensors);
	delay(100);
	Robot::move_to(0, 0, 90);
	delay(300);
	Robot::move_to(0, 0, 45);
	delay(300);
	Robot::move_to(0, 0, 180);
	delay(300);
	Robot::move_to(0, 0, 225);
	delay(300);
	Robot::move_to(0, 0, 0);

	// std::vector<std::vector<double>> points {{0, 0}, {800, 0.1}, {700, 470}, {300, 800}};
	// Robot::move_to_pure_pursuit(points, 1, 1);
	// while(Robot::balls_ejected_count() < 2 || Robot::balls_intook_count() < 3){
	// 	Robot::intake(1);
	// }
	// delay(300);
    // lcd::print(7, "%d", Robot::ball_count());
	// Robot::intake(0);
	// Robot::intake(1, false, "intakes");
	// Robot::move_to(500, 570, Robot::IMU.get_rotation());
	// Robot::move_to(570, 490, -45);
	// Robot::intake(0);

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
