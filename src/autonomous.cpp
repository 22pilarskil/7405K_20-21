#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous()
{
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::reset_Balls();
	Robot::start_task("SENSORS", Robot::sensors);
	delay(100);
	while(true){
		Robot::store();
		delay(5);
	}
	//std::vector<std::vector<double>> points{{0, 0}, {400, 0.1}, {250, 750}, {300, 800}};
	//Robot::store();
	//std::vector<double> pose {1000, 1000, 0};
	//Robot::move_to(pose);
	//Robot::move_to_pure_pursuit(points, true);
	// while(Robot::balls_ejected_count() < 2 || Robot::balls_intook_count() < 3){
	// 	Robot::intake(1);
	// }
	delay(300);
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
