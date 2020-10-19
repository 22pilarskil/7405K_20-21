#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous()
{
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::reset_Balls(1);
	Robot::start_task("SENSORS", Robot::sensors);
	delay(100);

	std::vector<std::vector<double>> points{{0, 0}, {750, 0.1}, {750, 500}, {180, 1000}};
	Robot::start_task("STORE", Robot::store);
	Robot::move_to_pure_pursuit(points);
	Robot::kill_task("STORE");
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(-1, false, "intakes");
	delay(50);
	Robot::intake(0);
	Robot::quickscore();
	Robot::reset_Balls(1, 1);
	Robot::start_task("STORE1", Robot::store);
	delay(500);
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(-1, false, "intakes");
	delay(50);
	Robot::move_to({600, 550, -135});
}

