#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous()
{
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("SENSORS", Robot::sensors);
	delay(100);


	//Tower 1
	std::vector<std::vector<double>> points1 {{0, 0}, {750, 0.1}, {750, 500}, {180, 1000}};
	Robot::reset_Balls(1);
	Robot::start_task("STORE", Robot::store);
	delay(200);
	Robot::move_to_pure_pursuit(points1);
	Robot::kill_task("STORE");
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(-1, false, "intakes");
	delay(50);
	Robot::intake(0);
	Robot::quickscore();
	Robot::reset_Balls(1, 1, true);
	Robot::intake(1);
	delay(100);
	if (Robot::LM1.get_value() == 1){
		while(Robot::LM1.get_value() == 1){
			delay(1);
		}
	}
	else {
		while(Robot::LM1.get_value() == 0){
			delay(1);
		}
		while(Robot::LM1.get_value() == 1){
			delay(1);
		}
	}
	Robot::intake(0);
	Robot::start_task("STORE1", Robot::store);
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE1");
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(-1, false, "intakes");
	delay(50);
	Robot::intake(0);
	Robot::move_to({600, 550, -160});
	delay(100);
	Robot::intake(-0.5);
	delay(1000);

	//Tower 2
	Robot::move_to({900, 650, -65});
	Robot::reset_Balls(1, 0);
	Robot::start_task("STORE2", Robot::store);
	Robot::move_to({980, 860, -65});
	std::vector<std::vector<double>> points2 {{980, 860}, {600, -600}, {2700, -500}, {2700, 690}};
	Robot::move_to_pure_pursuit(points2);
}

