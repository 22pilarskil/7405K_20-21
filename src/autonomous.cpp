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
	Robot::reset_Balls(1, 0);
	Robot::start_task("STORE1", Robot::store);
	delay(300);
	Robot::move_to_pure_pursuit(points1);
	Robot::kill_task("STORE1");
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(0);
	delay(5000);
	Robot::intake(-1, false, "intakes");
	delay(200);
	Robot::intake(0);
	Robot::quickscore();
	Robot::reset_Balls();
	Robot::intake(1);
	while (Robot::get_data()[0] == 0){
		delay(1);
	}
	Robot::intake(0);
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(0);
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(0);
	Robot::move_to({600, 550, -160}, false, 0, false, "both", {10, 10, 10});
	Robot::intake(-0.8);
	delay(1000);

	//Tower 2
	Robot::move_to({900, 650, -65}, false, 0, false, "both", {10, 10, 10});
	Robot::reset_Balls(0, 0, false, true);
	Robot::start_task("STORE3", Robot::store);
	Robot::move_to({900, 800, -65});
	std::vector<std::vector<double>> points2 {{980, 860}, {1850, -160}, {2690, -300}};
	Robot::move_to_pure_pursuit(points2);
	Robot::move_to({2700, -730, -90});
	Robot::move_to({2760, 660, -90});
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE3");
	Robot::quickscore();
	Robot::move_to({2760, 560, -90});
	Robot::reset_Balls(1, 1);
	Robot::start_task("STORE4", Robot::store);
	Robot::move_to({2700, 660, -90});
	Robot::intake(1, false, "intakes");
	while(Robot::LM1.get_value() == 0){
		delay(1);
	}
	Robot::intake(0);
	Robot::quickscore();
}

