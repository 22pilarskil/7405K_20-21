

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
	 Robot::reset_Balls(1, 0);
	 Robot::start_task("STORE1", Robot::store);
	 delay(300);
	 std::vector<std::vector<double>> points1 {{0, 0}, {750, 0.1}, {750, 500}, {180, 1000}};
	 Robot::move_to_pure_pursuit(points1, {1.5, 1.5, 1.5}, 1, false, "intakes");
	 Robot::kill_task("STORE1");
	 Robot::brake("hold");
	 Robot::intake(0);
	 Robot::intake(1, false, "intakes");
	 while(Robot::LM1.get_value() == 0){
	 	delay(1);
	 }
	 Robot::intake(0);
	 Robot::intake(-1, false, "indexer");
	 delay(100);
	 Robot::intake(0);
	 Robot::reset_Balls(2, 1, false, true);
	 Robot::start_task("STORE2", Robot::store);
	 delay(10);
	 Robot::quickscore();
	 Robot::kill_task("STORE2");
	 Robot::move_to({800, 450, -160}, {5, 5, 5});
	 Robot::intake(-0.8);
	 delay(1000);

	// //Tower 2
	Robot::move_to({900, 650, -65}, {10, 10, 10});
	Robot::reset_Balls(0, 0, true);
	Robot::start_task("STORE3", Robot::store);
	Robot::move_to({910, 840, -65}, {1, 1, 1}, {.8, .8, .8});
	std::vector<std::vector<double>> points2 {{980, 860}, {1850, -160}, {2640, -350}};
	Robot::move_to_pure_pursuit(points2, {1, 1, 1});
	Robot::move_to({2620, -730, -90}, {1, 1, 1}, {1, 1, 1});
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE3");
	Robot::reset_Balls(2, 1, true, true);
	Robot::start_task("STORE4", Robot::store);
	Robot::move_to({2680, 650, -90}, {2, 2, 2}, {.8, 1.3, .8});
	Robot::brake("hold");
	lcd::print(7, "HERE");
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE4");
	delay(200);
	Robot::intake(-1, false, "indexer");
	delay(100);
	Robot::intake(0);
	Robot::quickscore();
	Robot::reset_Balls(1, 1, true, true);
	Robot::start_task("STORE5", Robot::store);
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE5");
	Robot::quickscore(0);
	Robot::intake(-1, false, "intakes");
	delay(200);
	Robot::intake(0);
	Robot::move_to({2620, 450, -90}, {1, 1, 1});

	//Tower 3
	Robot::move_to({2720, 450, 0}, {2, 2, 2}, {1, 1, 1});
	Robot::intake(1, true, "both");
	delay(1000);
	Robot::intake(0);
	Robot::move_to({4180, 380, -90});
	Robot::reset_Balls(1, 0);
	Robot::start_task("STORE6", Robot::store);
	Robot::move_to({4220, 740, -90}, {1, 1, 1});
	delay(100);
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE6");
	Robot::reset_Balls(2, 1, true, true);
	Robot::start_task("STORE7", Robot::store);
	Robot::move_to({4500, 600, -45}, {1, 1, 2});
	Robot::move_to({4970, 680, -45}, {1, 1, 2}); //-60, -80
	Robot::move_to({4980, 780, -45});
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE7");
	Robot::quickscore(false);
	Robot::reset_Balls(1, 1, true, true);
	Robot::start_task("STORE8", Robot::store);
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::intake(-1);
	delay(50);
	Robot::intake(0);
	Robot::kill_task("STORE8");
	Robot::move_to({4800, 550, -45});
	delay(100);
	Robot::move_to({4700, 500, 45});
	Robot::intake(1, true, "both");
	delay(1000);
	Robot::intake(0);
	Robot::move_to({5000, 640, 110});

	// //Tower 4
	// Robot::reset_Balls();
	// Robot::start_task("STORE9", Robot::store);
	// Robot::move_to({3500, -1370, 125}, {2, 2, 2}, {.8, .8, .8});
	// Robot::move_to({3500, -1370, 0}, {2, 2, 2});
	// while(!Robot::store_complete){
	// 	delay(1);
	// }
	// Robot::kill_task("STORE9");
	// Robot::reset_Balls(2, 1, true, true);
	// Robot::move_to({4890, -1620, 0}, {1, 1, 1}, {.8, .8, .8});
	// Robot::brake("hold");
	// Robot::quickscore();
	// Robot::intake(1, false, "intakes");
	// while(Robot::LM1.get_value() == 0){
	// 	delay(1);
	// }
	// Robot::intake(-1, false, "intakes");
	// delay(50);
	// Robot::intake(0);
	// Robot::move_to({4640, -1620, 0});
	// Robot::move_to({4640, -1620, 90});
	// Robot::intake(1, true, "both");
	// delay(1000);
	// Robot::intake(0);



}


void with_pid(){
	Robot::move_to({0, 3000, 0});
}

void without_pid(){
	Robot::mecanum(0,127,0);
	delay(1100);
	Robot::mecanum(0,0,0);
}


