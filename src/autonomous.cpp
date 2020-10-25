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
	 std::vector<std::vector<double>> points1 {{0, 0}, {750, 0.1}, {750, 500}, {220, 960}};
	 Robot::move_to_pure_pursuit(points1, {1.2, 1.2, 1.2}, 1, false, "intakes");
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
	Robot::reset_Balls(1, 0, true, true);
	Robot::start_task("STORE3", Robot::store);
	Robot::move_to({910, 840, -65}, {2, 2, 2}, {.8, .8, .8});
	Robot::move_to({910, 840, 15});
	Robot::move_to_pure_pursuit({{910, 840}, {2660, 500}}, {1, 1, .4});
	Robot::move_to({2680, 500, -90}, {2, 2, 2});
	Robot::move_to({2680, 730, -90}, {1, 2.5, 2}, {1, 3, 1});
	Robot::intake(-1, false, "indexer");
	delay(50);
	Robot::intake(0);
	Robot::quickscore(false);
	delay(200);
	Robot::move_to({2620, 300, -90}, {2, 2, 2});
	Robot::move_to({2620, 300, 0}, {2, 2, 2});


	// //Tower 3
	Robot::reset_Balls(1, 0, false);
	Robot::intake(1);
	delay(100);
	Robot::intake(0);
	Robot::start_task("STORE5", Robot::store);
	Robot::move_to_pure_pursuit({{2620, 300}, {4500, 600}, {5100, 1000}}, {1.2, 1.2, .6});
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::intake(-1, false, "indexer");
	delay(50);
	Robot::intake(0);
	Robot::quickscore(false);
	Robot::move_to({4800, 500, -45});
	Robot::move_to({5020, 680, 110});
	
	//Tower 4
	Robot::reset_Balls();
	Robot::start_task("STORE6", Robot::store);
	Robot::move_to({3490, -1360, 125}, {2, 2, 2}, {.8, 1.2, .6});
	Robot::move_to({3490, -1360, 0}, {2, 2, 2});
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE6");
	Robot::reset_Balls(2, 1, false, true);
	Robot::move_to({4850, -1580, 0}, {1.5, 2, 1.5}, {.8, .8, .8});
	Robot::brake("hold");
	Robot::intake(-1, false, "indexer");
	delay(50);
	Robot::intake(0);
	Robot::quickscore();
	Robot::move_to({4640, -1620, 0});
	Robot::move_to({4640, -1620, 90});

	//Tower 5
	Robot::reset_Balls(1, 1);
	Robot::start_task("STORE7", Robot::store);
	Robot::move_to_pure_pursuit({{4640, -1620}, {4700, -3250}, {5150, -4000}}, {1.2, 1.2, .7});
	Robot::kill_task("STORE7");
	Robot::intake(1, false, "indexer");
	delay(800);

	//Tower 6
	Robot::move_to({4770, -3650, 45});
	Robot::move_to({4770, -3650, -135});
	Robot::reset_Balls();
	Robot::start_task("STORE8", Robot::store);
	Robot::move_to({2750, -2850, -175});
	Robot::move_to({2750, -2700, 90});
	Robot::move_to({2650, -3700, 90});
	Robot::intake(-1, false, "intakes");
	delay(100);
	Robot::intake(0);
	Robot::quickscore(false);


}


void with_pid(){
	Robot::move_to({0, 3000, 0});
}

void without_pid(){
	Robot::mecanum(0,127,0);
	delay(1100);
	Robot::mecanum(0,0,0);
}