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

    std::vector<std::vector<double>> points1 {{0, 0}, {50, 0.1}, {2000, -700}, {2000, 700}, {1000, 0}, {0, 0}};
    Robot::move_to_pure_pursuit(points1, {1.2, 1.2, 1.2});


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
	Robot::move_to({800, 455, -160}, {5, 5, 5});
	Robot::intake(-0.8);
	delay(1000);

	//Tower 2
	Robot::move_to({900, 650, -65}, {10, 10, 10});
	Robot::reset_Balls(1, 0, true, true);
	Robot::start_task("STORE3", Robot::store);
	Robot::move_to({910, 880, -65}, {2, 2, 2}, {.8, .8, .8});
	Robot::move_to({910, 840, 15});
	Robot::move_to_pure_pursuit({{910, 840}, {2600, 500}}, {1, 1, .4});
	Robot::move_to({2640, 500, -90}, {2, 2, 2});
	Robot::kill_task("STORE3");
	Robot::move_to({2655, 710, -90}, {1, 1.5, 2}, {1, 1, 1});
	Robot::intake(-1, false, "indexer");
	delay(150);
	Robot::intake(1, false, "indexer");
	delay(1000);
	Robot::intake(0);
	Robot::move_to({2620, 300, -90}, {2, 2, 2});
	Robot::move_to({2620, 300, 0}, {3, 3, 3}, {1, 1, 1.5});


	//Tower 3
	Robot::reset_Balls(1, 0, true, false, false);
	Robot::start_task("STORE5", Robot::store);
	Robot::intake(1);
	delay(300);
	Robot::intake(0);
	Robot::move_to_pure_pursuit({{2620, 300}, {4500, 600}, {5000, 1000}}, {1.2, 1.2, .6});
	delay(500);
	Robot::kill_task("STORE5");
	Robot::intake(0);
	Robot::quickscore();
	Robot::move_to({4840, 560, -45});
	Robot::move_to({5020, 720, 110});

	//Tower 4
	Robot::reset_Balls();
	Robot::start_task("STORE6", Robot::store);
	Robot::move_to({3490, -1360, 125}, {2, 2, 2}, {.8, 1.2, .6});
	Robot::move_to({3490, -1360, 0}, {2, 2, 2});
	delay(500);
	Robot::kill_task("STORE6");
	Robot::move_to({4850, -1580, 0}, {1.5, 2, 1.5});
	Robot::brake("hold");
	Robot::intake(-1, false, "indexer");
	delay(50);
	Robot::intake(0);
	Robot::quickscore();
	Robot::move_to({4640, -1620, 0}, {2, 2, 2});
	Robot::move_to({4640, -1620, 90}, {3, 3, 3});

	//Tower 5
	Robot::reset_Balls(1, 1);
	Robot::start_task("STORE7", Robot::store);
	Robot::move_to_pure_pursuit({{4640, -1620}, {4700, -3250}, {5180, -3900}}, {1.2, 1.2, .7});
	Robot::kill_task("STORE7");
	Robot::intake(1, false, "indexer");
	delay(800);
	Robot::intake(0);

	//Tower 6
	Robot::move_to({4770, -3650, 45});
	Robot::move_to({4770, -3650, -154});
	Robot::reset_Balls(1, 0, false, true);
	Robot::start_task("STORE8", Robot::store);
	Robot::move_to({2750, -2550, -154}, {2, 2, 2.5}, {.8, .8, .4});
	Robot::move_to({2750, -2700, 90}, {2, 2, 2});
	Robot::move_to({2700, -3685, 90});
	while (!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("STORE8");
	Robot::quickscore(1);
	Robot::move_to({2650, -3500, 90});
	Robot::move_to({2650, -3500, 180});

	//Tower 7
	Robot::reset_Balls(1, 0, true, false, false);
	Robot::start_task("STORE9", Robot::store);
	Robot::intake(1);
	delay(50);
	Robot::intake(0);
	delay(200);
	Robot::kill_task("STORE9");
	Robot::move_to_pure_pursuit({{2650 -3500}, {720, -3800}, {180, -3990}}, {1, 1, .4});
	Robot::quickscore(1);

	//Tower 8
	Robot::reset_Balls(1, 0);
	Robot::move_to({584, -3740, Robot::IMU.get_rotation()}, {2, 2, 2});
	Robot::move_to({584, -3780, 270}, {2, 2, 2});
	Robot::start_task("STORE10", Robot::store);
	Robot::move_to({645, -3155, 270});
	Robot::move_to({660, -1550, 180});
	Robot::move_to({475, -1570, 180});
	Robot::quickscore(1);

	//Tower 9
	Robot::move_to({500, -1600, 180});
	Robot::move_to({500, -1600, 0});
	Robot::reset_Balls(1, 0, true, true);
	Robot::start_task("STORE11", Robot::store);
	Robot::move_to({2150, -1560, 0}, {3, 1.5, 1}, {.6, .6, .6});
	Robot::brake("coast");
	while(!Robot::store_complete){
		delay(1);
	}
	Robot::kill_task("DISPLAY");
	Robot::quickscore();
	Robot::intake(-1, false, "intakes");
	Robot::move_to({1000, -1560, 0});
	Robot::kill_task("STORE11");
	Robot::intake(1, true, "indexer");
	Robot::reset_Balls(1, 1, false, true);
	Robot::start_task("STORE12", Robot::store);


}
