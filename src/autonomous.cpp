#include "main.h"
#include "Robot.h"
using namespace pros;

/* Below is our programming skills path in its entirety. It takes advantage of the Robot::move_to, Robot::move_to_pure_pursuit,
and Robot::store functions to move to preset positions on the field and intake balls that we know to be there. It is
important to note that while in our programming skills video (https://www.youtube.com/watch?v=_H-iJ-kX9H8) it appears
that our robot is detecting and chasing down balls on the field, this is not the case. Since the field is set up the 
same way every time, we simply use odometry to find the points on the field where balls are located, and initiate a
single, uniform sequence each run. In effect, if the field is set up incorrectly (say a ball is shifted to the left 
or right by a significant margin) the odometry points will not work, and our skills path will fail. */

void autonomous()
{
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	delay(100);


	//Tower 1
	Robot::move_to({110, 250, 27}, {2, 2, 2}, {1.5, 1.5, 1.5});
	Robot::quickscore();

	//Tower 2
	Robot::move_to({300, 600, 179}, {2, 2, 2});
	Robot::intake(.6, "intakes");
	delay(1200);
	Robot::intake(-1, "intakes");
	delay(700);
	Robot::intake(0);
	Robot::move_to({-100, 600, 179}, {2.5, 2.5, 2.5});		
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(400);
	Robot::intake(0);
	Robot::move_to({-2020, 410, 93}, {2, 2, 2});
	Robot::quickscore();

	//Tower 3
	Robot::move_to({-2020, 600, 93}, {2, 2, 2});
	Robot::move_to({-2900, 580, 180}, {2, 2, 2}, {1, 1, 1}, 700);
	Robot::move_to({-3050, 580, 180}, {2, 2, 2});
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(400);
	Robot::intake(0);
	Robot::move_to({-4250, 300, 135});
	Robot::quickscore();

	//Tower 4
	Robot::move_to({-4006, 562, 135});
	Robot::move_to({-3980, 1870, 270}, {1, 1, 1}, {1, 1, 1}, 700);
	Robot::move_to({-3980, 2180, 270});
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(400);
	Robot::intake(0);
	Robot::move_to({-4100, 2615, 180});
	Robot::quickscore();

	//Tower 5
	Robot::move_to({-3900, 2615, 180}, {2, 2, 2});
	Robot::move_to({-3620, 3856, 280}, {2, 2, 2}, {1, 1, 1}, 700);
	Robot::move_to({-3600, 4180, 280}, {2, 2, 2});
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(400);
	Robot::intake(0);
	Robot::move_to({-4277, 4800, 221});
	Robot::quickscore();

	//Tower 6
	Robot::move_to({-4200, 4700, 221});
	Robot::move_to({-2625, 3940, 389}, {1, 1, 1}, {1, 1, 1}, 700);
	Robot::move_to({-2300, 3820, 389}, {2, 2, 2});
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(400);
	Robot::intake(0);
	Robot::move_to({-1990, 4733, 265});
	Robot::quickscore();

	//Tower 7
	Robot::move_to({-1990, 4500, 265}, {2, 2, 2});
	Robot::move_to({-1135, 4630, 360}, {1, 1, 1}, {1, 1, 1}, 700);
	Robot::move_to({-857, 4632, 360}, {2, 2, 2});
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(400);
	Robot::intake(0);
	Robot::move_to({250, 4930, 312});
	Robot::quickscore();

	//Tower 8
	Robot::move_to({27, 4800, 315});
	Robot::move_to({40, 3380, 445}, {1, 1, 1}, {1, 1, 1}, 700);
	Robot::move_to({20, 3100, 445});
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(400);
	Robot::intake(0);
	Robot::move_to({152, 2650, 358});
	Robot::quickscore();

	//Tower 9
	Robot::move_to({-100, 2650, 358}, {2, 2, 2});
	Robot::move_to({-111, 2620, 532}, {1, 1, 1}, {1, 1, 1}, 700);
	Robot::move_to({-500, 2590, 532}, {2, 2, 2});
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	delay(800);
	Robot::intake(0);
	Robot::move_to({-1387, 2580, 532}, {1, 1, 1}, {1, 1, 1}, 700);
	Robot::intake(1, "both");
	while(!Robot::FB.get_value()) delay(1);
	Robot::intake(-1, "intakes");
	delay(700);
	Robot::move_to({-1487, 2580, 532}, {2, 2, 2});
	Robot::quickscore();





	// //Tower 1
	// Robot::intake(1, "both");
	// delay(600);
	// Robot::intake(0);
	// Robot::move_to({560, 0, 0});
	// Robot::move_to({840, 340, -135});
	// Robot::move_to({250, 840, -135});
	// Robot::quickscore();

	// //Tower 2
	// Robot::move_to({600, 525, -135});
	// Robot::move_to({600, 525, 0});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({2200, 570, -10});
	// Robot::intake(1, "both");
	// delay(800);
	// Robot::intake(0);
	// Robot::move_to({2580, 470, -90});
	// Robot::move_to({2580, 740, -90}, {1.8, 1.8, 1.8});
	// Robot::quickscore();

	// //Tower 3
	// Robot::move_to({2580, 460, -90});
	// Robot::move_to({2600, 460, 9});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({4070, 175, 9});
	// Robot::intake(1, "both", true);
	// delay(1000);
	// Robot::intake(0);
	// Robot::move_to({4070, 175, -45});
	// Robot::move_to({4820, 900, -45});
	// Robot::quickscore();

	// //Tower 4
	// Robot::move_to({4540, 615, -45});
	// Robot::move_to({4540, 615, 114});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({3800, -1030, 114});
	// Robot::intake(1, "both", true);
	// delay(1000);
	// Robot::intake(0);
	// Robot::move_to({3800, -1440, 0});
	// Robot::move_to({4830, -1440, 0}, {1, 1, 1}, {1, .8, .7});
	// Robot::quickscore();

	// //Tower 5
	// Robot::move_to({4430, -1420, 0});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({4430, -1420, 74});
	// Robot::move_to({4610, -2420, 74});
	// Robot::intake(1, "both", true);
	// delay(1000);
	// Robot::intake(0);
	// Robot::move_to({5060, -3675, 45});
	// Robot::quickscore();

	// //Tower 6
	// Robot::move_to({4850, -3400, 45});
	// Robot::move_to({4850, -3400, 176});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({3165, -3370, 176});
	// Robot::intake(1, "both", true);
	// delay(1000);
	// Robot::intake(0);
	// Robot::move_to({2680, -3320, 89});
	// Robot::move_to({2680, -3620, 89});
	// Robot::quickscore();

	// //Tower 7
	// Robot::move_to({2630, -3350, 89}, {2, 2, 2});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({1230, -3350, 89}, {2, 2, 2});
	// Robot::move_to({1230, -3870, 89});
	// Robot::intake(1, "both");
	// delay(1000);
	// Robot::intake(0);
	// Robot::move_to({1230, -3950, 135});
	// Robot::move_to({560, -3810, 135});
	// Robot::move_to({410, -3850, 135});
	// Robot::quickscore();


	// //Tower 8
	// Robot::move_to({740, -3910, 135});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({715, -3880, 262}, {2, 2, 2});
	// Robot::move_to({690, -3520, 262}, {2, 2, 2});
	// Robot::intake(1, "both");
	// delay(1000);
	// Robot::intake(0);
	// Robot::move_to({460, -1620, 180});
	// Robot::quickscore();

	// //Tower 9
	// Robot::move_to({460, -1500, 0});
	// Robot::intake(-1, "intakes");
	// delay(400);
	// Robot::move_to({1170, -1500, 0});
	// Robot::intake(1, "both");
	// delay(1000);
	// Robot::intake(-1, "intakes");
	// delay(1000);
	// Robot::move_to({2060, -1500, 0}, {1, 1, 1}, {.8, 1, 1});
	// Robot::intake(1, "intakes");
	// delay(1000);
	// Robot::quickscore();
	// Robot::intake(1, "both", false, true);
	// delay(1000);
	// Robot::intake(-1, "intakes");
	// delay(500);
	// Robot::intake(0);
	// Robot::move_to({1800, -1500, 0});
}
