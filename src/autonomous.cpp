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
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
    Robot::start_task("SENSORCHECKING", Robot::balls_checking);


    //Tower 1
	Robot::intake({127, 127, 127, 0});
	Robot::move_to_pure_pursuit({{0, 0}, {1200, 100}, {1500, 0}}, {1600, -180, 45}, {1.5, 1.5, 1,5});
	int last_store_count = Robot::count();
	while(Robot::count() == last_store_count){
		delay(1);
	}
	delay(100);
	Robot::intake({0, 0, 0, 0});
	Robot::move_to({1100, -620, 90}, {2, 2, 2});
	Robot::move_to({1100, -840, 90}, {1, 1, 1}, {2, 2, 2});
	Robot::shoot_store(3, 2);

	//Tower 2
	while(Robot::UF.get_value() > 400){
		Robot::intake({127, 127, 127, 0});
	}
	delay(200);
    Robot::move_to({1100, -400, 90}, {3, 3, 3});
	Robot::move_to({1100, -400, 250}, {3, 3, 3});
	Robot::intake({127, 127, 127, -127});
	delay(400);
	Robot::intake({0, 0, 127, -127});
	Robot::toggle_outtake(600, 0);
	Robot::start_task("OUTTAKE1", Robot::balls_outtake);
	Robot::move_to({450, 1300, 247}, {1, 1, 1}, {1, 1, 2});
	Robot::kill_task("OUTTAKE1");
	Robot::R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	Robot::intake({127, 127, 127, 0});
	delay(500);
	Robot::move_to({-470, 870, 132}, {1, 1, 1}, {1, 1, 2});
	Robot::intake({0, 0, 0, 0});
	Robot::shoot_store(1, 1);

//	//Tower 3
    Robot::move_to({-380, 960, 132});
    Robot::intake({0, 0, 127, -127});
    Robot::toggle_outtake(600, 0);
    Robot::start_task("OUTTAKE2", Robot::balls_outtake);
	Robot::move_to({-1140, 1710, 221}, {1, 1, 1}, {1, 1, 2});
    Robot::kill_task("OUTTAKE2");
    Robot::intake({127, 127, 127, 0});
    Robot::move_to({-1790, 2930, 219}, {1, 1, 1}, {1, 2, 1});
    delay(200);
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({-2090, 2370, 178});
    Robot::move_to({-2320, 2320, 178}, {1, 1, 1}, {2, 2, 2});
    Robot::shoot_store(2, 2);

//   //Tower 4
    Robot::move_to({-2090, 2290, 180}, {2, 2, 2}, {1, 1, 1});
    Robot::intake({127, 127, 127, -127});
	delay(400);
	Robot::intake({0, 0, 127, -127});
    Robot::toggle_outtake(600, 1000);
    Robot::start_task("OUTTAKE3", Robot::balls_outtake); //ratchet for second to last ball
    Robot::move_to({-220, 2980, 338}, {2, 2, 1}, {1, 1, 2}); //point for second to last ball
    Robot::kill_task("OUTTAKE3");
    delay(200);
    Robot::intake({127, 127, 127, 0}); //intake second to last ball
    Robot::move_to({-8, 3340, 221}); // prelim point
    Robot::move_to({-230, 3490, 221}); // get last ball
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({-660, 3900, 221}); // into tower 4
    Robot::shoot_store(2, 1);

    //tower 5
    Robot::move_to({-565, 3900, 221});
    Robot::toggle_outtake(1000, 0);
    Robot::intake({127, 127, 127, -127});
    delay(400);
    Robot::intake({0, 0, 127, -127});
    Robot::start_task("OUTTAKE4", Robot::balls_outtake);
    Robot::move_to({455, 4950, 221}); // get last ball
    Robot::kill_task("OUTTAKE4");
    Robot::move_to({240, 5170, 223});
    Robot::intake({127, 127, 127, 0});
    Robot::move_to({850, 5600, 266});
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({770, 5680, 270}); //tower 5 THIS IS A FUCKING PIECE OF SHIT THAT NEVER FUCKING WORKS FIXME
    Robot::shoot_store(1, 2);
    Robot::move_to({850, 5600, 266});

    //Tower 6
    Robot::move_to({560, 5380, 394});
    Robot::intake({0, 0, 127, -127});
    delay(400);
    Robot::intake({0, 0, 127, -127});

    Robot::toggle_outtake(500, 0);
    Robot::start_task("OUTTAKE5", Robot::balls_outtake);
    Robot::move_to({830, 5230, 399});
    Robot::kill_task("OUTTAKE5");
    Robot::intake({127, 127, 127, 0});
    delay(1000);
    Robot::intake({0, 0, 0, 0});
    Robot::start_task("OUTTAKE6", Robot::balls_outtake);
    Robot::move_to({1330, 3520, 401});
    Robot::kill_task("OUTTAKE6");
    Robot::intake({127, 127, 127, 0});

    Robot::move_to({2510, 4090, 311});
    Robot::shoot_store(2, 1);

    //Tower 7
    Robot::intake({0, 0, 127, -127});
    delay(400);
    Robot::intake({0, 0, 127, -127});
    Robot::move_to({2320, 3920, 312});
    Robot::start_task("OUTTAKE7", Robot::balls_outtake);
    Robot::move_to({3140, 3300, 401});
    Robot::kill_task("OUTTAKE7");
    Robot::intake({127, 127, 127, 0});
    Robot::move_to({3800, 2110, 401}); //other ball
    Robot::move_to({4100, 2700, 359}); //prelim
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({4260, 2690, 357}); //tower
    Robot::shoot_store(2, 2);

    //Tower 8
    Robot::move_to({4010, 2670, 357}); //tower
    Robot::start_task("OUTTAKE8", Robot::balls_outtake);
    Robot::move_to({2810, 1520, 491}); //tower
    Robot::kill_task("OUTTAKE8");
    Robot::intake({127, 127, 127, 0});
    delay(300);
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({2480, 1200, 403}); //tower
    Robot::move_to({2610, 1020, 399}); //tower
    Robot::shoot_store(1, 1);
    Robot::intake({0, 0, 127, -127});
    delay(400);
    Robot::intake({0, 0, 127, -127});



}