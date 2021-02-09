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
	//intake format: index 0,1 is the intakes, index 2 is the indexer, index 3 is the fly
	//every tower after 3 is inconsistent


    Robot::set_pass(false);
    //Tower 1 -------------------------------------------------------------------------------------------------------
	//intaking
	Robot::intake({127, 127, 80, 0}); //turning on intakes for the next two balls
	Robot::move_to_pure_pursuit({{0, 0}, {800, 1}, {1000, 0}}, {1590, -150, 43}, {1.5, 1.5, 1,5}); //pp for first two balls
	delay(100);
	Robot::move_to({1150, -530, 94}); //tower prelim
	Robot::intake({0, 0, 0, 0});
	Robot::move_to({1150, -810, 94}); //tower 1
	Robot::shoot_store(3, 2); //shooting 3, storing 2

	//Tower 2 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::intake({0, 0, 127, 0});
	int last_ejector = Robot::count();
	Robot::balls_intake_toggle(600, 0);
	Robot::start_task("OUTTAKE0", Robot::balls_intake);
    Robot::move_to({1020, 0, 90}); //backout
    Robot::intake({0, 0, 0, 0});
	Robot::move_to({1020, 0, 238}); //turning
	Robot::kill_task("OUTTAKE0");
	Robot::intake({0, 0, 127, -127});
	//first ball
	Robot::balls_intake_toggle(600, 0, false); //thread delay
	Robot::start_task("OUTTAKE1", Robot::balls_intake);
	Robot::move_to({550, 1290, 238}, {1, 1, 1}, {1, 1, 2}); //ball point
	Robot::kill_task("OUTTAKE1");
	if (Robot::count() - last_ejector < 2){
		Robot::intake({0, 0, -127, -127});
		delay(100);
		Robot::intake({0, 0, 127, -127});
		while(Robot::count() - last_ejector < 2){
			delay(1);
		}
	}
	
	Robot::shoot_store(0, 1);
	Robot::intake({0, 0, 127, 0});
	//tower
	Robot::move_to({-470, 870, 132}, {1, 1, 1}, {1, 1, 2}); //tower 2
	Robot::shoot_store(1, 1); //shoot 1, store 1


	//Tower 3 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::intake({0, 0, 127, -90});
    Robot::move_to({-380, 960, 221}, {1, 1, 1}, {3, 3, 1}); //back out
	//first ball
    Robot::balls_intake_toggle(600, 300, true); //thread delay
    Robot::start_task("OUTTAKE2", Robot::balls_intake);
	Robot::move_to({-1140, 1710, 221}, {1, 1, 1}, {1, 1, 2}); //ball point
	//second ball
    Robot::move_to({-1730, 2870, 221}, {1, 1, 1}, {1, 2, 1}); //ball point
    delay(100);
    Robot::kill_task("OUTTAKE2");
	//tower
    Robot::move_to({-2090, 2370, 178}); //prelim point
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({-2360, 2230, 181}); //tower 3
    Robot::shoot_store(2, 2); //shoot 2, store 2


    //Tower 4 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::intake({0, 0, 127, -127});
	last_ejector = Robot::count();
	Robot::balls_intake_toggle(600, 0);
	Robot::start_task("OUTTAKE3", Robot::balls_intake);
    Robot::move_to({-1870, 2380, 200}); //backout
    Robot::move_to({-1870, 2380, 334}); //turning
    if (Robot::count() - last_ejector < 2){
		Robot::intake({0, 0, -127, -127});
		delay(100);
		Robot::intake({0, 0, 127, -127});
	}
	//first ball
    Robot::move_to({-215, 2920, 333}, {1, 1, 1}, {1, 1, 3}); //ball point
	Robot::shoot_store(0, 1);
	Robot::intake({0, 0, 127, 0});
	Robot::kill_task("OUTTAKE3");
	//second ball
    Robot::balls_intake_toggle(600, 0, false); //thread delay
    Robot::start_task("OUTTAKE4", Robot::balls_intake);
    Robot::move_to({-220, 3550, 221}, {1, 1, 1}, {1, 1, 3}); //ball point
    Robot::kill_task("OUTTAKE4");
    Robot::intake({0, 0, 0, 0});
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0});
	//tower
    Robot::move_to({-620, 3890, 222}); //tower 4
    Robot::intake({0, 0, 0, 0});
    Robot::shoot_store(2, 1); //shoot 2, store 1

    Robot::set_pass(false);
	//Tower 5 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::balls_intake_toggle(600, 0);
	Robot::intake({0, 0, 127, -127});
	Robot::start_task("OUTTAKE5", Robot::balls_intake);
	Robot::move_to({-350, 3640, 222}); //backout
	delay(200);
	//first ball
	Robot::move_to({-350, 3640, 320}); //turning
	delay(400); //delay to open intakes before strafing out of tower
	Robot::kill_task("OUTTAKE5");
	Robot::move_to({780, 4570, 320}, {1, 1, 1}, {3, 3, 3}); //ball point
	Robot::intake({127, 127, 127, 0});
	delay(1000);
	//wall ball
	Robot::move_to({300, 5120, 220}, {1, 1, 1}, {1, 1, 3});
	Robot::intake({127, 127, 127, 0});
	//tower
	Robot::move_to({810, 5210, 268}); //prelim point
	Robot::intake({0, 0, 0, 0});
	Robot::move_to({810, 5700, 267}); //tower 5 very inconsistent
	Robot::shoot_store(2, 2); //shoot 1, store 2


	//Tower 6 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::intake({0, 0, 127, -127});
	last_ejector = Robot::count();
	Robot::balls_intake_toggle(600, 0);
	Robot::start_task("OUTTAKE7", Robot::balls_intake);
	Robot::move_to({830, 5210, 268}); //backout
	Robot::move_to({830, 5350, 435}); //turning
	Robot::kill_task("OUTTAKE7");
	if (Robot::count() - last_ejector < 2){
		Robot::intake({0, 0, -127, -127});
		delay(200);
		Robot::intake({0, 0, 127, -127});
	}
	//first ball
	Robot::move_to({1550, 3720, 435});
	Robot::shoot_store(0, 1); //storing one ball
	Robot::move_to({1900, 3440, 313}); //turning
	Robot::intake({0, 0, 127, 0}); //indexing while moving
	//tower
	Robot::move_to({2470, 4040, 313}, {1, 1, 1}, {2, 2, 2}); //tower 6
	Robot::shoot_store(1,1); //shoot 2, store 1


	//Tower 7 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::intake({0, 0, 127, -127});
	Robot::balls_intake_toggle(600, 0);
	Robot::start_task("OUTTAKE7", Robot::balls_intake);
	Robot::move_to({2160, 3860, 312});
	Robot::intake({0, 0, 127, -127}); //pooping
	//first ball
	Robot::move_to({3070, 3270, 401}); //ball point
	Robot::kill_task("OUTTAKE7");
	Robot::intake({127, 127, 127, 0}); //intske and indexer is continuous for next ball
	//second ball
	Robot::move_to({3750, 2110, 401}); //ball point
	//tower
	Robot::move_to({3580, 2570, 358}); //prelim tower
	Robot::intake({0, 0, 0, 0}); //intake shut off
	Robot::move_to({4290, 2670, 358}); //tower 7
	Robot::shoot_store(2, 2); //shoot 2, store 2


	//Tower 8 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::intake({0, 0, 127, -127}); //pooping, no intaking
	Robot::move_to({3900, 2670, 357}); //backout
	Robot::move_to({3900, 2670, 488}); //backout
	//first ball
	Robot::intake({0, 0, -127, -127});
	Robot::balls_intake_toggle(600, 0);
	Robot::start_task("OUTTAKE8", Robot::balls_intake); //ratchet
	Robot::move_to({2860, 1520, 488}); //ball point
	Robot::kill_task("OUTTAKE8");
	Robot::shoot_store(0, 1);
	Robot::intake({0, 0, 127, 0}); //indexer
	//tower
	Robot::move_to({2500, 1140, 399}); //prelim point
	Robot::move_to({2600, 1000, 399}); //tower 8
	Robot::shoot_store(1, 1); //shoot 1, store 1


	//Tower 9 -------------------------------------------------------------------------------------------------------
	//pooping
	Robot::move_to({2540, 1160, 400}); //backout
	Robot::intake({127, 127, 127, -127}); //pooping
	delay(400);
	//first ball
	Robot::balls_intake_toggle(600, 0);
	Robot::start_task("OUTTAKE8", Robot::balls_intake); //ratchet
	Robot::move_to({2480, 1200, 586}); //turning
	Robot::kill_task("OUTTAKE8");
	Robot::move_to({2070, 1470, 586}); //ball point
	Robot::shoot_store(0, 1); //making sure ball is in bot for the next ratchet
	//tower
	Robot::balls_intake_toggle(600, 0);
	Robot::start_task("OUTTAKE8", Robot::balls_intake); //ratchet for middle tower poles
	Robot::move_to({1340, 2090, 586}); //tower 9
	Robot::kill_task("OUTTAKE8");
	Robot::shoot_store(1, 3); //shoot 1, store 3
	Robot::balls_intake_toggle(400, 0);
	Robot::start_task("OUTTAKE8", Robot::balls_intake);
	Robot::move_to({1520, 1970, 586});
}