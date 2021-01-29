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



    bool pass = false;
    //Tower 1
	//intaking
	Robot::intake({127, 127, 127, 0}); //turning on intakes for the next two balls
	Robot::move_to_pure_pursuit({{0, 0}, {1200, 100}, {1500, 0}}, {1600, -180, 45}, {1.5, 1.5, 1,5}); //pp for first two balls
	int last_store_count = Robot::count();
	while(Robot::count() == last_store_count){
		delay(1);
	}
	delay(100);
	Robot::intake({0, 0, 0, 0}); //shutting off intake after wall ball

	//tower
	Robot::move_to({1100, -620, 90}, {2, 2, 2}); //tower prelim
	Robot::move_to({1100, -840, 90}, {1, 1, 1}, {2, 2, 2}); //tower 1
	Robot::shoot_store(3, 2, pass); //shooting 3, storing 2


    pass = false;
	//Tower 2

	//pooping
    Robot::move_to({1100, -400, 90}, {3, 3, 3}); //backout
	Robot::move_to({1100, -400, 250}, {3, 3, 3}); //turning
	Robot::intake({127, 127, 127, -127}); //pooping
	delay(400);
	Robot::intake({0, 0, 127, -127});

	//first ball
	Robot::toggle_outtake(600, 0); //thread delay
	Robot::start_task("OUTTAKE1", Robot::balls_outtake);
	Robot::move_to({450, 1300, 247}, {1, 1, 1}, {1, 1, 2});
	Robot::kill_task("OUTTAKE1");
	Robot::R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE); //braking fly to not poop
	Robot::intake({127, 127, 127, 0}); //intaking first ball
	delay(500);

	//tower
	Robot::move_to({-470, 870, 132}, {1, 1, 1}, {1, 1, 2}); //tower 2
	Robot::intake({0, 0, 0, 0}); //shutting off intakes
	Robot::shoot_store(1, 1, pass); //shooting one and storing one

	pass = false;

	//Tower 3
	//pooping
    Robot::move_to({-380, 960, 132}); //back out
    Robot::intake({0, 0, 127, -127}); //pooping

	//first ball
    Robot::toggle_outtake(600, 0); //thread delay
    Robot::start_task("OUTTAKE2", Robot::balls_outtake); //ratchet
	Robot::move_to({-1140, 1710, 221}, {1, 1, 1}, {1, 1, 2}); //ball point
    Robot::kill_task("OUTTAKE2");
    Robot::intake({127, 127, 127, 0}); //intaking and indexing

	//second ball
    Robot::move_to({-1800, 2930, 219}, {1, 1, 1}, {1, 2, 1}); //ball point
    delay(200);
    Robot::intake({0, 0, 0, 0});

	//tower
    Robot::move_to({-2090, 2370, 178}); //prelim point
    Robot::move_to({-2320, 2320, 178}, {1, 1, 1}, {2, 2, 2}); //tower 3
    Robot::shoot_store(2, 2, pass); //shoot 2 and store 2

    pass = false;
    //Tower 4
	//pooping
    Robot::move_to({-2090, 2290, 180}, {2, 2, 2}, {1, 1, 1}); //backout
    Robot::intake({127, 127, 127, -127}); //pooping
	delay(400);
	Robot::intake({0, 0, 127, -127}); //popping and shutting intakes

	//first ball
    Robot::toggle_outtake(600, 1000);
    Robot::start_task("OUTTAKE3", Robot::balls_outtake); //ratchet
    Robot::move_to({-220, 2980, 338}, {2, 2, 1}, {1, 1, 2}); //ball point
    Robot::kill_task("OUTTAKE3");
    delay(200);
    Robot::intake({127, 127, 127, 0}); //intake
    delay(1000);
	//second ball
    Robot::toggle_outtake(600, 0);
    Robot::start_task("OUTTAKE4", Robot::balls_outtake); //ratchet
    Robot::move_to({300, 3060, 221}); //prelim point
    Robot::kill_task("OUTTAKE4");

    Robot::move_to({-230, 3490, 221}); //ball point
    Robot::intake({127, 127, 127, 0});
    delay(1000);
	//tower
    Robot::move_to({-600, 3930, 221}); //tower 4
    Robot::shoot_store(2, 1, pass); //shoot 2 store 1

    pass = false;

//    tower 5
//	pooping
    Robot::move_to({-565, 3900, 221});
    Robot::intake({0, 0, 127, -127});
    Robot::toggle_outtake(1000, 0);
    Robot::start_task("OUTTAKE5", Robot::balls_outtake);
    delay(1000);

	//wall ball
    Robot::move_to({450, 5030, 221}); //prelim point with intakes open
    Robot::kill_task("OUTTAKE5");
    Robot::move_to({300, 5090, 223}); //ball point
    Robot::intake({127, 127, 127, 0});
    delay(1000);

	//tower
    Robot::move_to({850, 5600, 266}); //prelim point
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({800, 5690, 270}); //tower 5 very inconsistent
    Robot::shoot_store(1, 2, pass); //shoot 1, store 2


//    //Tower 6
//	//pooping
//	Robot::move_to({850, 5600, 266}); //backout
//    Robot::move_to({560, 5380, 394}); //turning
//    Robot::intake({0, 0, 127, -127}); //pooping
//    delay(400);
//    Robot::intake({0, 0, 127, -127});
//
//	//first ball
//    Robot::toggle_outtake(500, 0);
//    Robot::start_task("OUTTAKE5", Robot::balls_outtake); //ratchet
//    Robot::move_to({830, 5230, 399}); //ball point
//    Robot::kill_task("OUTTAKE5");
//    Robot::intake({127, 127, 127, 0});
//    delay(1000);
//    Robot::intake({0, 0, 0, 0});
//
//	//second ball
//    Robot::start_task("OUTTAKE6", Robot::balls_outtake); //ratchet
//    Robot::move_to({1330, 3520, 401}); //ball point
//    Robot::kill_task("OUTTAKE6");
//    Robot::intake({127, 127, 127, 0});
//
//	//tower
//    Robot::move_to({2510, 4090, 311}); //tower 6
//    Robot::shoot_store(2, 1); //shoot 2, store 1
//
//
//    //Tower 7
//	//pooping
//    Robot::intake({0, 0, 127, -127});
//    delay(400);
//    Robot::intake({0, 0, 127, -127});
//	Robot::move_to({2320, 3920, 312}); //backout
//
//	//first ball
//    Robot::start_task("OUTTAKE7", Robot::balls_outtake); //ratchet
//    Robot::move_to({3140, 3300, 401}); //ball point
//    Robot::kill_task("OUTTAKE7");
//    Robot::intake({127, 127, 127, 0});
//
//	//second ball
//    Robot::move_to({3800, 2110, 401}); //ball point
//
//	//tower
//    Robot::move_to({4100, 2700, 359}); //prelim
//    Robot::intake({0, 0, 0, 0});
//    Robot::move_to({4260, 2690, 357}); //tower 7
//    Robot::shoot_store(2, 2); //shoot 2, store 2
//
//
//    //Tower 8
//	//pooping
//    Robot::move_to({4010, 2670, 357}); //backout
//	Robot::intake({127, 127, 127, -127}); //pooping
//    delay(400);
//    Robot::intake({0, 0, 127, -127});
//
//	//first ball
//    Robot::start_task("OUTTAKE8", Robot::balls_outtake); //ratchet
//    Robot::move_to({2810, 1520, 491}); //ball point
//    Robot::kill_task("OUTTAKE8");
//    Robot::intake({127, 127, 127, 0});
//    delay(300);
//    Robot::intake({0, 0, 0, 0});
//
//	//tower
//	Robot::move_to({2480, 1200, 403}); //prelim point
//    Robot::move_to({2610, 1020, 399}); //tower 8
//    Robot::shoot_store(1, 1); //shoot 1, store 1
//

	//all the following points are not accurate and are for format purposes

	// //tower 9
	// //pooping
	// Robot::move_to({2480, 1200, 403}); //backout
	// Robot::intake({127, 127, 127, -127}); //pooping
    // delay(400);
    // Robot::intake({0, 0, 127, -127});

	// //first ball
	// Robot::start_task("OUTTAKE8", Robot::balls_outtake); //ratchet
	// Robot::move_to({2480, 1200, 403}); //turning
	// Robot::kill_task("OUTTAKE8");
    // Robot::move_to({2810, 1520, 491}); //ball point
	// Robot::shoot_store(0, 1); //making sure ball is in bot for the next ratchet

	// //tower
	// Robot::start_task("OUTTAKE8", Robot::balls_outtake); //ratchet
    // Robot::move_to({2810, 1520, 491}); //tower 9
    // Robot::kill_task("OUTTAKE8");
	// Robot::shoot_store(1, 3); //shoot 1, store 3
}