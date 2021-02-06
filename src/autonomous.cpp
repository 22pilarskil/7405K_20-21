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




    bool pass;

    //Tower 1
	//intaking
	Robot::intake({127, 127, 127, 0}); //turning on intakes for the next two balls
	Robot::move_to_pure_pursuit({{0, 0}, {800, 1}, {1000, 0}}, {1650, -200, 45}, {1.5, 1.5, 1,5}); //pp for first two balls
	int last_store_count = Robot::count();
	while(Robot::count() == last_store_count){
		delay(1);
	}
	delay(100);
	Robot::intake({0, 0, 0, 0}); //shutting off intake after wall ball

	//tower
	Robot::move_to({1130, -650, 89}, {2, 2, 2}); //tower prelim
	Robot::move_to({1130, -760, 90}, {1, 1, 1}, {2, 2, 2}); //tower 1
	Robot::shoot_store(3, 2); //shooting 3, storing 2


	pass = false;
	//Tower 2
	//pooping
    Robot::toggle_outtake(300, 0);
    Robot::start_task("OUTTAKE0", Robot::balls_outtake);
    delay(200);
    Robot::move_to({1020, 0, 90}, {3, 3, 3}); //backout
    Robot::kill_task("OUTTAKE0");
	Robot::move_to({1020, 0, 238}, {3, 3, 3}); //turning
	Robot::intake({127, 127, -127, -127});
	delay(100);
	Robot::intake({0, 0, 127, -90});

	//first ball
	Robot::toggle_outtake(600, 0); //thread delay
	Robot::start_task("OUTTAKE1", Robot::balls_outtake);
	Robot::move_to({450, 1300, 238}, {1, 1, 1}, {1, 1, 2}); //ball point
	Robot::kill_task("OUTTAKE1");
	Robot::R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE); //braking fly to not poop
	Robot::shoot_store(0, 1, pass);
    Robot::intake({0, 0, 127, 0});

	//tower
	Robot::move_to({-470, 870, 132}, {1, 1, 1}, {1, 1, 2}); //tower 2
	Robot::intake({0, 0, 0, 0}); //shutting off intakes
	Robot::shoot_store(1, 1, pass); //shoot 1, store 1


	//pass = false;
	//Tower 3
	//pooping
    Robot::move_to({-380, 960, 132}); //back out
    Robot::intake({127, 127, 127, -90}); //pooping
    delay(300);

	//first ball
    Robot::toggle_outtake(600, 0); //thread delay
    Robot::start_task("OUTTAKE2", Robot::balls_outtake); //ratchet
	Robot::move_to({-1140, 1710, 221}, {1, 1, 1}, {1, 1, 2}); //ball point
    Robot::kill_task("OUTTAKE2");
    Robot::intake({127, 127, 127, 0}); //intaking and indexing

	//second ball
    Robot::move_to({-1810, 2800, 221}, {1, 1, 1}, {1, 2, 1}); //ball point
    delay(200);

	//tower
    Robot::move_to({-2090, 2370, 178}); //prelim point
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({-2320, 2280, 180}); //tower 3
    Robot::shoot_store(2, 2, pass); //shoot 2, store 2


    //Tower 4
	//pooping
    Robot::move_to({-1870, 2380, 180}, {1, 1, 1}, {1, 1, 1}); //backout
    Robot::intake({127, 127, 127, -90}); //pooping
	Robot::intake({0, 0, 127, -127}); //popping and shutting intakes
    Robot::move_to({-1870, 2380, 334}, {1, 1, 1}, {1, 1, 1}); //backout
    

	//first ball
    Robot::toggle_outtake(600, 0);
    Robot::start_task("OUTTAKE3", Robot::balls_outtake); //ratchet
    Robot::move_to({-215, 2920, 333}, {1, 1, 1}, {1, 1, 3}); //ball point
    Robot::kill_task("OUTTAKE3");
    Robot::shoot_store(0, 1);
    Robot::intake({127, 127, 127, 0});

	//second ball
    Robot::toggle_outtake(600, 0);
    Robot::start_task("OUTTAKE4", Robot::balls_outtake); //ratchet
    Robot::kill_task("OUTTAKE4");
    Robot::move_to({-266, 3428, 221}, {1, 1, 1}, {1, 1, 3}); //ball point
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0});

	//tower
    Robot::move_to({-620, 3890, 222}); //tower 4
    Robot::intake({0, 0, 0, 0});
    Robot::shoot_store(2, 1, pass); //shoot 2, store 1


    //tower 5
    //pooping
    Robot::move_to({-350, 3640, 222}); //backout
    Robot::intake({127, 127, 127, -127}); //pooping
    delay(200);

	//first ball
    Robot::move_to({-350, 3640, 320}); //turning
    Robot::toggle_outtake(600, 0);
    Robot::start_task("OUTTAKE5", Robot::balls_outtake);
    delay(400); //delay to open intakes before strafing out of tower
    Robot::kill_task("OUTTAKE5");
    Robot::move_to({780, 4570, 320}, {1, 1, 1}, {3, 3, 3}); //ball point
    Robot::intake({127, 127, 127, 0});
    delay(1000);

    //wall ball
    Robot::move_to({300, 5120, 220}, {1,    b. gb   z  1, 1}, {1, 1, 3});
    Robot::intake({127, 127, 127, 0});

	//tower
    Robot::move_to({720, 5390, 272}); //prelim point
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({650, 5800, 272}); //tower 5 very inconsistent
    Robot::shoot_store(2, 2, pass); //shoot 1, store 2


    //Tower 6
	//pooping
	Robot::move_to({830, 5350, 270}); //backout
    Robot::move_to({830, 5350, 435}); //turning
    Robot::toggle_outtake(450, 0);
    Robot::start_task("OUTTAKE6", Robot::balls_outtake);
    Robot::intake({0, 0, 127, -90}); //pooping
    delay(400);
    Robot::intake({0, 0, 127, -127});
    Robot::kill_task("OUTTAKE6");

    //first ball
    Robot::toggle_outtake(600, 0);
    Robot::start_task("OUTTAKE7", Robot::balls_outtake);
    Robot::move_to({1550, 3720, 435}); //second ball
    Robot::kill_task("OUTTAKE7");
    Robot::shoot_store(0, 1, pass); //storing one ball
    Robot::intake({0, 0, 127, 0}); //indexing while moving

    //tower
    Robot::move_to({2440, 4110, 311}, {1, 1, 1}, {2, 2, 2}); //tower 6
    Robot::shoot_store(1,1); //shoot 2, store 1


   //Tower 7
	//pooping
   Robot::move_to({2160, 3860, 312});
   Robot::intake({0, 0, 127, -127}); //pooping

	//first ball
   Robot::toggle_outtake(600, 0);
   Robot::start_task("OUTTAKE7", Robot::balls_outtake); //ratchet
   Robot::move_to({3070, 3270, 401}); //ball point
   Robot::kill_task("OUTTAKE7");
   Robot::intake({127, 127, 127, 0}); //intske and indexer is continuous for next ball

	//second ball
   Robot::move_to({3750, 2110, 401}); //ball point

	//tower
   Robot::move_to({4090, 2660, 358}); //prelim tower
   Robot::intake({0, 0, 0, 0}); //intake shut off
   Robot::move_to({4290, 2750, 358}); //tower 7
   Robot::shoot_store(2, 2); //shoot 2, store 2


    //Tower 8
	//pooping
   Robot::intake({0, 0, 127, -127}); //pooping, no intaking
   Robot::move_to({3900, 2670, 357}); //backout
   Robot::move_to({3900, 2670, 488}); //backout

	//first ball
   Robot::toggle_outtake(600, 0);
   Robot::start_task("OUTTAKE8", Robot::balls_outtake); //ratchet
   Robot::move_to({2860, 1520, 488}); //ball point
   Robot::kill_task("OUTTAKE8");
   Robot::shoot_store(0, 1);
   Robot::intake({0, 0, 127, 0}); //indexer

	//tower
	Robot::move_to({2540, 1160, 402}); //prelim point
    Robot::move_to({2600, 1030, 402}); //tower 8
    Robot::shoot_store(1, 1); //shoot 1, store 1


	 //tower 9
	 //pooping
	Robot::move_to({2540, 1160, 400}); //backout
	 Robot::intake({127, 127, 127, -127}); //pooping
    delay(400);
    Robot::shoot_store(0, 1);

	 //first ball
	 Robot::toggle_outtake(600, 0);
	 Robot::start_task("OUTTAKE8", Robot::balls_outtake); //ratchet
	 Robot::move_to({2480, 1200, 580}); //turning
	 Robot::kill_task("OUTTAKE8");
    Robot::move_to({2160, 1480, 580}); //ball point
	 Robot::shoot_store(0, 1); //making sure ball is in bot for the next ratchet

	 //tower
    Robot::toggle_outtake(600, 0);
	 Robot::start_task("OUTTAKE8", Robot::balls_outtake); //ratchet for middle tower poles
    Robot::move_to({1330, 2126, 580}); //tower 9
    Robot::kill_task("OUTTAKE8");
	 Robot::shoot_store(1, 3); //shoot 1, store 3
}