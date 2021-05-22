#include "main.h"
#include "Robot.h"
using namespace pros;

/* Below is our programming skills path in its entirety. It takes advantage of the Robot::move_to, Robot::move_to_pure_pursuit,
and Robot::store functions to move to preset positions on the field and intake balls that we know to be there. It is
important to note that while in our programming skills video (https://www.youtube.com/watch?v=_H-iJ-kX9H8) it appears
that our robot is detecting and chasing down balls on the field, this is set the case. Since the field is set up the 
same way every time, we simply use odometry to find the points on the field where balls are located, and initiate a
single, uniform sequence each run. In effect, if the field is set up incorrectly (say a ball is shifted to the left 
or right by a significant margin) the odometry points will not work, and our skills path will fail. */

void autonomous() {
    Robot::start_task("FPS", Robot::fps);
    Robot::start_task("DISPLAY", Robot::display);
    Robot::start_task("SENSING", Robot::sensing);
    Robot::start_task("RECORD", Robot::record_thread);
    Robot::start_task("TIMING", Robot::time_run);
    //intake format: index 0,1 is the intakes, index 2 is the indexer, index 3 is the fly
    //every tower after 3 is inconsistent
//    Robot::start_task("SAVEPOINTS", Robot::save_point);
    red_auton();
}

void blue_auton() {




    //Tower 1
    Robot::intake({127, 127, 127, 0});
    Robot::move_to({1610, 630, 14}, .1, false, 0, {3, 1.5, 1}); //push ball into tower one

    Robot::intake({127, 127, 127, 0});

    Robot::move_to({1740, -1320, 61}, .1, false, 0, {.8, 1, 4}); //line ball
    Robot::intake({0, 0, 0, 0});

    Robot::move_to({970, -1790, 78}); //tower ball push
    Robot::move_to({415, -1310, 78}, .1, false, 0, {2, 1, 1});

    Robot::intake({127, 127, 50, 0});
    Robot::move_to({445, -1600, 78});
    Robot::intake({0, 0, 0, 0});

    Robot::move_to({200, -2190, 22}, .1, false, 0, {1, 3, 1}); //tower point

    Robot::shoot_store(2, 0);
    

    Robot::move_to({-990, -2290, 63});
    Robot::intake({127, 127, 30, 0});
    Robot::move_to({-870, -2630, 63});
    Robot::intake({0, 0, 50, 0});
    
    // Robot::move_to({-740, -2460, 84}, .1, false, 0, {0.5, 3, 5});


    Robot::move_to({-2380, -1400, 195}, 0.1, false, 0, {0.8, 2, 4});
    Robot::shoot_store(1, 0);

    Robot::end_timer();
}


void red_auton() {

    //Tower 1
    Robot::intake({127, 127, 127, 0});
    Robot::move_to({1580, 650, 24}, .1, false, 0, {3, 1.5, 1}); //push ball into tower one

    Robot::intake({127, 127, 127, 0});

    Robot::move_to({1740, -1320, 61}, .1, false, 0, {.8, 1, 4}); //line ball
    delay(100);
    Robot::intake({0, 0, 0, 0});

    Robot::move_to({970, -1790, 78}); //tower ball push
    Robot::move_to({415, -1310, 78}, .1, false, 0, {2, 1, 1});

    Robot::intake({127, 127, 60, 0});
    Robot::move_to({445, -1600, 78});
    Robot::intake({0, 0, 0, 0});

    Robot::move_to({210, -2150, 21}, .1, false, 0, {1, 3, 1}); //tower point

    // Robot::shoot_store(1, 0);
    Robot::intake({0, 0, 0, -127});
    delay(100);
    Robot::intake({0, 0, 0, 127});
    delay(1900);
    Robot::intake({0, 0, 0, 0});

    Robot::intake({0, 0, 35, 0});


    Robot::move_to({-2410, -1480, 196}, 0.1, false, 0, {1, 1, 2});
    Robot::shoot_store(1, 0);
}



void skills_auton() {

    Robot::start_task("TIMER", Robot::time_run);
    Robot::set_pass(false);
    //Tower 1 -------------------------------------------------------------------------------------------------------
    //intaking
    Robot::intake({0, 0, 0, -90});
    delay(100);
    Robot::intake({127, 127, 60, 0}); //turning on intakes for the next two balls
    Robot::move_to({620,43,-9}); //CHANGECHANGECHANGE
    Robot::intake({127, 127, 60, 0});
    delay(100); 
    Robot::move_to({1109,-338,88}); //tower prelim
    Robot::intake({0, 0, 20, 0});
    Robot::move_to({1131,-801,91}, true); //tower 1
    Robot::shoot_store(3, 2); //shooting 3, storing 2

    //Tower 2 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::balls_intake_toggle(300, 0);
    Robot::start_task("OUTTAKE0", Robot::balls_intake);
    Robot::move_to({972,-275,90}); //backout
    Robot::intake({0, 0, -60, 0});
    delay(1000);
    Robot::move_to({1328,57,44});
    Robot::intake({127, 127, 60, 0});
    //TURN    Robot::move_to()
    Robot::move_to({1647,-220,46}); //BALLLLLLLL
    Robot::move_to({702,1300,-117});
    Robot::intake({0, 0, 0, -90});
    Robot::kill_task("OUTTAKE0");
    Robot::intake({0, 0, 127, -127});

    //first ball
    Robot::move_to({614,1381,239}); //ball point
    Robot::intake({0, 0, 0, 0});
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0});
    //tower
    Robot::move_to({-421,920,135}, true); //tower 2
    Robot::shoot_store(2, 1); //shoot 1, store 1

    delay(10000000);

    //Tower 3 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::intake({0, 0, 127, -90});
    Robot::move_to({-380, 960, 221}); //back out

    //first ball
    Robot::balls_intake_toggle(600, 300, true); //thread delay
    Robot::start_task("OUTTAKE2", Robot::balls_intake);
    Robot::move_to_pure_pursuit({{-380, 960}, {-1190, 1690}, {-1800, 2560}}, {-1640,3040,222});
    delay(100);
    Robot::kill_task("OUTTAKE2");
    //tower
    Robot::move_to({-1990, 2360, 180}); //prelim point
    Robot::move_to({-2260,2360,180}, true); //tower 3
    Robot::shoot_store(2, 2); //shoot 2, store 3


    //Tower 4 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::intake({0, 0, 127, -127});
    Robot::balls_intake_toggle(600, 0);
    Robot::start_task("OUTTAKE3", Robot::balls_intake);
    Robot::move_to({-1870, 2380, 200}); //backout
    Robot::move_to({-1870, 2380, 334}); //turning
    Robot::intake({0, 0, -127, -127});
    delay(200);
    Robot::intake({0, 0, 127, -127});

    //first ball
    Robot::move_to({-200, 3000, 333}); //ball point
    Robot::kill_task("OUTTAKE3");
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0});

    //second ball
    Robot::balls_intake_toggle(600, 0); //thread delay
    Robot::start_task("OUTTAKE4", Robot::balls_intake);
    Robot::move_to({-55, 3530, 224}); //ball point
    Robot::kill_task("OUTTAKE4");
    Robot::shoot_store(0, 1);
    Robot::intake({127, 127, 127, 0});
    delay(200);
    Robot::intake({0, 0, 127, 0});
    //tower
    Robot::move_to({-560,4000,225}, true); //tower 4
    Robot::shoot_store(2, 1); //shoot 2, store 1

    //Tower 5 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::balls_intake_toggle(600, 0);
    Robot::intake({0, 0, 127, -127});
    Robot::start_task("OUTTAKE5", Robot::balls_intake);
    Robot::move_to({-350, 3640, 222}); //backout
    //first ball
    Robot::move_to({-350, 3640, 320}); //turning
    Robot::R2.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    Robot::move_to({780, 4570, 320}); //ball point
    Robot::intake({127, 127, 127, 0});
    delay(400);
    //wall ball
    Robot::move_to({460, 5320, 215});
    Robot::kill_task("OUTTAKE5");
    //tower
    Robot::move_to({840, 5430, 269}); //prelim point
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({820, 5770, 269}, true); //tower 5 very inconsistent
    Robot::shoot_store(2, 2); //shoot 1, store 2


    //Tower 6 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::intake({0, 0, 127, -90});
//    int last_ejector = Robot::count();
    Robot::balls_intake_toggle(400, 0);
    Robot::start_task("OUTTAKE7", Robot::balls_intake);
    Robot::move_to({830, 5210, 268}); //backout
    Robot::kill_task("OUTTAKE7");
    Robot::move_to({830, 5350, 435}); //turning
//    if (Robot::count() - last_ejector < 2){
//        Robot::intake({0, 0, -127, -127});
//        delay(200);
//        Robot::intake({0, 0, 127, -127});
//    }

    //first ball
    Robot::move_to({1570, 3670, 435});
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0}); //indexing while moving

    //tower
    Robot::move_to({2540, 4100, 313}, true); //tower 6
    Robot::shoot_store(1, 1); //shoot 2, store 1


    Robot::set_pass(false);
    //Tower 7 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::intake({0, 0, 127, -127});
    delay(200);
    Robot::balls_intake_toggle(600, 0);
    Robot::start_task("OUTTAKE7", Robot::balls_intake);
    Robot::move_to({2160, 3860, 312});
    Robot::intake({0, 0, 127, -127}); //pooping
    //first ball
    Robot::move_to({3070, 3270, 401}); //ball point
    Robot::kill_task("OUTTAKE7");
    Robot::R2.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    Robot::intake({127, 127, 127, 0}); //intske and indexer is continuous for next ball
    //second ball
    Robot::move_to({3750, 2110, 401}); //ball point
    //tower
    Robot::move_to({3931,2574,358}); //prelim tower
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({4351,2596,358}, true); //tower 7
    Robot::shoot_store(2, 2); //shoot 2, store 2


    //Tower 8 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::intake({0, 0, 127, -100}); //pooping, no intaking
    Robot::balls_intake_toggle(600, 0);
    Robot::start_task("OUTTAKE8", Robot::balls_intake); //ratchet
    Robot::move_to({3900, 2670, 357}); //backout
    Robot::move_to({3900, 2670, 488}); //backout
    Robot::kill_task("OUTTAKE8");
    //first ball
    Robot::intake({0, 0, 0, -127});
    Robot::move_to({2860, 1520, 488}); //ball point
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0}); //indexer
    //tower
    Robot::move_to({2500, 1140, 400}); //prelim point
    Robot::move_to({2700,995,401}, true); //tower 8
    Robot::shoot_store(1, 1); //shoot 1, store 1


    //Tower 9 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::move_to({2540, 1160, 400}); //backout
    Robot::intake({127, 127, 127, -127}); //pooping
    delay(400);
    //first ball
    Robot::balls_intake_toggle(600, 0);
    Robot::start_task("OUTTAKE8", Robot::balls_intake); //ratchet
    Robot::move_to({2480, 1200, 578}); //turning
    Robot::kill_task("OUTTAKE8");
    Robot::move_to({2110, 1530, 578}); //ball point
    Robot::shoot_store(0, 1); //making sure ball is in bot for the next ratchet
    //tower
    Robot::balls_intake_toggle(600, 0);
    Robot::start_task("OUTTAKE8", Robot::balls_intake); //ratchet for middle tower poles
    Robot::move_to({1420, 2130, 580}); //tower 9
    Robot::kill_task("OUTTAKE8");
    Robot::shoot_store(1, 3); //shoot 1, store 3
    Robot::balls_intake_toggle(400, 0);
    Robot::start_task("OUTTAKE8", Robot::balls_intake);
    Robot::move_to({1520, 1970, 586}, true);
    Robot::end_timer();
}

void threaded_auton(void *ptr) {
    skills_auton();
}