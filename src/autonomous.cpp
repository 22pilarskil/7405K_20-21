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
    Robot::start_task("SENSORCHECKING", Robot::balls_checking);
    Robot::start_task("RECORD", Robot::record_thread);
    //intake format: index 0,1 is the intakes, index 2 is the indexer, index 3 is the fly
    //every tower after 3 is inconsistent
//    Robot::start_task("SAVEPOINTS", Robot::save_point);
    skills_auton();
}

void match_auton() {
    //Tower 1
    Robot::intake({0, 0, 0, -127});
    delay(500);
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({730, -415, -41});
    Robot::shoot_store(0, 1);
    Robot::move_to({960, -220, -41});
    Robot::shoot_store(1, 0);
    Robot::intake({0, 0, 127, 0});

    //Tower 2
    Robot::move_to({700, -500, -41});
    Robot::intake({0, 0, 0, 0});

    Robot::balls_intake_toggle(1000, 0); 
    Robot::start_task("OUTTAKE0", Robot::balls_intake);
    Robot::move_to({-1380, -490, -87});
    Robot::move_to({-1380, -410, -87});
    Robot::shoot_store(1, 0);
    
    //Tower 3
    Robot::move_to({-1370, -540, -87});
    Robot::balls_intake_toggle(1000, 0); 
    Robot::start_task("OUTTAKE1", Robot::balls_intake);
    Robot::move_to({-3400, -480, -129});
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0});
    Robot::move_to({-3710, -170, -129});
    Robot::shoot_store(1, 0);
}


void skills_auton() {

    Robot::set_pass(false);
    //Tower 1 -------------------------------------------------------------------------------------------------------
    //intaking
    Robot::intake({0, 0, 0, -90});
    delay(100);
    Robot::intake({127, 127, 80, 0}); //turning on intakes for the next two balls
    Robot::move_to_pure_pursuit({{0, 0}, {800, 1}, {1000, 0}}, {1590, -150, 43}, {1.5, 1.5, 1.5}); //pp for first two balls
    Robot::intake({127, 127, 30, 0});
    delay(100);
    Robot::move_to({1075,-580,90}); //tower prelim
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({1128,-858,90}); //tower 1
    Robot::shoot_store(3, 2); //shooting 3, storing 2

    //Tower 2 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::balls_intake_toggle(600, 0);
    Robot::start_task("OUTTAKE0", Robot::balls_intake);
    Robot::move_to({1020, 0, 90}, {2, 2, 2}); //backout
    Robot::move_to({1020, 0, 238}); //turning
    Robot::intake({0, 0, 0, -90});
    Robot::kill_task("OUTTAKE0");
    Robot::intake({0, 0, 127, -127});

    //first ball
    Robot::move_to({550, 1290, 238}, {1, 1, 1}, {1, 1, 2}); //ball point
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0});
    //tower
    Robot::move_to({-490,919,134}, {1, 1, 1}, {1, 1, 2}); //tower 2
    Robot::shoot_store(1, 1); //shoot 1, store 1


    //Tower 3 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::intake({0, 0, 127, -90});
    Robot::move_to({-380, 960, 221}, {1, 1, 1}, {3, 3, 1}); //back out

    //first ball
    Robot::balls_intake_toggle(600, 300, true); //thread delay
    Robot::start_task("OUTTAKE2", Robot::balls_intake);
    Robot::move_to_pure_pursuit({{-380, 960}, {-1190, 1690}, {-1800, 2560}}, {-1903,2851,250});
    delay(100);
    Robot::kill_task("OUTTAKE2");
    //tower
    Robot::move_to({-2120, 2330, 178}); //prelim point
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({-2360, 2330, 178}); //tower 3
    Robot::shoot_store(2, 2); //shoot 2, store 2


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
    Robot::move_to({-215, 2920, 333}, {1, 1, 1}, {1, 1, 3}); //ball point
    Robot::kill_task("OUTTAKE3");
    Robot::shoot_store(0, 1);
    Robot::intake({0, 0, 127, 0});

    //second ball
    Robot::balls_intake_toggle(600, 0); //thread delay
    Robot::start_task("OUTTAKE4", Robot::balls_intake);
    Robot::move_to({-55, 3530, 224}, {1, 1, 1}, {1, 1, 3}); //ball point
    Robot::kill_task("OUTTAKE4");
    Robot::shoot_store(0, 1);
    Robot::intake({127, 127, 127, 0});
    delay(200);
    //tower
    Robot::move_to({-566,4073,220}); //tower 4
    Robot::intake({0, 0, 0, 0});
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
    Robot::move_to({780, 4570, 320}, {1, 1, 1}, {3, 3, 3}); //ball point
    Robot::intake({127, 127, 127, 0});
    delay(400);
    //wall ball
    Robot::move_to({460, 5320, 215}, {1, 1, 1}, {1, 1, 3});
    Robot::kill_task("OUTTAKE5");
    //tower
    Robot::move_to({940,5520,268}); //prelim point
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({890,5828,268}); //tower 5 very inconsistent
    Robot::shoot_store(2, 2); //shoot 1, store 2


    //Tower 6 -------------------------------------------------------------------------------------------------------
    //pooping
    Robot::intake({0, 0, 127, -127});
    int last_ejector = Robot::count();
    Robot::balls_intake_toggle(600, 0);
    Robot::start_task("OUTTAKE7", Robot::balls_intake);
    Robot::move_to({830, 5210, 268}); //backout
    Robot::kill_task("OUTTAKE7");
    Robot::move_to({830, 5350, 435}); //turning
    if (Robot::count() - last_ejector < 2){
        Robot::intake({0, 0, -127, -127});
        delay(200);
        Robot::intake({0, 0, 127, -127});
    }
    //first ball
    Robot::move_to({1480, 3690, 435});
    Robot::intake({127, 127, 127, 0});
    delay(400);
    Robot::move_to({1900, 3440, 313}); //turning
    Robot::intake({0, 0, 127, 0}); //indexing while moving
    //tower
    Robot::move_to({2540, 4100, 313}, {1, 1, 1}); //tower 6
    Robot::shoot_store(1,1); //shoot 2, store 1


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
    Robot::move_to({3750, 2110, 401}, {1, 1, 1}, {1, 1, 1}); //ball point
    //tower
    Robot::move_to({3931,2574,358}); //prelim tower
    Robot::intake({0, 0, 0, 0}); //intake shut off
    Robot::move_to({4351,2596,358}); //tower 7
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
    Robot::move_to({2700,995,401}); //tower 8
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
    Robot::shoot_store(1, 3, false); //shoot 1, store 3
    Robot::balls_intake_toggle(400, 0);
    Robot::start_task("OUTTAKE8", Robot::balls_intake);
    Robot::move_to({1520, 1970, 586});
}

void threaded_auton(void *ptr) {
    skills_auton();
}