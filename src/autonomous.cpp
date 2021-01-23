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
    Robot::move_to({1100, -400, 90}, {2, 2, 2});
	Robot::move_to({1100, -400, 250}, {2, 2, 2});
	Robot::intake({127, 127, 127, -127});
	delay(400);
	Robot::intake({0, 0, 127, -127});
	Robot::toggle_intaking(true, 600, 0);
	Robot::start_task("OUTTAKE1", Robot::balls_intaking);
	Robot::move_to({400, 1400, 250}, {1, 1, 1}, {1, 1, 2});
	while(!Robot::toggle_intaking(true)){
	    delay(1);
	}
	Robot::kill_task("OUTTAKE1");
	Robot::R2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	Robot::intake({127, 127, 127, 0});
	Robot::move_to({-470, 870, 132}, {1, 1, 1}, {1, 1, 2});
	Robot::intake({0, 0, 0, 0});
	Robot::shoot_store(1, 1);

	//Tower 3
    Robot::move_to({-380, 960, 132});
    Robot::intake({0, 0, 127, -127});
    Robot::toggle_intaking(true, 600, 0);
    Robot::start_task("OUTTAKE2", Robot::balls_intaking);
	Robot::move_to({-1270, 1770, 219});
	Robot::move_to({-1790, 2930, 219}, {1, 1, 1}, {1, 2, 1});
	Robot::kill_task("OUTTAKE2");
    while(Robot::UF.get_value() > 400){
        delay(1);
    }
    delay(200);
    Robot::intake({0, 0, 0, 0});
    Robot::move_to({-730, 1260, 188});
    Robot::move_to({-2340, 2260, 180}, {1, 1, 1}, {2, 2, 2});
}