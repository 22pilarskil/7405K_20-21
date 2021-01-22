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

	while(Robot::UF.get_value() > 400){
		Robot::intake({127, 127, 0, 0});
	}
	Robot::intake({0, 0, 127, -127});
	Robot::toggle_intaking(true, 500);
	Robot::start_task("OUTTAKE", Robot::balls_intaking);
	Robot::move_to({400, 1400, 250});




}