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

	Robot::move_to({500, 0, 0});
	Robot::move_to({0, 0, 0});
	Robot::move_to({1000, 0, 0});
	Robot::move_to({0, 0, 0});

	// Robot::intake(1, "both");
	// delay(600);
	// Robot::intake(0);
	// Robot::move_to({560, 0, 0});
	// Robot::move_to({840, 340, -135});
	// Robot::move_to({250, 840, -135});
	// Robot::quickscore();
	// Robot::move_to({600, 525, -135});
	// Robot::move_to({600, 525, 0});
	// Robot::intake(-1, "intakes");
	// Robot::move_to({2200, 530, 0});
	// Robot::intake(1, "both");
	// delay(800);
	// Robot::intake(0);
	// Robot::move_to({2600, 470, 0});
	// Robot::move_to({2600, 470, -90});
	// Robot::move_to({2600, 750, -90});
	// Robot::quickscore();

}
