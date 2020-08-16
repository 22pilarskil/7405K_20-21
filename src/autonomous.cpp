#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);

  //std::vector<std::vector<double>> path {{.1,.1}, {0, 1000}, {-1000, 1000}, {1000, 1000}, {0, 1000}, {0, 0}};
	//Robot::move_to_pure_pursuit(path, 1);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	delay(300);
	Robot::move_to(500, 0, 0);
	delay(300);
	Robot::move_to(0, 0, 0);
	delay(300);
	Robot::move_to(1000, 0, 0);
  delay(300);
  Robot::move_to(0, 0, 0);
  delay(300);
  Robot::move_to(1500, 0, 0);
  delay(300);
  Robot::move_to(0,0,0);

}
