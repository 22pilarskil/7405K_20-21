#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	lcd::initialize();
	delay(100);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
  double scale = 0.8;
  std::vector<std::vector<double>> path{{0, 4000}, {1000, 4000}}; 
  Robot::move_to_pure_pursuit(path, scale);	
}
