#include "main.h"
#include "Robot.h"
using namespace pros;

void autonomous() {
	Robot::move_to(1000, 1000, 0);
}
