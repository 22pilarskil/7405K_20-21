#include "Robot.h"
using namespace pros;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(1);
Motor Robot::FR(2, E_MOTOR_GEARSET_18, true);
Motor Robot::BL(3);
Motor Robot::BR(4, E_MOTOR_GEARSET_18, true);
int Robot::x = 0;
std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;


void Robot::drive(void* x){
	while (true){
		int lefty = master.get_analog(ANALOG_LEFT_Y);
		int leftx = master.get_analog(ANALOG_LEFT_X);
		int right = master.get_analog(ANALOG_RIGHT_X);
		lcd::print(1, "%d", lefty);
		lcd::print(2, "%d", leftx);
		lcd::print(3, "%d", right);
	}
}

void Robot::move_to(int x, int y){
	while(FL.get_position() < x){
		FL.move_velocity(50);
		BL.move_velocity(50);
		BR.move_velocity(50);
		FR.move_velocity(50);
	}
	Robot::brake("coast");
}

void Robot::brake(std::string mode){
	if (mode.compare("coast") == 0){
		FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
		FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
		BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
		BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
	}
	else{
		FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	}
}

void Robot::start_tasks(){
	start_task("DRIVE", drive);
}

void Robot::start_task(std::string name, void (*func)(void*)) {
	if (!task_exists(name)) {
		tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, name.c_str()))));
	}
}

bool Robot::task_exists(std::string name) {
	return tasks.find(name) != tasks.end();
}
