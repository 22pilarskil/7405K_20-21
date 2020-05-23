#include "Robot.h"
using namespace pros;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(4);
Motor Robot::FR(8, E_MOTOR_GEARSET_18, true);
Motor Robot::BL(3);
Motor Robot::BR(9, E_MOTOR_GEARSET_18, true);
ADIEncoder Robot::LE(3, 4);
ADIEncoder Robot::RE(1, 2, true);
ADIEncoder Robot::BE(5, 6); 
Imu Robot::IMU(10);
Acceleration Robot::power_acc(1, 1);
Acceleration Robot::strafe_acc(1, 1);
Acceleration Robot::turn_acc(2.6, 20);
PID Robot::power_PID(.14, 0, 0);
PID Robot::strafe_PID(.16, 0, 0);
PID Robot::turn_PID(1.2, 0, 0);


int Robot::y = 0; 
int Robot::x = 0;

float direct_constant = .1;
std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

float imu_last_pos = 0;
bool auto_correct = false;

void Robot::drive(void* x){
	while (true){
		int power = power_acc.get_curve(master.get_analog(ANALOG_LEFT_Y));
		int strafe = strafe_acc.get_curve(master.get_analog(ANALOG_LEFT_X));
		int turn = turn_acc.get_curve(master.get_analog(ANALOG_RIGHT_X));
		lcd::print(1, "Power: %d", power);
		lcd::print(2, "Strafe: %d", strafe);
		lcd::print(3, "Turn: %d", turn);
		mecanum(power, strafe, turn);
		delay(10);
	}
}

void Robot::mecanum(int power, int strafe, int turn) {
	FL = power - strafe - turn;
	FR = power + strafe + turn;
	BL = power - strafe + turn;
	BR = power + strafe - turn;
	delay(10);
}

void Robot::display(void* x){
	while (true){
		master.print(0, 0, "Joystick %d", master.get_analog(ANALOG_LEFT_X));
		lcd::print(1, "Left Encoder: %d", LE.get_value());
		lcd::print(2, "Right Encoder: %d", RE.get_value());
		lcd::print(3, "Back Encoder: %d", BE.get_value());
		lcd::print(4, "IMU value: %f", IMU.get_rotation());
		delay(10);
	}
}

bool Robot::get_error(bool y_fwd, bool x_fwd, int new_y, int new_x){
	if (x_fwd && y_fwd) return x >= new_x && y >= new_y;
	else if (x_fwd && !y_fwd) return x >= new_x && y <= new_y;
	else if (!x_fwd && y_fwd) return x <= new_x && y >= new_y;
	else return x <= new_x && y <= new_y;
}

void Robot::move_to(int new_y, int new_x, int heading){
	bool x_fwd = x < new_x;
	bool y_fwd = y < new_y;
	while (!get_error(x_fwd, y_fwd, new_x, new_y)){ //while both goals are not reached

		double y_error = (new_y - y); //distance between goal_y and current y
		double x_error = - (new_x - x); //distance between goal_x and current x
		double imu_error = - (IMU.get_rotation() - heading); //difference between goal heading and current IMU reading
		double right_left_error = (RE.get_value() - LE.get_value()) / 10; // difference between right and left encoders, scaled down 10x

		double power = power_PID.get_value(y_error); 
		double strafe = strafe_PID.get_value(x_error);
		double turn = turn_PID.get_value(right_left_error + imu_error);

		mecanum(power, strafe, turn);
		y = (RE.get_value() + LE.get_value()) / 2;
		x = BE.get_value();
	}
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
	start_task("DISPLAY", display);
}

void Robot::start_task(std::string name, void (*func)(void*)) {
	if (!task_exists(name)) {
		tasks.insert(std::pair<std::string,std::unique_ptr<pros::Task>>
			(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));	
	}
}

bool Robot::task_exists(std::string name) {
	return tasks.find(name) != tasks.end();
}

void Robot::reset_IMU(){
	IMU.reset();
}
