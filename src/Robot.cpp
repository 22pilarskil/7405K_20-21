#include "Robot.h"
#include <cmath>
#include <atomic>
using namespace pros;

#define TO_RAD(n) n * M_PI / 180;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(4, true);
Motor Robot::FR(8);
Motor Robot::BL(3);
Motor Robot::BR(9, true);
Motor Robot::IL(6);
Motor Robot::IR(7, true);
ADIEncoder Robot::LE(3, 4);
ADIEncoder Robot::RE(1, 2, true);
ADIEncoder Robot::BE(5, 6); 
Imu Robot::IMU(10);
Acceleration Robot::power_acc(1, 1);
Acceleration Robot::strafe_acc(1, 1);
Acceleration Robot::turn_acc(2.6, 20);
PID Robot::power_PID(.10, 0, 0);
PID Robot::strafe_PID(.16, 0, 0);
PID Robot::turn_PID(.7, 0, 0);


std::atomic<double> Robot::y = 0; 
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::turn_offset_x = 0;
std::atomic<double> Robot::turn_offset_y = 0;
double Robot::offset_back = 4.25;
double Robot::offset_middle = 5.375;
double Robot::wheel_circumference = 2.75 * M_PI;

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

void Robot::drive(void* ptr){
	while (true){

		int power = power_acc.get_curve(master.get_analog(ANALOG_LEFT_Y));
		int strafe = strafe_acc.get_curve(master.get_analog(ANALOG_LEFT_X));
		int turn = turn_acc.get_curve(master.get_analog(ANALOG_RIGHT_X));
		mecanum(power, strafe, turn);

		bool intake = master.get_digital(DIGITAL_L1);
		bool outtake = master.get_digital(DIGITAL_L2);
		double motorpwr = 0;
		if (intake || outtake){
			motorpwr = (intake) ? 127 : -127;
		}
		IL = motorpwr;
		IR = motorpwr;

	}
}

void Robot::fps(void* ptr){
	double last_x = 0;
	double last_y = 0;
	double last_phi = 0;
	while (true){

		double cur_phi = TO_RAD(IMU.get_rotation());
		double dphi = cur_phi - last_phi;
		double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
		double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;
		turn_offset_x = (float)turn_offset_x + cur_turn_offset_x;
		turn_offset_y = (float)turn_offset_y + cur_turn_offset_y;

		double cur_y = (LE.get_value() - turn_offset_y)  + (RE.get_value() + turn_offset_y) / 2;
		double cur_x = BE.get_value() + turn_offset_x;

		double dy = cur_y - last_y;
		double dx = cur_x - last_x;

		last_y = cur_y;
		last_x = cur_x;
		last_phi = cur_phi;

		double global_dy = dy * std::cos(dphi) + dx * std::sin(dphi);
		double global_dx = dx * std::sin(dphi) - dx * std::cos(dphi);

		y = (float)y + global_dy;
		x = (float)x + global_dx;

		lcd::print(5, "Y: %f", (float)y);
		lcd::print(6, "X: %f", (float)x);
		lcd::print(7, "turn_offset: %f", (float)turn_offset_x);

		delay(10);
	}
}

void Robot::mecanum(int power, int strafe, int turn) {
	FL = power + strafe + turn;
	FR = power - strafe - turn;
	BL = power - strafe + turn;
	BR = power + strafe - turn;
	delay(10);
}

void Robot::display(void* ptr){
	while (true){
		master.print(0, 0, "Joystick %d", master.get_analog(ANALOG_LEFT_X));
		lcd::print(1, "Left Encoder: %d", LE.get_value());
		lcd::print(2, "Right Encoder: %d", RE.get_value());
		lcd::print(3, "Back Encoder: %d", BE.get_value());
		lcd::print(4, "IMU value: %f", IMU.get_rotation());
		delay(10);
	}
}

void Robot::move_to(double new_y, double new_x, double heading){
	bool x_fwd = x < new_x;
	bool y_fwd = y < new_y;
	while (!(abs(new_x - float(x)) < 5 && abs(new_y - float(y)) < 5 && abs(heading - IMU.get_rotation()) < 5)){ //while both goals are not reached

		double y_error = new_y - y; //distance between goal_y and current y
		double x_error = - new_x + x; //distance between goal_x and current x
		double imu_error = - (IMU.get_rotation() - heading); //difference between goal heading and current IMU reading

		double power = power_PID.get_value(y_error); 
		double strafe = strafe_PID.get_value(x_error);
		double turn = turn_PID.get_value(imu_error);

		mecanum(power, strafe, turn);
	}
	Robot::brake("hold");
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