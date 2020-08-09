#include "Robot.h"
#include "PurePursuit.h"
#include <cmath>
#include <atomic>
using namespace pros;

#define TO_RAD(n) n * M_PI / 180;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(20, true);
Motor Robot::FR(18);
Motor Robot::BL(13);
Motor Robot::BR(11, true);
Motor Robot::IL(2);
Motor Robot::IR(15, true);
Motor Robot::R1(8, true);
Motor Robot::R2(10);
ADIEncoder Robot::LE(3, 4);
ADIEncoder Robot::RE(1, 2, true);
ADIEncoder Robot::BE(5, 6);
Imu Robot::IMU(10);
Vision Robot::vision(16);
Acceleration Robot::power_acc(1, 1);
Acceleration Robot::strafe_acc(1, 1);
Acceleration Robot::turn_acc(2.6, 20);
PID Robot::power_PID(.2, 0, 1.3, 8);
PID Robot::strafe_PID(.26, 0, 1.3, 17);
PID Robot::turn_PID(.64, 0, 0, 0);


std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::turn_offset_x = 0;
std::atomic<double> Robot::turn_offset_y = 0;
double Robot::offset_back = 4 + 5/16;
double Robot::offset_middle = 5 + 7/16;
double Robot::wheel_circumference = 2.75 * M_PI;
bool flip = true;
int radius = 300;
std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;


void Robot::vis_sense(void* ptr){
	vision_signature_s_t red_signature = Vision::signature_from_utility(1, -669, 5305, 2318, -971, 571, -200, 0.700, 0);
	vision_signature_s_t blue_signature = Vision::signature_from_utility(2, -3277, -2313, -2796, 9039, 13285, 11162, 3.000, 0);
	vision.set_signature(1, &red_signature);
	vision.set_signature(2, &blue_signature);
	while (true){
		vision_object_s_t red_ball = vision.get_by_sig(0, 1);
		vision_object_s_t blue_ball = vision.get_by_sig(0, 2);
		int red_x_coord = red_ball.x_middle_coord;
		int blue_x_coord = blue_ball.x_middle_coord;
		lcd::print(6, "Relative: %d - Absolute: %d", 
			(red_x_coord > 0 && red_x_coord < 316 && red_ball.signature != 255) ? int((316/2 - red_x_coord)/10) : 0, red_ball.signature);
		lcd::print(7, "Relative: %d - Absolute: %d", 
			(blue_x_coord > 0 && blue_x_coord < 316 && blue_ball.signature != 255) ? int((316/2 - blue_x_coord)/10) : 0, blue_ball.signature);
		delay(500);
	}
 }


void Robot::drive(void* ptr){
	int fcd_toggle;
  while (true){
		
		int power_dz = 110;
	  int power_dz1 = 30;
	  int power_dz2 = 100;

	  int strafe_dz = 20;
	  int strafe_dz1 = 70;

		int power = power_acc.get_curve(master.get_analog(ANALOG_LEFT_Y));
		int strafe = strafe_acc.get_curve(master.get_analog(ANALOG_LEFT_X));
		int turn = turn_acc.get_curve(master.get_analog(ANALOG_RIGHT_X));

		if (abs(strafe) > strafe_dz1 && abs(power) > power_dz2) power=0;
	  if (abs(power) > power_dz && abs(strafe) > strafe_dz) strafe=0;
	  if (abs(power) < power_dz1) power=0;

    if (fcd_toggle%2 == 1){
      double theta = TO_RAD(IMU.get_rotation());
      power = power*cos(theta) - strafe*sin(theta);
      strafe = power*sin(theta) + strafe*cos(theta); 
    }
    if (master.get_digital(DIGITAL_DOWN)) fcd_toggle++;

		mecanum(power, strafe, turn);

		bool inttake = master.get_digital(DIGITAL_L1);
		bool outtake = master.get_digital(DIGITAL_L2);

		bool flip = master.get_digital(DIGITAL_R1);

		bool fps = master.get_digital(DIGITAL_R2);
		if (fps){
			move_to(0, 0, IMU.get_rotation() - (int(IMU.get_rotation()) % 360));
		}
		double motorpwr = 0;

		if (inttake || outtake){
			motorpwr = (inttake) ? 1 : -1;
		}
		intake(motorpwr, flip);
	}
}


void Robot::intake(int coefficient, bool flip){
	IL = coefficient * 127;
	IR = coefficient * 127;
	R1 = coefficient * 127;
	if (coefficient < 0){
		coefficient = 0;
	}
	R2 = (!flip) ? coefficient * 127 : -coefficient * 127;
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
		/* Calculate how much the encoders have turned as a result of turning ONLY in order to 
		isolate readings representing lateral or axial movement from readings representing 
		turning in place */

		turn_offset_x = (float)turn_offset_x + cur_turn_offset_x;
		turn_offset_y = (float)turn_offset_y + cur_turn_offset_y;

		double cur_y = ((LE.get_value() - turn_offset_y)  + (RE.get_value() + turn_offset_y)) / 2;
		double cur_x = BE.get_value() - turn_offset_x;

		double dy = cur_y - last_y;
		double dx = cur_x - last_x;

		double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
		double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);
		//Apply rotation matrix to dx and dy to calculate dx and dy on the phi = 0 orientation

		y = (float)y + global_dy;
		x = (float)x + global_dx;

		lcd::print(4, "Offset: %d - Y: %f", int(turn_offset_y), (float)y);
		lcd::print(5, "Offset: %d - X: %f", int(turn_offset_x), (float)x);

		last_y = cur_y;
		last_x = cur_x;
		last_phi = cur_phi;
		delay(5);
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
		lcd::print(1, "LE: %d - RE: %d", LE.get_value(), RE.get_value());
		lcd::print(2, "Back Encoder: %d", BE.get_value());
		lcd::print(3, "IMU value: %f", IMU.get_rotation());
		delay(10);
	}
}

void Robot::move_to(double new_y, double new_x, double heading, bool pure_pursuit, double scale){
	double y_error = new_y - y;
	double x_error = - (new_x - x);

	double heading2 = (heading < 0) ? heading + 360 : heading -360;
	heading = (abs(IMU.get_rotation() - heading) < abs(IMU.get_rotation() - heading2)) ? heading : heading2;
	double imu_error = - (IMU.get_rotation() - heading);
	/* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current 
	heading (i.e. at IMU val 150, travel to 1 deg (|150 - 1| = 149 deg traveled) as opposed to -359 deg
	(|150 - (-359)| = 509 deg traveled) */

	while (abs(y_error) > 5 || abs(x_error) > 5 || abs(imu_error) > 1){ //while both goals are not reached

		double phi = TO_RAD(IMU.get_rotation());
		double power = power_PID.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
		double strafe = strafe_PID.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
		double turn = turn_PID.get_value(imu_error);
		//Apply rotation matrix to errors as they are derived from calculations using rotation matrices

		imu_error = - (IMU.get_rotation() - heading); 
		y_error = new_y - y;
		x_error = - (new_x - x); 

		mecanum(power * scale, strafe * scale, turn * scale);

		if (pure_pursuit) return;
	}
	Robot::brake("stop");
	//lcd::print(6, "DONE");
	//lcd::print(7, "YE: %d - XE: %d - IE: %d", int(y_error), int(x_error), int(imu_error));
}


void Robot::move_to_pure_pursuit(std::vector<std::vector<double>> points, double scale){

	std::vector<double> end;
	std::vector<double> start;
	std::vector<double> target;
	std::vector<double> cur {(float)y, (float)x};
	double heading;

  	for (int index = 0; index < points.size() - 1; index++) {

      	start = points[index];
      	end = points[index + 1];
      	//lcd::print(6, "%f", distance(cur, end));
      	//lcd::print(7, "{%f, %f}, {%f, %f}", (float)cur[0], (float)cur[1], end[0], end[1]);


      	while (distance(cur, end) > radius){

      		lcd::print(7, "%f, %d", distance(cur, end), index);
      		//lcd::print(6, "{%f, %f}", end[0], end[1]);

        	target = get_intersection(start, end, cur, radius, scale); 
        	heading = get_degrees(target, cur);

        	lcd::print(6, "{%f, %f} %f", target[0], target[1], heading);
        	Robot::move_to(target[0], target[1], heading, true);
        	delay(10);
        	cur = {(float)y, (float)x};
      	}
    }

    double x_error = end[0] - x;
	double y_error = end[1] - y;
	double imu_error = IMU.get_rotation() - heading;

  	Robot::brake("stop");
	lcd::print(6, "DONE");
	lcd::print(7, "YE: %d - XE: %d - IE: %d", int(x_error), int(y_error), int(imu_error));
}


void Robot::brake(std::string mode){
	if (mode.compare("coast") == 0){
		FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
		FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
		BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
		BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
	}
	else if (mode.compare("hold") == 0){
		FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	}
	else {
		FL = 0;
		FR = 0;
		BL = 0;
		BR = 0;
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
