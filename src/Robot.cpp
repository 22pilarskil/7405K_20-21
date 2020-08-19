#include "Robot.h"
#include "PurePursuit.h"
#include <cmath>
#include <atomic>
using namespace pros;

#define TO_RAD(n) n * M_PI / 180;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(9, true);
Motor Robot::FR(12);
Motor Robot::BL(1);
Motor Robot::BR(19, true);
Motor Robot::IL(3, true);
Motor Robot::IR(11);
Motor Robot::R1(4, true);
Motor Robot::R2(10);
ADIEncoder Robot::LE(3, 4);
ADIEncoder Robot::RE(7, 8, true);
ADIEncoder Robot::BE(5, 6);
Imu Robot::IMU(5);
Vision Robot::vision(21);
ADIAnalogIn Robot::LT1 (1);
ADIAnalogIn Robot::LT2 (2);
PID Robot::power_PID(.3, 0, .5, 10);
PID Robot::strafe_PID(.52, 0, 0, 19);
PID Robot::turn_PID(1.3, 0, 0, 16);


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


//L2: Blue Ball-1860 Red Ball-743
//L1: Blue Ball-490 Red Ball-2511


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
		delay(100);
	}
}

void Robot::lin_intake(int balls){
	int line2;
	int line1;
	while(line2 != 1 && line1 != 2){
		if(LT1.get_value())
	}
}



void Robot::reset_PID(){

	power_PID.reset();
	strafe_PID.reset();
	turn_PID.reset();
}


void Robot::drive(void* ptr){

	int fcd_toggle;

  	while (true){
		
		int power_dz = 110;
	  	int power_dz1 = 30;
	  	int power_dz2 = 100;

	  	int strafe_dz = 20;
	  	int strafe_dz1 = 60;

		int power = master.get_analog(ANALOG_LEFT_Y);
		int strafe = master.get_analog(ANALOG_LEFT_X);
		int turn = master.get_analog(ANALOG_RIGHT_X);


		if (abs(strafe) > strafe_dz1 && abs(power) > power_dz2) power = 0;
	  	if (abs(power) > power_dz && abs(strafe) > strafe_dz) strafe = 0;
	  	if (abs(power) < power_dz1) power = 0;

	  	/*
    	if (fcd_toggle % 2 == 1){
      		double theta = TO_RAD(IMU.get_rotation());
      		int divider = 360 / (round(IMU.get_rotation() / 10) * 10);
      		power = power * cos(theta) - strafe * sin(theta);
      		if(power > 0 && 360 / divider == 4) strafe = -power; 
     		else if(power < 0 && 360 / divider == 4) strafe = power; 
      		else if(power > 0 && 360 / divider == -4) strafe = power;
      		else if(power < 0 && 360 / divider == -4) strafe = -power;
      		else{strafe = power * sin(theta) + strafe * cos(theta);
   		}
    	if (master.get_digital(DIGITAL_DOWN)) fcd_toggle ++;
    	*/

    	if (master.get_digital(DIGITAL_LEFT)) move_to(0, 0, int(IMU.get_rotation()/360)*360);
		mecanum(power, strafe, turn);

		bool inttake = master.get_digital(DIGITAL_R2);
		bool outtake = master.get_digital(DIGITAL_X);

		bool just_intake = master.get_digital(DIGITAL_R1);
		bool just_indexer = master.get_digital(DIGITAL_L2);

		bool flip = master.get_digital(DIGITAL_L1);

		double motorpwr = 0;

		if (inttake || outtake){
			motorpwr = (inttake) ? 1 : -1;
		} 

		if(just_intake){
			IL = 127;
			IR = 127;
		} else if (just_indexer){
			R1 =   127;
		   	R2 = (!flip) ?  -127 : 127; 
		}
		else {
			intake(motorpwr, flip, true);
		}
	}
}


void Robot::intake(int coefficient, bool flip, bool rollers){
 	IL = coefficient * 127;
	IR = coefficient * 127;
	if(rollers){
	   	if (coefficient < 0) coefficient = 0;
		R1 = coefficient * 127;
	   	R2 = (!flip) ? -coefficient * 127 : coefficient * 127; 
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
	delay(5);
}


void Robot::display(void* ptr){

	while (true){
		master.print(0, 0, "Joystick %d", master.get_analog(ANALOG_LEFT_X));
		lcd::print(1, "LE: %d - RE: %d", LE.get_value(), RE.get_value());
		lcd::print(2, "Back Encoder: %d", BE.get_value());
		lcd::print(3, "IMU value: %f", IMU.get_rotation());
		lcd::print(4, "Line1 Value: %d", LT1.get_value());
		lcd::print(5, "Line2 Value: %d", LT2.get_value());

		delay(10);
	}
}


void Robot::move_to(double new_y, double new_x, double heading, int intakes, bool rollers, bool pure_pursuit, double scale){

	double y_error = new_y - y;
	double x_error = - (new_x - x);

	double heading2 = (heading < 0) ? heading + 360 : heading -360;
	heading = (abs(IMU.get_rotation() - heading) < abs(IMU.get_rotation() - heading2)) ? heading : heading2;
	double imu_error = - (IMU.get_rotation() - heading);
	/* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current 
	heading (i.e. at IMU val 150, travel to 1 deg (|150 - 1| = 149 deg traveled) as opposed to -359 deg
	(|150 - (-359)| = 509 deg traveled) */

	Robot::intake(intakes,1,rollers);
	reset_PID();

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

		if (pure_pursuit) {
			reset_PID();
			return;
		}
	}
	Robot::intake(0,1,rollers);
	brake("stop");
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

      	while (distance(cur, end) > radius){

      		lcd::print(7, "%f, %d", distance(cur, end), index);

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

void Robot::reset_LineTrackers(){
	LT1.calibrate();
	LT2.calibrate();
}