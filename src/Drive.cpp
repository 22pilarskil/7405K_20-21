#include "lib/PurePursuit.h"
using namespace pros;

#define TO_RAD(n) n * M_PI / 180;

Motor FL(9, true);
Motor FR(12);
Motor BL(1);
Motor BR(19, true);
Motor IL(3);
Motor IR(11, true);
Motor R1(4);
Motor R2(20);

void rollers(int coefficient, bool flip, std::string powered){

	if (coefficient == 0){
		IL = 0;
		IR = 0;
		R1 = 0;
		R2 = 0;
		return;
	}
	if (powered.compare("intakes") == 0 || powered.compare("both") == 0){
		IL = coefficient * 127;
		IR = coefficient * 127;
	}
	if (powered.compare("indexer") == 0 || powered.compare("both") == 0){
		R1 = coefficient * 127;
	   	if (coefficient < 0) coefficient = 0;
	   	R2 = (!flip) ? -coefficient * 127 : coefficient * 127;
	}
}


void mecanum(int power, int strafe, int turn) {

	FL = power + strafe + turn;
	FR = power - strafe - turn;
	BL = power - strafe + turn;
	BR = power + strafe - turn;
	delay(5);
}


void brake(std::string mode){

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


void move_to(double new_y, double new_x, double heading, bool pure_pursuit, double scale, int coefficient, bool flip, std::string powered){

	double y_error = new_y - y;
	double x_error = - (new_x - x);

	double heading2 = (heading < 0) ? heading + 360 : heading -360;
	heading = (abs(phi - heading) < abs(phi - heading2)) ? heading : heading2;
	double imu_error = - (phi - heading);
	/* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current
	heading (i.e. at IMU val 150, travel to 1 deg (|150 - 1| = 149 deg traveled) as opposed to -359 deg
	(|150 - (-359)| = 509 deg traveled) */

	intake(coefficient, flip, powered);

	while (abs(y_error) > 5 || abs(x_error) > 5 || abs(imu_error) > 1){ //while both goals are not reached

		double phi = TO_RAD(phi);
		double power = power_PID.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
		double strafe = strafe_PID.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
		double turn = turn_PID.get_value(imu_error) * 1.5;
		//Apply rotation matrix to errors as they are derived from calculations using rotation matrices

		imu_error = - (phi - heading);
		y_error = new_y - y;
		x_error = - (new_x - x);

		mecanum(power * scale, strafe * scale, turn * scale);

		if (pure_pursuit) {
			return;
		}
	}
	brake("stop");
}


void move_to_pure_pursuit(std::vector<std::vector<double>> points, double scale, int coefficient, bool flip, std::string powered){

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
        	move_to(target[0], target[1], heading, true, scale, coefficient, flip, powered);
        	delay(10);
        	cur = {(float)y, (float)x};
      	}
    }

    double x_error = end[1] - x;
	double y_error = end[0] - y;
	double imu_error = phi - heading;

  	brake("stop");
	lcd::print(6, "DONE");
	lcd::print(7, "YE: %d - XE: %d - IE: %d", int(x_error), int(y_error), int(imu_error));
}