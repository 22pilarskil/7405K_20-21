#include "main.h"
#include "PID.h"
using namespace pros;

PID::PID(double p, double i, double d, double min){
	kp = p;
	ki = i;
	kd = d;
	minspeed = min;

	prev_error = 0;
	prev_time = 0;
	I = 0;
	D = 0;
}


double PID::get_value(double error){
	
	int time = millis();
	int delta_time = time - prev_time;
	//Allow for PID to take into account imperfect loop times- delay(5) does not always delay 5 milliseconds

	D = (error - prev_error) / delta_time;
	I += error;

	prev_error = error;
	prev_time = time;
	
	double speed = (kp * error) + (ki * I) + (kd * D);
	return (abs(speed) > minspeed) ? speed : (speed > 0) ? minspeed : -minspeed;
}


void PID::reset(){
	prev_error = 0;
	prev_time = millis();
	I = 0;
	D = 0;
}