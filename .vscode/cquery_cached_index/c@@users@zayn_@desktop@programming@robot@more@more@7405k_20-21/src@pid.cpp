#include "main.h"
#include "PID.h"

PID::PID(double p, double i, double d, double min){
	kp = p;
	ki = i;
	kd = d;
	prev_error = 0;
	I = 0;
	D = 0;
	minspeed = min;

}
double PID::get_value(double error){
	I += error;
	D = error - prev_error;
	prev_error = error;
	double speed = (kp * error) + (ki * I) + (kd * D);
	return (abs(speed) > minspeed) ? speed : (speed > 0) ? minspeed : -minspeed;
}
void PID::reset(){
	prev_error = 0;
	I = 0;
	D = 0;
}