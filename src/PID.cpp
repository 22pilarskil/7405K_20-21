#include "main.h"
#include "PID.h"

PID::PID(double p, double i, double d){
	kp = p;
	ki = i;
	kd = d;
	prev_error = 0;
	I = 0;
	D = 0;
}
double PID::get_value(double error){
	I += error;
	D = error - prev_error;
	prev_error = error;
	return (kp * error) + (ki * I) + (kd * D);
}
void PID::reset(){
	prev_error = 0;
	I = 0;
	D = 0;
}