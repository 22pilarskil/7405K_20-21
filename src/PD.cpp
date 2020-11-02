#include "main.h"
#include "PD.h"
using namespace pros;
/**
 * @descL Constructor for our PID class/Initializes
 * variables that will be used by class later
 * @param p: Sets kp, the coefficient of our error term
 * @param d: Sets kd, the coefficient of our derivative term
 * @param min: Sets the minimum speed, designed to prevent robot from grinding to a halt
 * @param counter_ Sets the counter
 */
PD::PD(double p, double d, double min, int counter_) {
	kp = p;
	kd = d;
	minspeed = min;
	counter = counter_;

	prev_error = 0;
	prev_time = 0;
	D = 0;
	counter_reset = counter_;
}

/**
 * Calculates speed at which the robot should still be moving at.
 * @param error: Error between target position and current position (calculated as difference between target and current
 	position vectors in function PID::get_value is being called in)
 * @return: Speed of motors associated with PID object
 */
double PD::get_value(double error) {
	int time = millis();
	int delta_time = time - prev_time;
	//Allow for PID to take into account imperfect loop times- delay(5) does not always delay 5 milliseconds

	derivate_of_error = (error - prev_error) / delta_time;

	prev_error = error;
	prev_time = time;
	counter++;

	double speed = (kp * error) + (kd * derivative_of_error);
	double coefficient = (std::min(100, counter))/100;
	return coefficient *(abs(speed) > minspeed) ? speed : (speed > 0) ? minspeed : -minspeed;
}

/**
 * @desc: Resets counter back to counter_reset after each time we use PID in order to reset acceleration curve 
 */
void PID::reset() {
	counter = counter_reset;
}
