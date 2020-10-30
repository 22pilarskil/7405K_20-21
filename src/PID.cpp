#include "main.h"
#include "PID.h"
using namespace pros;
/**
 * Constructor for our PID class/Initializes
 * variables that will be used by class later
 * @param p Sets kp
 * @param i Sets ki
 * @param d Sets kd
 * @param min Sets the Minimum Speed
 * @param counter_ Sets the counter
 */
PID::PID(double p, double i, double d, double min, int counter_) {
	kp = p;
	ki = i;
	kd = d;
	minspeed = min;
	counter = counter_;

	prev_error = 0;
	prev_time = 0;
	I = 0;
	D = 0;
	counter_reset = counter_;
}

/**
 * Calculates speed at which the robot should still be moving at.
 * @param error Error between target position and current position
 * @return
 */
double PID::get_value(double error) {
	int time = millis();
	int delta_time = time - prev_time;
	//Allow for PID to take into account imperfect loop times- delay(5) does not always delay 5 milliseconds

	D = (error - prev_error) / delta_time;
	I += error;

	prev_error = error;
	prev_time = time;
	counter++;

	double speed = (kp * error) + (ki * I) + (kd * D);
	double coefficient = (std::min(100, counter))/100;
	return coefficient *(abs(speed) > minspeed) ? speed : (speed > 0) ? minspeed : -minspeed;
}

/**
 * Resets counter, after each time we use PID
 */
void PID::reset() {
	counter = counter_reset;
}
