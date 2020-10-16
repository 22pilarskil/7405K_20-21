#include "main.h"

class PID{
	public:
		double kp;
		double ki;
		double kd;
		double minspeed;
		int counter;

		double prev_error;
		int prev_time;
		double I;
		double D;
		int counter_reset;

		PID(double p, double i, double d, double min = 0, int counter_ = 100);
		double get_value(double error);
		void reset();
};