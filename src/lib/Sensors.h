#include "main.h"
#include <memory>
#include <string>
#include <vector>
using namespace pros;


class Sensors {
	public:
		static Imu IMU;
		static Vision vision;
		static ADIAnalogIn LM1;
		static ADIAnalogIn LT2;
		static ADIEncoder LE;
		static ADIEncoder RE; 
		static ADIEncoder BE;

		static std::atomic<int> balls_ejected;
		static std::atomic<int> balls_intook;

		static int balls_ejected_count();
		static int balls_intook_count();

		static void reset_sensors();
		static void sensors(void* ptr);
		static void vis_sense(void* ptr);
};