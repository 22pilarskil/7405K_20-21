#include "main.h"
#include "Sensors.h"
#include <memory>
#include <string>
#include <vector>
#include <atomic>
using namespace pros;

Imu Sensors::IMU(5);
Vision Sensors::vision(21);
ADIAnalogIn Sensors::LM1 (2);
ADIAnalogIn Sensors::LT2 (1);

ADIEncoder Sensors::LE(3, 4);
ADIEncoder Sensors::RE(7, 8, true);
ADIEncoder Sensors::BE(5, 6);

std::atomic<int> Sensors::balls_ejected = 0;
std::atomic<int> Sensors::balls_intook = 0;

double LT2_average = 0;


int Sensors::balls_ejected_count(){
	return balls_ejected;
}

int Sensors::balls_intook_count(){
	return balls_intook;
}

void Sensors::sensors(void* ptr){
	bool just_checked_ejected = true;
	bool just_checked_intook = true;
	while(true){
		if (just_checked_ejected && LM1.get_value()){
			just_checked_ejected = false;
		}
		else if (!LM1.get_value() && !just_checked_ejected){
			just_checked_ejected = true;
			balls_ejected = int(balls_ejected) + 1;
		}
		if (just_checked_intook && LT2.get_value() < 2000){
			just_checked_intook = false;
			balls_intook = int(balls_intook) + 1;
		}
		else if (LT2.get_value() > 2000 && !just_checked_intook){
			just_checked_intook = true;
		}
		lcd::print(7, "%d", int(balls_intook));
		delay(5);
	}
}


void Sensors::reset_sensors(){
	IMU.reset();
	double num_iter = 50;
	double LT2_total;
	for (int i = 0; i < num_iter; i++){
		LT2_total += double(LT2.get_value());
	}
	LT2_average = LT2_total / num_iter;
}

void Sensors::vis_sense(void* ptr){

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