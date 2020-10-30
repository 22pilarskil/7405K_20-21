#include "Robot.h"
#include "PurePursuit.h"
#include "filter.h"
#include <cmath>
#include <atomic>
#include <vector>
#include <numeric>
#include <chrono>
#include <unordered_map>
using namespace pros;

#define TO_RAD(n) n * M_PI / 180;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(10, true);
Motor Robot::FR(14);
Motor Robot::BL(1);
Motor Robot::BR(19, true);
Motor Robot::IL(4);
Motor Robot::IR(11, true);
Motor Robot::R1(16);
Motor Robot::R2(12, true);
ADIEncoder Robot::LE(3, 4);
ADIEncoder Robot::RE(7, 8, true);
ADIEncoder Robot::BE(5, 6);
Imu Robot::IMU(8);
Vision Robot::vision(21);
ADIDigitalIn Robot::LM1({{5, 5}});
ADIUltrasonic Robot::UB(1, 2);
ADIUltrasonic Robot::UT({{5, 1, 2}});
PID Robot::power_PID(.2, 0, 1.5, 10);
PID Robot::strafe_PID(.25, 0, 1.5, 10);
PID Robot::turn_PID(0.85, 0, 0, 15);

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::turn_offset_x = 0;
std::atomic<double> Robot::turn_offset_y = 0;
std::atomic<int> Robot::UB_count = 0;
std::atomic<int> Robot::UT_count = 0;

double Robot::offset_back = 4 + 5 / 16;
double Robot::offset_middle = 5 + 7 / 16;
double Robot::wheel_circumference = 2.75 * M_PI;
bool Robot::store_complete;
std::vector<double> LE_values;
std::vector<double> RE_values;
std::vector<double> BE_values;
int radius = 300;
int UT_LastBall;
int UB_LastBall;
int storing_count;
bool intakes_on;
bool intake_store = false;
bool move_up = true;
int count_constant = 20;
std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

void Robot::vis_sense(void *ptr)
{
	vision_signature_s_t red_signature = Vision::signature_from_utility(1, -669, 5305, 2318, -971, 571, -200, 0.700, 0);
	vision_signature_s_t blue_signature = Vision::signature_from_utility(2, -3277, -2313, -2796, 9039, 13285, 11162, 3.000, 0);
	vision.set_signature(1, &red_signature);
	vision.set_signature(2, &blue_signature);
	while (true)
	{
		vision_object_s_t red_ball = vision.get_by_sig(0, 1);
		vision_object_s_t blue_ball = vision.get_by_sig(0, 2);
		int red_x_coord = red_ball.x_middle_coord;
		int blue_x_coord = blue_ball.x_middle_coord;
		delay(100);
	}
}


std::vector<int> Robot::get_data()
{
	return {int(UB_count), int(UT_count)};
}


void Robot::drive(void *ptr)
{
	delay(300);
	int intake_state = 1;
	bool intake_last;

	while (true)
	{
		int power = master.get_analog(ANALOG_LEFT_Y);
		int strafe = master.get_analog(ANALOG_LEFT_X);
		int turn = master.get_analog(ANALOG_RIGHT_X);

		if (master.get_digital(DIGITAL_LEFT)) move_to({0, 0, IMU.get_rotation() / 360 * 360});

		mecanum(power, strafe, turn);

		bool intake_ = master.get_digital(DIGITAL_R2);
		bool outtake = master.get_digital(DIGITAL_X);
		bool just_intake = master.get_digital(DIGITAL_R1);
		bool just_indexer = master.get_digital(DIGITAL_L2);
		bool flip = master.get_digital(DIGITAL_L1);
		bool storingScore = master.get_digital(DIGITAL_RIGHT);
		bool quickScore_ = master.get_digital(DIGITAL_A);

		if (storingScore && !intake_last){
			intake_state++;
			intake_last = true;
			reset_Balls();
		}
		else if (!storingScore)intake_last = false;

		if (intake_state % 2 == 0){
			//store();
            Robot::start_task("STORE", Robot::store);
		}
		else {
            Robot::kill_task("STORE");
            double motorpwr = 0;
			if (intake_ || outtake) motorpwr = (intake_) ? 1 : -1;
			if (just_intake && just_indexer) intake(1, false, "both");
			else if (just_intake) intake(1, flip, "intakes");
			else if (just_indexer) intake(1, flip, "indexer");
			else if (quickScore_) quickscore();
			else intake(motorpwr, flip, "both");
		}
	}
}


void Robot::quickscore(int ball_id) //1 for top only, 0 for bottom only, -1 for both
{
	R2 = 127;
	if (ball_id == -1) {
		delay(600);
		R1 = -127;
	}
	else if (ball_id = 0){
		R1 = 127;
	}
	delay(1200);
	R1 = 0;
	R2 = 0;
}


void Robot::store(void *ptr)
{
	lcd::print(7, "STORE INCOMPLETE");
	while(true)
	{
		int sensorTop = int(UT_count - UT_LastBall);
		int sensorBottom = int(UB_count - UB_LastBall);
		lcd::print(1, "%d %d %d %d", UB.get_value(), int(UB_count));
		lcd::print(2, "%d %d", UT.get_value(), int(UT_count));
		lcd::print(7, "T: %d B: %d", sensorTop, sensorBottom);
		if (intakes_on){
			IL = 127;
			IR = 127;
		}
		if (sensorTop == 1 && sensorBottom == 1){
			R1 = -127;
			R2 = 0;
		}
		else if (sensorTop == 0 && sensorBottom <= 1){
			R1 = -127;
			R2 = 50;
		}
		else if (sensorTop >= 1 && sensorBottom >= 2){
			R1 = 0;
			R2 = 0;
			if (move_up){
				R1 = -127;
				delay(150);
				R1 = 0;
				R2 = 0;
				move_up = false;
			}
			if (intake_store){
				while(LM1.get_value() == 0){
					IR = 127;
					IL = 127;
					delay(1);
				}
				IR = 0;
				IL = 0;
				break;
			}
			else break;
		}
		delay(5);
	}
	lcd::print(7, "STORE COMPLETE");
	store_complete = true;
	IL = 0;
	IR = 0;
}


void Robot::flipout()
{
	IL = -127;
	IR = -127;
	R1 = 127;
	R2 = 127;
}


void Robot::intake(double coefficient, bool flip, std::string powered)
{
	if (coefficient == 0)
	{
		IL = 0;
		IR = 0;
		R1 = 0;
		R2 = 0;
		return;
	}
	if (powered.compare("intakes") == 0 || powered.compare("both") == 0)
	{
		IL = int(coefficient * 127);
		IR = int(coefficient * 127);
		if (!powered.compare("both") == 0) {
			R1 = 0;
			R2 = 0;
		}
	}
	if (powered.compare("indexer") == 0 || powered.compare("both") == 0)
	{
		R1 = -coefficient * 127;
		coefficient = std::max(0.0, coefficient);
		R2 = (!flip) ? coefficient * 127 : -coefficient * 127;
		if (!powered.compare("both") == 0) {
			IL = 0;
			IR = 0;
		}
	}
}


void Robot::fps(void *ptr)
{
	double last_x = 0;
	double last_y = 0;
	double last_phi = 0;
	while (true)
	{
		double cur_phi = TO_RAD(IMU.get_rotation());
		double dphi = cur_phi - last_phi;

		double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
		double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;
		/* Calculate how much the encoders have turned as a result of turning ONLY in order to
		isolate readings representing lateral or axial movement from readings representing
		turning in place */

		turn_offset_x = (float)turn_offset_x + cur_turn_offset_x;
		turn_offset_y = (float)turn_offset_y + cur_turn_offset_y;

		double cur_y = ((LE.get_value() - turn_offset_y) + (RE.get_value() + turn_offset_y)) / 2;
		double cur_x = BE.get_value() - turn_offset_x;

		double dy = cur_y - last_y;
		double dx = cur_x - last_x;

		double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
		double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);
		//Apply rotation matrix to dx and dy to calculate dx and dy on the phi = 0 orientation

		y = (float)y + global_dy;
		x = (float)x + global_dx;

		lcd::print(3, "IMU value: %f", IMU.get_rotation());
		lcd::print(4, "Offset: %d - Y: %f", int(turn_offset_y), (float)y);
		lcd::print(5, "Offset: %d - X: %f", int(turn_offset_x), (float)x);
		printf("Y: %f - X: %f - IMU value: %f\n", (float)y, (float)x, IMU.get_rotation());

		last_y = cur_y;
		last_x = cur_x;
		last_phi = cur_phi;

		delay(5);
	} 
}


void Robot::sensors(void *ptr)
{
	int UB_reset = 0;
	int UT_reset = 0;
	int LM_triggered = false;
	while (true)
	{
		if (LM1.get_value() == 1){
			LM_triggered = true;
		}
		if (UB.get_value() < 150 && LM_triggered){
			if (UB_reset > 10) {
				UB_count++;
				LM_triggered = false;
			}
			UB_reset = 0;
		}
		else if (UB.get_value() > 150 && UB.get_value() < 250){
			UB_reset += 1;
		}

		if (UT.get_value() < 150){
			if (UT_reset > 10) UT_count++;
			UT_reset = 0;
		}
		else if (UT.get_value() > 150 && UT.get_value() < 250){
			UT_reset += 1;
		}
		delay(5);
	}
}


void Robot::mecanum(int power, int strafe, int turn)
{
	FL = power + strafe + turn;
	FR = power - strafe - turn;
	BL = power - strafe + turn;
	BR = power + strafe - turn;
	delay(5);
}


void Robot::display(void *ptr)
{
	while (true){
		master.print(0, 0, "Joystick %d", master.get_analog(ANALOG_LEFT_X));
		//lcd::print(1, "LE: %d - RE: %d", LE.get_value(), RE.get_value());
		//lcd::print(2, "Back Encoder: %d", BE.get_value());
		lcd::print(6, "Limit switch %d", LM1.get_value());
		//lcd::print(3, "IMU value: %f", IMU.get_rotation());

		delay(10);
	}
}


void Robot::move_to(std::vector<double> pose, std::vector<double> margin, std::vector<double> speeds, bool pure_pursuit, int coefficient, bool flip, std::string powered)
{
	double new_y = pose[0];
	double new_x = pose[1];
	double heading = pose[2];

	double y_error = new_y - y;
	double x_error = -(new_x - x);

	double heading2 = (heading < 0) ? heading + 360 : heading - 360;
	heading = (abs(IMU.get_rotation() - heading) < abs(IMU.get_rotation() - heading2)) ? heading : heading2;
	double imu_error = -(IMU.get_rotation() - heading);
	/* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current
	heading (i.e. at IMU val 150, travel to 1 deg (|150 - 1| = 149 deg traveled) as opposed to -359 deg
	(|150 - (-359)| = 509 deg traveled) */

	while (abs(y_error) > 30 * margin[0] || abs(x_error) > 30 * margin[1] || abs(imu_error) > 2 * margin[2])
	{ //while both goals are not reached
		double phi = TO_RAD(IMU.get_rotation());
		double power = power_PID.get_value(y_error * std::cos(phi) + x_error * std::sin(phi)) * speeds[0];
		double strafe = strafe_PID.get_value(x_error * std::cos(phi) - y_error * std::sin(phi)) * speeds[1];
		double turn = turn_PID.get_value(imu_error) * 1.5 * speeds[2];
		//Apply rotation matrix to errors as they are derived from calculations using rotation matrices

		imu_error = -(IMU.get_rotation() - heading);
		y_error = new_y - y;
		x_error = -(new_x - x);

		mecanum(power, strafe, turn);

		if (pure_pursuit)
		{
			return;
		}
	}
	reset_PID();
	lcd::print(6, "DONE");
	brake("stop");
}


void Robot::move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> speeds, int coefficient, bool flip, std::string powered)
{

	std::vector<double> end;
	std::vector<double> start;
	std::vector<double> target;
	std::vector<double> cur{(float)y, (float)x};
	double heading;

	for (int index = 0; index < points.size() - 1; index++)
	{
		start = points[index];
		end = points[index + 1];
		while (distance(cur, end) > radius)
		{
			target = get_intersection(start, end, cur, radius);
			heading = get_degrees(target, cur);
			std::vector<double> pose {target[0], target[1], heading};
			Robot::move_to(pose, {1, 1, 1}, speeds, true, coefficient, flip, powered);
			delay(10);
			cur = {(float)y, (float)x};
		}
	}

	double x_error = end[1] - x;
	double y_error = end[0] - y;
	double imu_error = IMU.get_rotation() - heading;

	brake("stop");
	reset_PID();
	lcd::print(6, "DONE");
	//lcd::print(7, "YE: %d - XE: %d - IE: %d", int(x_error), int(y_error), int(imu_error));
}


void Robot::brake(std::string mode)
{

	if (mode.compare("coast") == 0)
	{
		FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
		FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
		BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
		BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
	}
	else if (mode.compare("hold") == 0)
	{
		FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	}
	else FL = FR = BL = BR = 0;
}


void Robot::reset_sensors()
{
	IMU.reset();
}


void Robot::reset_PID()
{
	power_PID.reset();
	strafe_PID.reset();
	turn_PID.reset();
}


void Robot::reset_Balls(int ultrasonic_bottom, int ultrasonic_top, bool move_up_, bool intake_store_, bool intakes_on_)
{
	UT_count = ultrasonic_top;
	UB_count = ultrasonic_bottom;
	UT_LastBall = 0;
	UB_LastBall = 0;
	storing_count = 0;
	move_up = move_up_;
	intake_store = intake_store_;
	store_complete = false;
	intakes_on = intakes_on_;
}


void Robot::start_task(std::string name, void (*func)(void *))
{
	if (!task_exists(name)){
		tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
	}
}


bool Robot::task_exists(std::string name)
{
	return tasks.find(name) != tasks.end();
}


void Robot::kill_task(std::string name)
{
	if (task_exists(name)){
		tasks.erase(name);
	}
}


void Robot::LE_filter(void *ptr)
{
	vector<float> positions = {0};
	vector<float> distances = {0};
	while (true)
	{
		float position = LE.get_value();
		positions.push_back(position);

		float distance = position - distances[-1];
		distances.push_back(distance);

		float meanSensor = accumulate(positions.begin(), positions.end(), 0) / positions.size();
		vector<double> position_var;
		for (int post_count; post_count < distances.size(); post_count++)
		{
			position_var.push_back(pow(positions[post_count] - meanSensor, 2));
		}
		float varSensor = accumulate(position_var.begin(), position_var.end(), 0) / distances.size();

		float meanMove = accumulate(distances.begin(), distances.end(), 0) / distances.size();
		vector<double> distance_var;
		for (int dist_count; dist_count < distances.size(); dist_count++)
		{
			distance_var.push_back(pow(distances[dist_count] - meanMove, 2));
		}
		float varMove = accumulate(distance_var.begin(), distance_var.end(), 0) / distances.size();

		Filter Filter0(0, 0, meanSensor, varSensor, meanMove, varMove, positions, distances);
		LE_values = Filter0.get_prediction();
	}
}

void Robot::RE_filter(void *ptr)
{
	vector<float> positions = {0};
	vector<float> distances = {0};
	while (true)
	{
		float position = RE.get_value();
		positions.push_back(position);

		float distance = position - distances[-1];
		distances.push_back(distance);

		float meanSensor = accumulate(positions.begin(), positions.end(), 0) / positions.size();
		vector<double> position_var;
		for (int post_count; post_count < distances.size(); post_count++)
		{
			position_var.push_back(pow(positions[post_count] - meanSensor, 2));
		}
		float varSensor = accumulate(position_var.begin(), position_var.end(), 0) / distances.size();

		float meanMove = accumulate(distances.begin(), distances.end(), 0) / distances.size();
		vector<double> distance_var;
		for (int dist_count; dist_count < distances.size(); dist_count++)
		{
			distance_var.push_back(pow(distances[dist_count] - meanMove, 2));
		}
		float varMove = accumulate(distance_var.begin(), distance_var.end(), 0) / distances.size();

		Filter Filter0(0, 0, meanSensor, varSensor, meanMove, varMove, positions, distances);
		RE_values = Filter0.get_prediction();
	}
}

void Robot::BE_filter(void *ptr)
{
	vector<float> positions = {0};
	vector<float> distances = {0};
	while (true)
	{
		float position = BE.get_value();
		positions.push_back(position);

		float distance = position - distances[-1];
		distances.push_back(distance);

		float meanSensor = accumulate(positions.begin(), positions.end(), 0) / positions.size();
		vector<double> position_var;
		for (int post_count; post_count < distances.size(); post_count++)
		{
			position_var.push_back(pow(positions[post_count] - meanSensor, 2));
		}
		float varSensor = accumulate(position_var.begin(), position_var.end(), 0) / distances.size();

		float meanMove = accumulate(distances.begin(), distances.end(), 0) / distances.size();
		vector<double> distance_var;
		for (int dist_count; dist_count < distances.size(); dist_count++)
		{
			distance_var.push_back(pow(distances[dist_count] - meanMove, 2));
		}
		float varMove = accumulate(distance_var.begin(), distance_var.end(), 0) / distances.size();

		Filter Filter0(0, 0, meanSensor, varSensor, meanMove, varMove, positions, distances);
		BE_values = Filter0.get_prediction();
	}
}