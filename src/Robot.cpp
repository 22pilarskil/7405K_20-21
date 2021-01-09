#include "Robot.h"
#include "PurePursuit.h"
#include <cmath>
#include <atomic>
#include <vector>
#include <numeric>
#include <chrono>
#include <unordered_map>
#include <deque>
using namespace pros;

#define TO_RAD(n) n * M_PI / 180;
/* Lambda function to convert number in degrees to radians. */

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(2);
Motor Robot::FR(10, true);
Motor Robot::BL(12);
Motor Robot::BR(19, true);
Motor Robot::IL(5, true);
Motor Robot::IR(21);
Motor Robot::R1(7, true);
Motor Robot::R2(8, true);
ADIEncoder Robot::LE(5, 6);
ADIEncoder Robot::RE(1, 2, true);
ADIEncoder Robot::BE(3, 4);
Imu Robot::IMU(4);
ADIAnalogIn Robot::LB1({{6, 6}});
ADIAnalogIn Robot::LF1({{6, 7}});
ADIAnalogIn Robot::LF2({{6, 8}});
ADIUltrasonic Robot::UF({{6, 1, 2}});
ADIDigitalIn Robot::LabelBumper({{6, 3}});
/* Initializing motors, sensors, controller */

PD Robot::power_PD(.24, 1.2, 5);
PD Robot::strafe_PD(.24, .6, 4);
PD Robot::turn_PD(1.5, 0, 10);
/* Initializing Our PD Instances */

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::turn_offset_x = 0;
std::atomic<double> Robot::turn_offset_y = 0;
/* Static member variables used to store information about positioning obtained from Robot::fps (our odometry function) */

double Robot::offset_back = 7;
double Robot::offset_middle = 7;
double Robot::wheel_circumference = 2.75 * M_PI;
int Robot::radius = 300;
/* Presets for odometry and pure pursuit calculations */

std::atomic<int> Robot::ejector_count = 0;
std::atomic<int> Robot::intake_count = 0;
std::atomic<double> Robot::BallsFrontAverage = 0;
std::atomic<double> Robot::BallsBackAverage = 0;
bool Robot::store_complete;
/* Static member variables used to store information about location and number of balls being stored by our bot obtained 
Robot::sensors */

bool intakes_on;
bool intake_store;
bool move_up;
bool store_off;
/* Parameters passed into Robot::store */

double fly_power = 0;
double increment = 1;
double Robot::fly_cap = 1;

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;
/* Mapping of tasks instantiated during the program */

/* Note: tasks are the pros version of threads, or a method for having independent subroutines run at the same time. Using
threading allows us to have different functions run simultaneously, which helps us save time and increase versatility in
our code */


/**
 * @desc: Starts a task and pairs it with a unique task ID to allow us to keep track of its status
 * @param name: An specific name we give to the task for organization purposes
 * @param func: Pointer of the function in question that we would like to put into a new task
 */
void Robot::start_task(std::string name, void (*func)(void *)) {
	if (!task_exists(name)) {
		tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
	}
}

/**
 * @desc: Checks if task exists
 * @param name: Name of the task in question
 * @return: A boolean (true or false) that represents whether the task exists or not
 */
bool Robot::task_exists(std::string name) {
	return tasks.find(name) != tasks.end();
}

/**
 * @desc: Kills a specific task by terminating it and removing it from Robot::tasks
 * @param name: Name of task to be terminated
 */
void Robot::kill_task(std::string name) {
	if (task_exists(name)) {
		tasks.erase(name);
	}
}

/**
 * @desc: Threaded function that performs our odometry calculations at all times, updating Robot::x and Robot::y to
 	provide us an accurate depiction of our robot's real time positioning. Since Robot::x and Robot::y are static 
 	member variables of type atomic (a datatype designed to allow multiple threads to access a variable at once, they 
 	can be accessed from anywhere in the code at any time. 
 * @param ptr: Required for compatibility with pros threading
 */
void Robot::fps(void *ptr) {
	double last_x = 0;
	double last_y = 0;
	double last_phi = 0;
	while (true) {
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
		/* Apply rotation matrix to dx and dy to calculate global_dy and global_dx. Is required because if the Robot moves
		on an orientation that is not a multiple of 90 (i.e. 22 degrees), x and y encoder values do not correspond 
		exclusively to either x or y movement, but rather a little bit of both */

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
		/* All of these calculations assume that the Robot is moving in a straight line at all times. However, while this 
		is not always the case, a delay of 5 milliseconds between each calculation makes dx and dy (distance traveled on 
		x and y axes) so small that any curvature is insignificant. */
	}
}


/**
 * @desc: Interfaces with PD classes as well as Robot::x and Robot::y (updated using odometry in Robot::fps) to accurately 
 	move to an input position.
 * @param pose: A vector of length three in the format {Y, X, heading} that contains information about the target end state
 	of the robot that we wish to achieve through Robot::move_to
 * @param margin: A vector of length three in the format {Y_margin, X_margin, heading_margin} that allows us to control how 
 	accurate our movements should be by acting as coefficients for tolerances, or how close our robot actually needs to be 
 	to the target in order for Robot::move_to to be complete (Higher margins = less accurate but faster convergence)
 	within 2 degrees of our target heading, but we can multiply 
 * @param speeds: A vector of length three in the format {Y_speed, X_speed, heading_speed} that allows us to control how
 	fast our movements should be by acting as coefficients for speeds outputted by our PD objects. 
 * @param pure_pursuit: A boolean (true or false) that tells us whether or not we are calling this function in the context
 	of Robot::move_to_pure_pursuit
 */
void Robot::move_to(std::vector<double> pose, std::vector<double> margin, std::vector<double> speeds, int seconds, bool pure_pursuit, bool ball_wait)
{
	double new_y = pose[0];
	double new_x = pose[1];
	double heading = pose[2];

	double y_error = new_y - y;
	double x_error = -(new_x - x);
	int coefficient = 0;
	std::string powered = "intakes";

	if (seconds > 0) coefficient = -1;
	else if (seconds < 0){
		coefficient = 1;
		powered = "both";
	}

	seconds = abs(seconds);

	int time = 0;

	double heading2 = (heading < 0) ? heading + 360 : heading - 360;
	heading = (abs(IMU.get_rotation() - heading) < abs(IMU.get_rotation() - heading2)) ? heading : heading2;
	double imu_error = -(IMU.get_rotation() - heading);
	/* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current heading. For 
	example, moving to -358 deg would require almost a full 360 degree turn from 1 degree, but from its equivalent of -359
	deg, it only takes a minor shift in position */

	while (abs(y_error) > 30 * margin[0] || abs(x_error) > 30 * margin[1] || abs(imu_error) > 2 * margin[2])
	{ /* while Robot::y, Robot::x and IMU heading are all more than the specified margin away from the target */
		if (time < seconds) intake(coefficient, powered);
		else intake(0);
		double phi = TO_RAD(IMU.get_rotation());
		double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi)) * speeds[0];
		double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi)) * speeds[1];
		double turn = turn_PD.get_value(imu_error) * 1.5 * speeds[2];
		mecanum(power, strafe, turn);
		/* Using our PD objects we use the error on each of our degrees of freedom (axial, lateral, and turning movement)
		to obtain speeds to input into Robot::mecanum. We perform a rotation matrix calculation to translate our y and x 
		error to the same coordinate plane as Robot::y and Robot::x to ensure that the errors we are using are indeed 
		proportional/compatible with Robot::y and Robot::x */

		imu_error = -(IMU.get_rotation() - heading);
		y_error = new_y - y;
		x_error = -(new_x - x);
		/* Recalculating our error by subtracting components of our current position vector from target position vector */


		if (pure_pursuit) return;
		if (ball_wait && UF.get_value() < 100) {
			new_y = y;
			new_x = x;
		}
		delay(5);
		time += 5;
	}
	intake(0);
	reset_PD();
	lcd::print(6, "DONE");
	brake("stop");
}

/**
 * @desc: Interfaces with PurePursuit.cpp to follow a smooth path generated from input points
 * @param points: An array of points in the form {{Y_1, X_1}..{Y_n, X_n}} whose direct pathing (i.e. what would result if 
 	a straight line was drawn between each consecutive pair of points) serves as the model for our curved, generated path 
 * @param speeds: Same function and format as @param speeds from Robot::move_to, see above
 */
void Robot::move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> speeds)
{

	std::vector<double> end;
	std::vector<double> start;
	std::vector<double> target;
	std::vector<double> cur{(float)y, (float)x};
	double heading;
	/* Instantiating filler variables that will be overwritten every iteration, instead of allocating memory to new 
	objects */

	for (int index = 0; index < points.size() - 1; index++)
	{
		start = points[index];
		end = points[index + 1];
		while (distance(cur, end) > radius)
		{
			target = get_intersection(start, end, cur, radius);
			heading = get_degrees(target, cur);
			/* Obtain pathing information through functions from PurePursuit.cpp */

			std::vector<double> pose {target[0], target[1], heading};
			Robot::move_to(pose, {1, 1, 1}, speeds, true);
			cur = {(float)y, (float)x};
			delay(5);
		}
	}
	
	brake("stop");
	reset_PD();
	lcd::print(6, "DONE");
}


/**
 * @desc: Allows us to access ball counts generated by Robot::sensors for increased versatility (very rarely used, as 
 Robot::store already interfaces with ball counts)
 * @return: Global vars UB_count and UT_count
 *
 * std::vector<int> Robot::get_data() {
	return {int(UB_count), int(UT_count)};
}/


/**
 * @desc Takes in information about where balls are as well as how many there are to shoot them in quick succession
 * @param ball_id: 1 to shoot from top stored position only, 0 to shoot a ball from bottom store, -1 to shoot from both
 * void Robot::quickscore(int num_balls, int speed) {
	while(UT.get_value() > 300){
		intake(speed, "indexer", false);
	}
	delay(100);
	intake(0);
	if (num_balls == 1) return;
	while(UT.get_value() > 300){
		intake(speed, "indexer", false);
	}
	intake(0);
}
 */



void Robot::set_fly_cap(double cap){
	fly_cap = cap;
}


/**
 * @desc: Resets/Sets all of the global variables that our store function uses.
 * @param ultrasonic_bottom: Number to reset UB_count to
 * @param ultrasonic_top: Number to reset UT_count to
 * @param move_up_: Boolean (true or false) to tell whether or not we should move our balls further up in our intakes 
 	after storing- designed to counter the problem that our balls often store too low in our Robot due to imperfect 
 	geometry
 * @param intake_store_: Boolean (true or false) that tells us whether we want a third ball to be stored in between our
 	intakes
 * @param intakes_on_: Boolean (true or false) that tells us whether our intakes should be running during store (designed
 	for when we know that the ball(s) we want to store are both already contacting one of the indexer rollers, meaning 
 	our intakes are not needed to bring the balls further into the bot)
	 void Robot::reset_balls(int ultrasonic_bottom, int ultrasonic_top, bool move_up_, bool intake_store_, bool intakes_on_) {
	UT_count = ultrasonic_top;
	UB_count = ultrasonic_bottom;
	move_up = move_up_;
	intake_store = intake_store_;
	store_complete = false;
	intakes_on = intakes_on_;
}

 */

/**
 * @desc: Thread that displays important information to the brain.
 * @param ptr: Required for compatibility with pros threading
 */
void Robot::display(void *ptr)
{
	while (true){

		master.print(0, 0, "Joystick %d", master.get_analog(ANALOG_LEFT_X));
		lcd::print(1, "LE: %d - RE: %d", LE.get_value(), RE.get_value());
		lcd::print(2, "Back Encoder: %d", BE.get_value());
		lcd::print(3, "IMU value: %f", IMU.get_rotation());
		lcd::print(6, "LF1: %d LF2: %d", LF1.get_value(), LF2.get_value());
		lcd::print(7, "UF: %d",UF.get_value());
		// lcd::print(6, "FrontSensors: %d %d", (int) intake_count, (int) BallsFrontAverage);
		// lcd::print(7, "EjectorSensors: %d %d", (int) ejector_count, (int) BallsBackAverage);

		//lcd::print(7, "Ultrasonic: %d", UT.get_value());

		delay(10);
	}
}


/**
 * @desc:
 * @param ptr: Required for compatibility with pros threading
 */

void Robot::drive(void *ptr) {
	delay(300);

	int motorpwr = 0;
	std::string powered = "indexer";
	bool fly_off = false;
	bool flip = false;

	int ejector_count=1;
	int ejector_state=false;


	while (true) {
		int power = master.get_analog(ANALOG_LEFT_Y);
		int strafe = master.get_analog(ANALOG_LEFT_X);
		int turn = master.get_analog(ANALOG_RIGHT_X);

		if (master.get_digital(DIGITAL_LEFT)) move_to({0, 0, IMU.get_rotation() / 360 * 360});

		mecanum(power, strafe, turn);


		// bool quickscore_ = master.get_digital(DIGITAL_A);
        // if (quickscore_) quickscore();



		//Intakes/Outtakes
		bool outtake = master.get_digital(DIGITAL_L1);
        bool just_intakes_indexer = master.get_digital(DIGITAL_R1);
        bool intake_ = master.get_digital(DIGITAL_X);

        //Indexer/Flywheel
		bool just_indexer = master.get_digital(DIGITAL_X);
		bool slow_indexer_fly = master.get_digital(DIGITAL_B);
		bool just_indexer_fly = master.get_digital(DIGITAL_R2) || master.get_digital(DIGITAL_Y);
		
		//Slower shoot
		fly_cap = 1;
		if (master.get_digital(DIGITAL_Y)) fly_cap = .8;

        //flip
		bool poop = master.get_digital(DIGITAL_L2);
		if (poop) flip = true;

		if (poop || just_indexer || just_indexer_fly || just_intakes_indexer) motorpwr = 1;
		else if (outtake) motorpwr = -1;

		if (poop) flip = true;



		if((poop || just_intakes_indexer) && !ejector_state) {
			ejector_state=true;
			if(ejector_count%2 != 0 && just_intakes_indexer) ejector_count=ejector_count;
			else ejector_count++;
		} else if (!poop) ejector_state=false;

		if (just_intakes_indexer) 

		lcd::print(7, "ejector count %d", ejector_count);

		if (ejector_count % 2 == 0 && !just_intakes_indexer){
			bool macro = false;
			if (outtake) macro = true;
			intake(1, "indexer", false, true, macro);
		}
		else {

			if (just_intakes_indexer || outtake) {
				motorpwr = (just_intakes_indexer) ? 1 : -1;
				if (just_intakes_indexer) intake(motorpwr, "both", !just_indexer_fly, false);
				if (outtake) intake(motorpwr, "intakes", !just_indexer_fly, false);
			}
			else if (intake_) intake(1, "intakes");
			else if (poop) intake(1, "indexer", false, true);
			else if (just_indexer_fly) intake(1, "indexer", false, false);
			else if (just_indexer) intake(1, "indexer", true, false);
			else if (slow_indexer_fly) intake(0.15, "indexer", false, false, false, true);
			else intake(0);
		}
		delay(5);
	}
}


/**
 * @desc: The equation for holonomic driving (feeding values to our drivtrain motors to allow us to move axially, laterally, 
	or turn)
 * @param power: Degree of axial movement
 * @param strafe: Degree of strafing movement
 * @param turn: Degree of turning movement
 */
void Robot::mecanum(int power, int strafe, int turn) {
	FL = power + strafe + turn;
	FR = power - strafe - turn;
	BL = power - strafe + turn;
	BR = power + strafe - turn;
}

/**
 * @desc Mostly used in driver control, it is a function for controlling our intakes and rollers with simplicity
 * @param coefficient: The power of the motor
 * @param flip: The direction of our ejector motor, either in ejection mode or intake mode.
 * @param powered: Tells which motors to power (intakes only, indexer only, or both)
 */
void Robot::intake(double coefficient, std::string powered, bool fly_off,  bool flip, bool macro, bool fast_fly) {
	if (fly_power < fly_cap) fly_power += increment;
	if (coefficient == 0) {
		fly_power = 0;
		IL = 0;
		IR = 0;
		R1 = 0;
		R2 = 0;
		return;
	}
	if (macro){
		R1 = -coefficient * 127;
		coefficient = std::max(0.0, coefficient);
		if (fly_off) R2 = 0;
		else R2 = fly_power * ((!flip) ? coefficient * 127 : -coefficient * 127);
		double revised = -.4;
		IL = int(revised * 127);
		IR = int(revised * 127);

	}
	else {
		if (powered.compare("intakes") == 0 || powered.compare("both") == 0) {
			double revised = (coefficient > 0) ? coefficient : coefficient * .4;
			IL = int(revised * 127);
			IR = int(revised * 127);
			if (!powered.compare("both") == 0 && !macro) {
				R1 = 0;
				R2 = 0;
			}
		}
		if (powered.compare("indexer") == 0 || powered.compare("both") == 0) {
			R1 = -coefficient * 127;
			coefficient = std::max(0.0, coefficient);
			if (fly_off) R2 = 0;
			else if (fast_fly) R2 = fly_power * ((!flip) ?  127 : -127);
			else R2 = fly_power * ((!flip) ? coefficient * 127 : -coefficient * 127);
			if (!powered.compare("both") == 0 && !macro) {
				IL = 0;
				IR = 0;
			}
		}
	}
	//if (coefficient < 0) coefficient *= .1;
}


/**
 * @desc: Causes our robot's drivetrain to stop
 * @param mode: The type of break to perform (coast, hold, or neither)
 */
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


/**
 * @desc: Sets the motors to the correct power in order to allow our robot to flip out, or in other words automatically 
 expand from within the 18" size limit to our functional size (intakes and deflector shield are both out of size, must
 flip out to be legal)
 */
void Robot::flipout()
{
	IL = -127;
	IR = -127;
	R1 = 127;
	R2 = 127;
}

/**
 * @desc: Boiler plate code for recognizing balls through the use of the VEX vision sensor.
 * @param ptr: Required for compatibility with pros threading
 */
void Robot::vis_sense(void *ptr) {
	vision_signature_s_t red_signature = Vision::signature_from_utility(1, -669, 5305, 2318, -971, 571, -200, 0.700, 0);
	vision_signature_s_t blue_signature = Vision::signature_from_utility(2, -3277, -2313, -2796, 9039, 13285, 11162, 3.000, 0);
	vision.set_signature(1, &red_signature);
	vision.set_signature(2, &blue_signature);
	while (true) {
		vision_object_s_t red_ball = vision.get_by_sig(0, 1);
		vision_object_s_t blue_ball = vision.get_by_sig(0, 2);
		int red_x_coord = red_ball.x_middle_coord;
		int blue_x_coord = blue_ball.x_middle_coord;
		delay(100);
	}
}


void Robot::BallsUpdating(void *ptr) {
	std::deque<double> BallsFront;
	std::deque<double> BallsBack;

	while(true) {
		int BallsFrontLength = (int) BallsFront.size();
		BallsFront.push_back((LF1.get_value()+LF2.get_value())/2);
		if(BallsFrontLength == 10) {
			BallsFront.pop_front();
			int sum = 0;
			for(int i = 0; i<BallsFront.size(); i++) sum += BallsFront[i];
			BallsFrontAverage = sum/10;
		}

		int BallsBackLength = (int) BallsBack.size();
		BallsBack.push_back(LB1.get_value());
		if(BallsBackLength == 10) {
			BallsBack.pop_front();
			int sum = 0;
			for(int i = 0; i<BallsBack.size(); i++) sum += BallsBack[i];
			BallsBackAverage = sum/10;
		}
		delay(100);
	}
}


void Robot::BallsChecking(void *ptr) {
	double sensorAverages = (LF1.get_value()+LF2.get_value())/2;
	if(abs(BallsFrontAverage-sensorAverages) > 750) intake_count++;
	if(abs(BallsBackAverage-LB1.get_value()) > 750) ejector_count++;
}



void Robot::collectData(void *ptr) {
	FILE* data_store = fopen("/usd/example.txt", "w");
	std::deque<std::string> pastValues;

	while(true) {
		std::string data = "";
		for(int i = 0; i<pastValues.size(); i++) data += (pastValues[i] + " ");
		data+=LabelBumper.get_value()+"\n";
		fputs(const_cast<char*>(data.c_str()), data_store);

		int BallsFrontLength = pastValues.size();
		pastValues.push_back(std::to_string((LF1.get_value()+LF2.get_value())/2));
		if(BallsFrontLength > 15) pastValues.pop_front();
	}

	fclose(data_store);
}


/**
 * @desc: Resets IMU. Must be called in initialize.cpp and given at least 3 seconds to complete to allow IMU to calibrate
 */
void Robot::reset_sensors() {
	IMU.reset();
}

/**
 * @desc: Resets all PD objects
 */
void Robot::reset_PD() {
	power_PD.reset();
	strafe_PD.reset();
	turn_PD.reset();
}