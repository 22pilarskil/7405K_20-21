#include "Robot.h"
#include "PurePursuit.h"
#include <cmath>
#include <atomic>
#include <vector>
#include <numeric>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <bits/stdc++.h> 
using namespace pros;
using namespace std;

#define TO_RAD(n) n * M_PI / 180;
/* Lambda function to convert number in degrees to radians. */

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FL(14);
Motor Robot::FR(10, true);
Motor Robot::BL(12);
Motor Robot::BR(18, true);
Motor Robot::IL(5, true);
Motor Robot::IR(21);
Motor Robot::R1(7);
Motor Robot::R2(8, true);
ADIEncoder Robot::LE(5, 6);
ADIEncoder Robot::RE(1, 2, true);
ADIEncoder Robot::BE(3, 4);
Imu Robot::IMU(4);
ADIAnalogIn Robot::LB1({{6, 6}});
ADIAnalogIn Robot::LF2({{6, 7}});
ADIAnalogIn Robot::LF1({{6, 8}});
ADIUltrasonic Robot::UF({{6, 1, 2}});
ADIUltrasonic Robot::UT(7, 8);
ADIDigitalIn Robot::LM1({{6, 3}});
/* Initializing motors, sensors, controller */

//.4, 0.1, 5
//.4, 0.1, 5
PD Robot::power_PD(.45, 0.1, 5);
PD Robot::strafe_PD(.45, 0.1, 5);
PD Robot::turn_PD(3, 0, 5);
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

std::atomic<int> Robot::ejector_count = -1;
std::atomic<int> Robot::intake_count = 0;
std::atomic<int> Robot::shooting_count = 0;
std::atomic<int> Robot::storing_count = 0;
std::atomic<double> Robot::BallsFrontAverage = 0;
std::atomic<double> Robot::BallsBackAverage = 0;
std::atomic<bool> Robot::intaking = false;
std::atomic<int> Robot::outtake_delay = 0;
std::atomic<int> Robot::outtake_opening_delay = 0;
std::atomic<bool> Robot::close_intakes;
std::atomic<double> Robot::updateDelay = 5;
std::atomic<double> Robot::checkDelay = 5;
/* Static member variables used to store information about location and number of balls being stored by our bot obtained
Robot::sensors */

double Robot::fly_power = 0;
double Robot::increment = 1;
double Robot::fly_cap = 1;
bool Robot::pass = false;
/* Static member variables for flywheel control */

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

//		lcd::print(3, "IMU value: %f", IMU.get_rotation());
		lcd::print(5, "Y: %f - X: %f", (float)y, (float)x);
		//printf("Y: %f - X: %f - IMU value: %f\n", (float)y, (float)x, IMU.get_rotation());

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
void Robot::move_to(std::vector<double> pose, std::vector<double> margin, std::vector<double> speeds, bool pure_pursuit)
{
	double new_y = pose[0];
	double new_x = pose[1];
	double heading = pose[2];

	double y_error = new_y - y;
	double x_error = -(new_x - x);
	int coefficient = 0;
	std::string powered = "intakes";

	int time = 0;

	double heading2 = (heading < 0) ? heading + 360 : heading - 360;
	heading = (abs(IMU.get_rotation() - heading) < abs(IMU.get_rotation() - heading2)) ? heading : heading2;
	double imu_error = -(IMU.get_rotation() - heading);
	/* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current heading. For 
	example, moving to -358 deg would require almost a full 360 degree turn from 1 degree, but from its equivalent of -359
	deg, it only takes a minor shift in position */

	while (abs(y_error) > 30 * margin[0] || abs(x_error) > 30 * margin[1] || abs(imu_error) > 1 * margin[2])
	{ /* while Robot::y, Robot::x and IMU heading are all more than the specified margin away from the target */
		
		// if (time < seconds) intake(coefficient, powered);
		// else intake(0);
		
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
		delay(5);
		time += 5;
	}
	reset_PD();
	//lcd::print(6, "DONE");
	brake("stop");
}

/**
 * @desc: Interfaces with PurePursuit.cpp to follow a smooth path generated from input points
 * @param points: An array of points in the form {{Y_1, X_1}..{Y_n, X_n}} whose direct pathing (i.e. what would result if 
 	a straight line was drawn between each consecutive pair of points) serves as the model for our curved, generated path 
 * @param speeds: Same function and format as @param speeds from Robot::move_to, see above
 */
void Robot::move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> final_point, std::vector<double> speeds)
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
			move_to(pose, {1, 1, 1}, speeds, true);
			cur = {(float)y, (float)x};
			delay(5);
		}
	}
	move_to(final_point, {1, 1, 1});
	brake("stop");
	reset_PD();
	//lcd::print(6, "DONE");
}


/**
 * @desc Takes in information about where balls are as well as how many there are to shoot them in quick succession
 * @param ball_id: 1 to shoot from top stored position only, 0 to shoot a ball from bottom store, -1 to shoot from both
 */ 

void Robot::set_fly_cap(double cap){
	fly_cap = cap;
}





void Robot::balls_checking(void *ptr) {
    int intake_toggle=0;
    bool ejector_toggle=false;

    bool ut_toggle=false;
	bool uf_toggle=false;

    std::deque<double> BallsFront;
    std::deque<double> BallsBack;


    while (true) {

        int BallsBackLength = (int) BallsBack.size();
        BallsBack.push_back(LB1.get_value());
        if(BallsBackLength == 10) {
            BallsBack.pop_front();
            int sum = 0;
            for(int i = 0; i<BallsBack.size(); i++) sum += BallsBack[i];
            BallsBackAverage = sum/10;
        }

        bool ball_at_ejector = abs(BallsBackAverage-LB1.get_value()) > 750;
        if(ball_at_ejector && !ejector_toggle) {
            ejector_count++;
            ejector_toggle=true;
        } else if (!ball_at_ejector && ejector_toggle) ejector_toggle=false;


		bool shoot_ball = UT.get_value() < 100;
        if(shoot_ball && !ut_toggle) {
            shooting_count++;
            ut_toggle = true;
        } else if (!shoot_ball && ut_toggle) ut_toggle = false;

        bool intake_ball = LM1.get_value();
        if(intake_ball && intake_toggle==8) {
            intake_count++;
            intake_toggle=0;
        } else if (!intake_ball && intake_toggle<8) intake_toggle++;

        storing_count = intake_count-(ejector_count+shooting_count);
        delay(30);
	}
}

int Robot::count(){
	return int(intake_count);
}

bool Robot::check_intaking(){
    return bool(intaking);
}

void Robot::balls_intake(void *ptr) {
    intaking = true;
    bool outtake=false;
    int outtake_count = 0;

    delay(outtake_opening_delay);
    IL = -127 * .5;
    IR = -127 * .5;
    delay(outtake_delay);
    IL = 0;
    IR = 0;

    while(true) {
        if(UF.get_value() < 300 && close_intakes) {
            Robot::intake({127, 127, 127, 0});
            intaking = false;
            break;
        }
        delay(5);
    }
}

void Robot::balls_intake_toggle(int outtake_delay_, int outtake_opening_delay_, bool close_intakes_){
	outtake_delay = outtake_delay_;
    outtake_opening_delay = outtake_opening_delay_;
    close_intakes = close_intakes_;
}

/**
 * @desc: Thread that displays important information to the brain.
 * @param ptr: Required for compatibility with pros threading
 */
void Robot::display(void *ptr)
{
	while (true) {

        master.print(0, 0, "Joystick %d", master.get_analog(ANALOG_LEFT_X));
        lcd::print(1, "LE: %d - RE: %d", LE.get_value(), RE.get_value());
        lcd::print(2, "Back Encoder: %d", BE.get_value());
        lcd::print(3, "IMU value: %f", IMU.get_rotation());
        lcd::print(4, "LF1: %d LF2: %d LB1: %d", LF1.get_value(), LF2.get_value(), LB1.get_value());
        //lcd::print(5, "UF: %d UT: %d SC: %d %d", UF.get_value(), UT.get_value(), (int) storing_count, (int) shooting_count);
        lcd::print(6, "Intake: %d shoot: %d UF: %d", (int) intake_count, (int) shooting_count, UF.get_value());
        lcd::print(7, "EjectorSensors: %d %d", (int) ejector_count, (int) BallsBackAverage);

        delay(10);
    }
}


void Robot::set_pass(bool pass_){
	pass = pass_;
}

//TODO possibly make it so second stage of acceleration with 3 balls
void Robot::shoot_store(int shoot, int store){
    if (pass){
        return;
    }

    if (shoot > 0) {
    	R1 = -127 * .5;
	    delay(100);
	    R2 = -127 * .1;
	    delay(100);
    }

    double R1_coefficient = .35;
    double R2_coefficient = 1;

    int last_shooting_count = shooting_count;
    int last_intake_count = int(intake_count);

    bool off_1 = true;
    bool off_2 = true;

    int time = 0;

    while(shooting_count - last_shooting_count < shoot || intake_count - last_intake_count < store){
		if (shooting_count - last_shooting_count >= shoot && off_1){
		    R1_coefficient = .5;
		    off_1 = false;
		}
		if (shooting_count - last_shooting_count >= 1 && off_2){
		    off_2 = false;
		    R1 = 0;
		    delay(200);
		}
        if (intake_count - last_intake_count < store){
            IL = 127;
            IR = 127;
            R1 = 127 * R1_coefficient;
        }
        else {
            if (shooting_count - last_shooting_count == shoot) R1_coefficient = 1;
            IL = 0;
            IR = 0;
        }

        if (shooting_count - last_shooting_count < shoot){
            R1 = 127 * R1_coefficient;
            R2 = 127 * R2_coefficient;
        }
        else {
            time += 1;
            if (time > 100) R2.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        }
        delay(1);
    }
    R1 = 127;
    delay(50);
    intake({0,0,0,0});
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
	bool ejector_state=false;

	bool store_state=false;
	int last_store_count;

	while (true) {
		int power = master.get_analog(ANALOG_LEFT_Y);
		int strafe = master.get_analog(ANALOG_LEFT_X);
		int turn = master.get_analog(ANALOG_RIGHT_X);

		if (master.get_digital(DIGITAL_LEFT)) move_to({0, 0, IMU.get_rotation() / 360 * 360});

		mecanum(power, strafe, turn);

		//Intakes/Outtakes
		bool outtake = master.get_digital(DIGITAL_L1);
        bool intakes_indexer = master.get_digital(DIGITAL_R1);

        //Indexer/Flywheel
		bool indexer = master.get_digital(DIGITAL_X);
		bool ejector = master.get_digital(DIGITAL_L2);
		bool indexer_fly = master.get_digital(DIGITAL_R2);
		
		//Storing	
		bool store1 = master.get_digital(DIGITAL_B);
		bool store2 = master.get_digital(DIGITAL_Y);
        bool store3 = master.get_digital(DIGITAL_X);

		if ((store1 || store2) && !store_state) {
			last_store_count=intake_count;
			store_state=true;
			R1 = -127 * .5;
			delay(10);
			R2 = -127 * .1;
			delay(10);
		} else if (!(store1 || store2)) store_state=false;


		if((ejector || intakes_indexer) && !ejector_state) {
			ejector_state=true;
			if ((intakes_indexer && ejector_count%2 == 0) || ejector) ejector_count++;
		} else if (!ejector) ejector_state=false;
		bool eject = ejector_count%2 == 0;

		int IL_ = 0;
		int IR_ = 0;
		int R1_ = 0;
		int R2_ = 0;

//		lcd::print(6, "%d %d", int(last_store_count), int(intake_count));

		if (indexer_fly) {
            R1_ = 127;
            R2_ = 127;
		}


		if (intakes_indexer) {
			IL_ = IR_ = R1_ = 127;
			if (!indexer_fly) R2.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		}

		if (outtake) IL_ = IR_ = -127 * .6;

		if (eject) {
			if (UF.get_value() < 200) IL_ = IR_ = 127;
			R2_ = -127;
			R1_ = 127;
		}


        if (store1) {
            if (intake_count - last_store_count < 1) {
                IL_ = IR_ = 127;
                delay(150);
            }
            R2_ = 127;
            R1_ = 127 * .25;
        }

        if (store2) {
            if (intake_count - last_store_count < 2) {
                IL_ = IR_ = 127;
                delay(150);
            }
            R2_ = 127;
            R1_ = 127 * .25;
        }

        if (store3) {
            if (intake_count - last_store_count < 3) {
                IL_ = IR_ = 127;
                delay(150);
            }
            R2_ = 127;
            R1_ = 127 * .25;
        }

        intake({IL_, IR_, R1_, R2_});

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

	int powers[] {
		power + strafe + turn,
		power - strafe - turn,
		power - strafe + turn, 
		power + strafe - turn
	};

	int max = *max_element(powers, powers + 4);
	int min = abs(*min_element(powers, powers + 4));

	double true_max = double(std::max(max, min));
	double scalar = (true_max > 127) ? 127 / true_max : 1;

	FL = (power + strafe + turn) * scalar;
	FR = (power - strafe - turn) * scalar;
	BL = (power - strafe + turn) * scalar;
	BR = (power + strafe - turn) * scalar;
}

/**sss
 * @desc Mostly used in driver control, it is a function for controlling our intakes and rollers with simplicity
 * @param coefficient: The power of the motor
 * @param flip: The direction of our ejector motor, either in ejection mode or intake mode.
 * @param powered: Tells which motors to power (intakes only, indexer only, or both)
 */
void Robot::intake(std::vector<int> coefficients) {
	IL = coefficients[0];
	IR = coefficients[1];
	R1 = coefficients[2];
	R2 = coefficients[3];
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
