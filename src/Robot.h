#include "main.h"
#include "PID.h"
#include <map>
#include <memory>
#include <string>
#include <vector>
using namespace pros;

class Robot
{
public:

	static Controller master;
	static Motor FL;
	static Motor FR;
	static Motor BL;
	static Motor BR;
	static Motor IR;
	static Motor IL;
	static Motor R1;
	static Motor R2;
	static ADIEncoder LE;
	static ADIEncoder RE;
	static ADIEncoder BE;
	static Imu IMU;
	static Vision vision;
	static ADIDigitalIn LM1;
	static ADIUltrasonic UB;
	static ADIUltrasonic UT;
	//Initializing motors, sensors, controller

	static PID power_PID;
	static PID strafe_PID;
	static PID turn_PID;
	//Initializing Our PD Instances

	static std::atomic<double> x;
	static std::atomic<double> y;
	static std::atomic<double> turn_offset_x;
	static std::atomic<double> turn_offset_y;
	//Static member variables used to store information about positioning obtained from Robot::fps (our odometry function)

	static double offset_back;
	static double offset_middle;
	static double wheel_circumference;
	static int radius;
	//Presets for odometry and pure pursuit calculations

	static std::atomic<int> UT_count;
	static std::atomic<int> UB_count;
	static bool store_complete;
	/* Static member variables used to store information about location and number of balls being stored by our bot obtained 
	Robot::sensors */

	static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
	// Mapping of tasks instantiated during the program

	static void start_task(std::string name, void (*func)(void *));
	static bool task_exists(std::string name);
	static void kill_task(std::string name);
	// Threading functions

	static void fps(void *ptr);
	static void move_to(std::vector<double> pose, std::vector<double> margins = {1, 1, 1}, std::vector<double> speeds = {1, 1, 1}, bool pure_pursuit = false);
	static void move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> speeds = {1, 1, 1});
	// Autonomous movement and positioning functions

	static void sensors(void *ptr);
	static void store(void *ptr);
	static std::vector<int> get_data();
	static void quickscore(int ball_id = -1);
	static void reset_Balls(int ultrasonic_bottom = 0, int ultrasonic_top = 0, bool move_up_ = true, bool intake_store_ = false, bool intakes_on_ = true);
	// Ball storing functions

	static void display(void *ptr);
	static void drive(void *ptr);
	static void mecanum(int power, int strafe, int turn);
	static void intake(double coefficient, bool flip = false, std::string powered = "both");
	static void brake(std::string mode);
	static void flipout();
	static void vis_sense(void *ptr);
	static void reset_sensors();
	// Driver control functions
};
