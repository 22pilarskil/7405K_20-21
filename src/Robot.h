#include "main.h"
#include "PD.h"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
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
	static ADIUltrasonic UT;
	static ADIAnalogIn LB1;
	static ADIAnalogIn LB2;
	/* Initializing motors, sensors, controller */

	static PD power_PD;
	static PD strafe_PD;
	static PD turn_PD;
	/* Initializing Our PD Instances */

	static std::atomic<double> x;
	static std::atomic<double> y;
	static std::atomic<double> turn_offset_x;
	static std::atomic<double> turn_offset_y;
	/* Static member variables used to store information about positioning obtained from Robot::fps (our odometry 
	function) */

	static double offset_back;
	static double offset_middle;
	static double wheel_circumference;
	static int radius;
	/* Presets for odometry and pure pursuit calculations */

	static std::atomic<int> UT_count;
	static std::atomic<int> UB_count;
	static bool store_complete;
	static double fly_cap;
	/* Static member variables used to store information about location and number of balls being stored by our bot obtained 
	Robot::sensors */

	static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
	static std::atomic<double> BallsFrontAverage;
	/* Mapping of tasks instantiated during the program */

	static void start_task(std::string name, void (*func)(void *));
	static bool task_exists(std::string name);
	static void kill_task(std::string name);
	/* Threading functions */

	static void fps(void *ptr);
	static void move_to(std::vector<double> pose, std::vector<double> margins = {1, 1, 1}, std::vector<double> speeds = {1, 1, 1}, int seconds = 0, bool pure_pursuit = false);
	static void move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> speeds = {1, 1, 1});
	/* Autonomous movement and positioning functions */

	static void sensors(void *ptr);
	static void store(void *ptr);
	static std::vector<int> get_data();
	static void quickscore(int num_balls = 1, int speed = 1);
	static void set_fly_cap(double cap = 1);
	static void reset_balls(int ultrasonic_bottom = 0, int ultrasonic_top = 0, bool move_up_ = true, bool intake_store_ = false, bool intakes_on_ = true);
	/* Ball storing functions */

	static void display(void *ptr);
	static void drive(void *ptr);
	static void mecanum(int power, int strafe, int turn);
	static void intake(double coefficient, std::string powered = "both", bool fly_off = true, bool flip = false, bool macro = false);
	static void brake(std::string mode);
	static void flipout();
	static void vis_sense(void *ptr);
	static void reset_sensors();
	static void reset_PD();
	static void BallsUpdating(void *ptr);
	static bool BallsChecking(double coefficient);

	/* Driver control functions */
};
