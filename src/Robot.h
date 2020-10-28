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

	static PID power_PID;
	static PID strafe_PID;
	static PID turn_PID;

	static std::atomic<double> x;
	static std::atomic<double> y;
	static std::atomic<double> turn_offset_x;
	static std::atomic<double> turn_offset_y;
	static std::atomic<int> UT_count;
	static std::atomic<int> UB_count;

	static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
	static double offset_back;
	static double offset_middle;
	static double wheel_circumference;
	static bool store_complete;

	static void move_to(std::vector<double> pose, std::vector<double> margins = {1, 1, 1}, std::vector<double> speeds = {1, 1, 1}, bool pure_pursuit = false, int coefficient = 0, bool flip = false, std::string powered = "both");
	static void move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> speeds = {1, 1, 1}, int coefficient = 0, bool flip = false, std::string powered = "both");
	static void brake(std::string mode);
	static void drive(void *ptr);
	static std::vector<int> get_data();
	static void intake(double coefficient, bool flip = false, std::string powered = "both");
	static void fps(void *ptr);
	static void sensors(void *ptr);
	static void vis_sense(void *ptr);
	static void display(void *ptr);
	static void mecanum(int power, int strafe, int turn);
	static void BE_filter(void *ptr);
	static void LE_filter(void *ptr);
	static void RE_filter(void *ptr);
	static void quickscore(int ball_id = -1);
	static void store(void *ptr);
	static void reset_sensors();
	static void reset_Balls(int ultrasonic_bottom = 0, int ultrasonic_top = 0, bool move_up_ = true, bool intake_store_ = false, bool intakes_on_ = true);
	static void reset_PID();
	static void flipout();
	static void start_tasks();
	static int balls_ejected_count();
	static int balls_intook_count();
	static void start_task(std::string name, void (*func)(void *));
	static bool task_exists(std::string name);
	static void kill_task(std::string name);
};
