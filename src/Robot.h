#include "main.h"
#include "Acceleration.h"
#include "PID.h"
#include <map>
#include <memory>
#include <string>
using namespace pros;

class Robot{
	public:
		static Controller master;
		static Motor FL;
		static Motor FR;
		static Motor BL;
		static Motor BR;
		static Motor IR;
		static Motor IL;
		static ADIEncoder LE;
		static ADIEncoder RE;
		static ADIEncoder BE;
		static Imu IMU;
		static Acceleration power_acc;
		static Acceleration strafe_acc;
		static Acceleration turn_acc;
		static PID power_PID;
		static PID strafe_PID;
		static PID turn_PID;
		static std::atomic<double> x;
		static std::atomic<double> y;
		static std::atomic<double> turn_offset_x;
		static std::atomic<double> turn_offset_y;
		static double offset_back;
		static double offset_middle;
		static double wheel_circumference;
		static void move_to(double new_y, double new_x, double heading);
		static bool get_error(bool y_fwd, bool x_fwd, double new_y, double new_x);
		static void brake(std::string mode);
		static void drive(void* ptr);
		static void fps(void* ptr);
		static void display(void* ptr);
		static void mecanum(int power, int strafe, int turn);
		static void reset_IMU();
		static void start_tasks();
		static void start_task(std::string name, void (*func)(void*));
		static bool task_exists(std::string name);
		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

};