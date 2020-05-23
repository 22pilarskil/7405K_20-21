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
		static int x;
		static int y;
		static void move_to(int new_y, int new_x, int heading);
		static bool get_error(bool y_fwd, bool x_fwd, int new_y, int new_x);
		static void brake(std::string mode);
		static void drive(void* x);
		static void display(void* x);
		static void mecanum(int power, int strafe, int turn);
		static void reset_IMU();
		static void start_tasks();
		static void start_task(std::string name, void (*func)(void*));
		static bool task_exists(std::string name);
		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

};