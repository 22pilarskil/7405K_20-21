#include "main.h"
#include "PID.h"
#include <map>
#include <memory>
#include <string>
#include <vector>
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
		static Motor R1;
		static Motor R2;
		static ADIEncoder LE;
		static ADIEncoder RE;
		static ADIEncoder BE;
		static Imu IMU;
		static Vision vision;
		static ADIAnalogIn LT1;
		static ADIAnalogIn LT2;


		static PID power_PID;
		static PID strafe_PID;
		static PID turn_PID;

		static std::atomic<double> x;
		static std::atomic<double> y;
		static std::atomic<double> turn_offset_x;
		static std::atomic<double> turn_offset_y;
		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
		static double offset_back;
		static double offset_middle;
		static double wheel_circumference;

    static void move_to(double new_y, double new_x, double heading, bool pure_pursuit = false, int intakes = 0, bool rollers = false, double scale = 1);
		static void move_to_pure_pursuit(std::vector<std::vector<double>> points, double scale = 1, int intakes = 0, bool rollers = false);
		static void brake(std::string mode);
		static void drive(void* ptr);
		static void intake(int coefficient, bool flip, bool rollers=false);
		static void lin_intake(int balls);
		static void fps(void* ptr);
		static void vis_sense(void* ptr);
		static void display(void* ptr);
		static void mecanum(int power, int strafe, int turn);
		static void reset_sensors();
		static void reset_PID();
		static void start_tasks();
		static void start_task(std::string name, void (*func)(void*));
		static bool task_exists(std::string name);

};