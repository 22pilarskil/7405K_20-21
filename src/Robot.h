#include "main.h"
#include <map>
#include <memory>
#include <string>
using namespace pros;

#define sRobot Robot::Instance();

class Robot{
	public:
		static Controller master;
		static Motor FL;
		static Motor FR;
		static Motor BL;
		static Motor BR;
		static void move_to(int x, int y);
		static void brake(std::string mode);
		static void drive(void* x);
		static int x;
		static void start_tasks();
		static void start_task(std::string name, void (*func)(void*));
		static bool task_exists(std::string name);
		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

};