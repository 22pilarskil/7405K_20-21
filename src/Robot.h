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
	static Motor IL;
	static Motor IR;
	static Motor R1;
	static Motor R2;
	static ADIEncoder LE;
	static ADIEncoder RE;
	static ADIEncoder BE;
	static Imu IMU;
    static ADIUltrasonic USF;
    static ADIAnalogIn LSI;
    static ADIAnalogIn LSS;
    static ADIDigitalIn LMR;
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

	static std::atomic<int> ejector_count;
	static std::atomic<int> intake_count;
	static std::atomic<int> shooting_count;
    static std::atomic<int> storing_count;
    static std::atomic<int> BallsShootAverage;
    static std::atomic<int> BallsStoreAverage;

	static std::atomic<bool> intaking;
	static std::atomic<int> outtake_delay;
    static std::atomic<int> outtake_opening_delay;
    static std::atomic<bool> close_intakes;

    static std::atomic<int> driver_type;
    static std::atomic<int> progress;
    /* Static member variables used to store information about location and number of balls being stored by our bot obtained
	Robot::sensors */

	static double fly_power;
	static double increment;
	static double fly_cap;
	static bool pass;
	static bool driver;
	static bool time_end;
	static std::string recorded_points;
	/* Static member variables for flywheel control */

	static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
	/* Mapping of tasks instantiated during the program */

	static void start_task(std::string name, void (*func)(void *));
	static bool task_exists(std::string name);
	static void kill_task(std::string name);
	/* Threading functions */

	static void fps(void *ptr);
	static void move_to(std::vector<double> pose, double stop_threshold = .1, bool pure_pursuit = false, int flipout_timer = 0, std::vector<double> speeds = {1, 1, 1});
	static void move_to_pure_pursuit(std::vector<std::vector<double>> points, std::vector<double> final_point, std::vector<double> speeds = {1, 1, 1});
	/* Autonomous movement and positioning functions */

	static void quickscore(int num_balls = 1, int speed = 1);
	static void set_fly_cap(double cap = 1);

    static int count();
    static bool check_intaking();
	static void balls_checking(void *ptr);
    static void sensing(void *ptr);
    static void balls_intake_toggle(int outtake_delay_=0, int outtake_opening_delay_=0, bool close_intakes_=false);
    static void balls_outtake(void *ptr);

    static void balls_intake(void *ptr);
    static void set_pass(bool pass_);
    static void record_thread(void *ptr);
	static void toggle_storing_count(int storing_numb);
	static void get_storing_count();
    static void shoot(void *ptr);
    static void store(void *ptr);
    static void shoot_store(int shoot, int store);
	static void driverStore1(void *ptr);
    static void driverStore2(void *ptr);
    static void flipout(void *ptr);
    /* Ball storing functions */

	static void display(void *ptr);
	static void drive(void *ptr);
	static void drive_tune(void *ptr);
	static void save_point(void *ptr);
    static int progress_state();
    static int get_driver_type();
    static void mecanum(int power, int strafe, int turn);
	static void intake(std::vector<int> coefficients);
	static void brake(std::string mode);
	static void reset_sensors();
	static void reset_PD();
	static void record_points();
	/* Driver control functions */

	static void read_autonomous();
	static void time_run(void *ptr);
	static void end_timer();
	/* Collecting Data */
};
