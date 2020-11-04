#include "main.h"
#include "Robot.h"
using namespace pros;

/*Runs at the beginning of every program started in mode initialize (configured by plugging controller into VEX 
competition switch). This is required to allow our sensors to callibrate: as a result, without starting our program 
in initialize, autonomous functions will not work properly */

void initialize() {
    Robot::reset_sensors();
}

