/*#include "main.h"
#include <atomic>
#include "BallTracker.h"
using namespace pros;

std::atomic<int> balls;
int initial_val;
int buffer;
ADIAnalogIn line_tracker;

BallTracker::BallTracker(int port, int buffer_){
	line_tracker = ADIAnalogIn(port);
    buffer = buffer_;
    initial_val = line_tracker.get_value();
}

void BallTracker::count_balls(void* ptr){
    int time = 0;
    while (true){
        if (line_tracker.get_value() < initial_val - buffer){
            if (time > 50){
                balls += 1;
            }
            time = 0;
        }
        delay(5);
        time += 5;
    }
}

double ball_count(){
    return balls;
}*/