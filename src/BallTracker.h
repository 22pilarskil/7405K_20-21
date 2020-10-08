/*#include "main.h"
#include <atomic>
using namespace std;
using namespace pros;

class BallTracker {
    public:
        BallTracker(int port, int buffer_);
        ADIAnalogIn line_tracker;
        void count_balls(void* ptr);
        double ball_count();
        std::atomic<int> balls;
        int initial_val;
        int buffer;


};*/