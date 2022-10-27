#ifndef MSCKF_UTILS_TIMECOST_H
#define MSCKF_UTILS_TIMECOST_H

#include <chrono>

class TimeCost{
public:
    TimeCost() {
        start = std::chrono::high_resolution_clock::now();  
    }

    double timecost() {
        end = std::chrono::high_resolution_clock::now();  

        double cost = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() * 1e-6;

        start = end;

        return cost;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif // MSCKF_UTILS_TIMECOST_H
