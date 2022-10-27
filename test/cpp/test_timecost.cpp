#include "utils/time_cost.h"
#include <iostream>
#include <thread>
#include <unistd.h>

int main() {

  TimeCost test_timer;
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  usleep(1000000);
  printf("time cost(1): %.6f\n", test_timer.timecost());

  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  usleep(500000);

  printf("time cost(0.5): %.6f\n", test_timer.timecost());

}