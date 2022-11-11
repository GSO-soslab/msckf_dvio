#include <iostream>
#include <string>
#include <sstream>

template <typename T>
std::string toCloneStamp(const T a_value, const int n = 9)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

int main() {

  double time = 1614971139.511340141;

  // same dimension
  std::string time_str_1 = toCloneStamp(time);
  double time_1 = std::stod(time_str_1);

  printf("bef time=%.9f\n", time);
  printf("aft time=%.9f\n", time_1);


  // wrong dimension
  std::string time_str_2 = toCloneStamp(time,6);
  double time_2 = std::stod(time_str_2);

  printf("bef time=%.9f\n", time);
  printf("aft time=%.9f\n", time_2);

  if(time == time_1){
    printf("time_1: yes");
  }
  else {
    printf("time_1: no");
  }

  if(time == time_2){
    printf("time_2: yes");
  }
  else {
    printf("time_2: no");
  }

}


