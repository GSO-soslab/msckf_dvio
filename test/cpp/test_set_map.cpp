#include <map>
#include <set>
#include <iostream>
#include "boost/date_time/posix_time/posix_time.hpp"

struct Data{
  int id;
  double value;  

  bool operator==(const Data& rhs) const{
    return id == rhs.id;
  }

  bool operator<(const Data& rhs) const{
    return id < rhs.id;
  }


};

int main() {
  std::map<int,Data> data_map;
  std::set<Data> data_set;

  Data data; 
  boost::posix_time::ptime t1, t2;

  // init
  t1 = boost::posix_time::microsec_clock::local_time();

  int size = 100000;
  for(int i =0; i<size; i++) {
    data.id = i;
    data.value = i*0.5+12;

    data_map[i] = data;
    data_set.insert(data);
  }

  t2 = boost::posix_time::microsec_clock::local_time();

  printf("[For init]: %.4f seconds\n", (t2-t1).total_microseconds() * 1e-6);


  int index[10] = {1000, 10000, 20000, 30000, 40000, 50000, 60000, 70000, 80000, 90000};
  int count=0;

  // find map
  boost::posix_time::ptime t3, t4;

  t3 = boost::posix_time::microsec_clock::local_time();
  for(int i=0; i<100;i++) {
    if(data_map.find(55500) == data_map.end()){
      count++;
    }
  }
  t4 = boost::posix_time::microsec_clock::local_time();

  printf("[For map]: %.6f seconds\n", (t4-t3).total_microseconds() * 1e-6);


  // find set

}
