#ifndef MSCKF_INITIALIZER_BASE_H_
#define MSCKF_INITIALIZER_BASE_H_

#include <mutex>
#include <atomic>

#include "types/type_all.h"
#include "utils/utils.h"
#include "msckf/state.h"

namespace msckf_dvio {

class Initializer
{
public:
  Initializer(paramInit param_init_) 
   : initialized(false), param_init(param_init_) 
  {}

  virtual ~Initializer(){};

  inline bool isInit() { return initialized; }

  virtual void checkInit() = 0;

  /**
   * @brief update the initialization result to state
   * 
   * @param state: system state
   * @param params: parameters
   * @param data_time: initialized timestamp for each sensor  
  */
  virtual void updateInit(std::shared_ptr<State> state, Params &params, std::map<Sensor, double> &data_time) = 0;

  virtual void cleanBuffer() = 0;

  // check if this sensor will used in initialization
  virtual bool useSensor(const Sensor &sensor) = 0;

  void feedImu(const ImuMsg &data) {
    std::unique_lock<std::recursive_mutex> lck(buffer_mutex);

    buffer_imu.emplace_back(data);
  }

  void feedDvl(const DvlMsg &data) {
    std::unique_lock<std::recursive_mutex> lck(buffer_mutex);

    buffer_dvl.emplace_back(data);
  }

  void feedPressure(const PressureMsg &data) {
    std::unique_lock<std::recursive_mutex> lck(buffer_mutex);

    buffer_pressure.emplace_back(data);
  }

protected:

  std::atomic<bool> initialized;

  paramInit param_init;

  std::recursive_mutex buffer_mutex;

  std::vector<ImuMsg> buffer_imu;
  std::vector<DvlMsg> buffer_dvl;
  std::vector<PressureMsg> buffer_pressure;

};

} // namespace msckf_dvio

#endif  //MSCKF_INITIALIZER_BASE_H_