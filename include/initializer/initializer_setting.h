#ifndef MSCKF_INITIALIZER_SETTING_H_
#define MSCKF_INITIALIZER_SETTING_H_

#include "initializer.h"

namespace msckf_dvio {

class InitSetting : public Initializer
{

public:
  InitSetting(paramInit param_init);

  ~InitSetting(){}

  void checkInit() override;

  bool useSensor(const Sensor &sensor) override;

  void updateInit(std::shared_ptr<State> state, Params &params, std::map<Sensor, double> &data_time) override;

  void cleanBuffer() override;

private:

  // IMU

  // DVL

  // PRESSURE 

};

}

#endif //MSCKF_INITIALIZER_SETTING_H_
