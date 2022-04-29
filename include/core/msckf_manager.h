#ifndef MSCKF_CORE_MANAGER_H
#define MSCKF_CORE_MANAGER_H
//c++
#include <memory>
#include <mutex>
#include <algorithm>
#include <iterator>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <tuple>
// customized
#include "core/state.h"
#include "types/type.h"
#include "utils/utils.h"
#include "core/imu_initializer.h"

namespace msckf_dvio
{

class MsckfManager {

public:
  MsckfManager(Params &parameters);

  void feedImu(const ImuMsg &data);

  void feedDvl(const DvlMsg &data);

  void feedCamera(const ImageMsg &data);

  void backend();

private:
  std::vector<ImuMsg> buffer_imu;
  std::vector<DvlMsg> buffer_dvl;
  std::vector<ImageMsg> buffer_img;

  std::mutex buffer_mutex;

  std::shared_ptr<State> state;

  std::shared_ptr<ImuInitializer> imu_initializer;

  // std::shared_ptr<Propagator> propagator;

  Params params;
};

} // namespace msckf_dvio


#endif // MSCKF_CORE_MANAGER_H

