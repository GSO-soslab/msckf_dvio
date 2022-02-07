#ifndef MSCKF_CORE_MANAGER_H
#define MSCKF_CORE_MANAGER_H
//c++
#include <memory>
#include <mutex>
#include <algorithm>
#include <iterator>
#include <atomic>
#include <cmath>
#include <fstream>
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
  std::vector<ImuMsg> imu_buffer;
  std::vector<DvlMsg> dvl_buffer;
  std::vector<ImageMsg> img_buffer;

  std::mutex buffer_mutex;

  std::shared_ptr<State> state;

  std::shared_ptr<ImuInitializer> imu_initializer;

  // std::shared_ptr<Propagator> propagator;

  Params params;

  /***** Remap DVL timestamp, not a case of MSCKF-DVIO *****/
  bool mapDvlTime(const DvlMsg &in);

  std::vector<std::tuple<Eigen::Vector3d, double, double>> remap_queue;

  std::vector<DvlMsg> remapped_queue;

  double last_integral = 0.0;

  const char *file_path="/home/lin/Desktop/remap_dvl_time.dat";

  std::ofstream file;

  std::string last_flag ="#";
};

} // namespace msckf_dvio


#endif // MSCKF_CORE_MANAGER_H

