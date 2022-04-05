#ifndef MSCKF_CORE_PROPAGATOR_H
#define MSCKF_CORE_PROPAGATOR_H

#include "types/type.h"
#include "utils/utils.h"
#include <mutex>


namespace msckf_dvio
{

class Propagator {

public:
    Propagator() {}

    void feedImu(const ImuMsg &data);

private:
  std::vector<ImuMsg> buffer_imu;

  std::mutex buffer_mutex;

  Eigen::Vector3d gravity;

};

} // namespace msckf_dvio


#endif //MSCKF_CORE_PROPAGATOR_H