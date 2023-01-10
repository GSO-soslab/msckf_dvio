/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#ifndef OV_CORE_TRACK_FEATURE_H
#define OV_CORE_TRACK_FEATURE_H

#include "TrackBase.h"

namespace msckf_dvio {

/**
 * @brief Tracker to append uv measurements to database
 *
 * used for simulated data, or tracking processing did on different side
 */
class TrackFeature : public TrackBase {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
   */
  TrackFeature(int numaruco, std::map<size_t, std::pair<int, int>> &wh) : TrackBase(0, numaruco) {
    this->camera_wh = wh;
  }

  /// @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
  void feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) override {
    printf("[TrackFeature]: THIS feed_monocular SHOULD NEVER HAPPEN!\n");
    std::exit(EXIT_FAILURE);
  }

  /// @warning This function should not be used!! Use @ref feed_measurement_simulation() instead.
  void feed_stereo(double timestamp, cv::Mat &img_left, cv::Mat &img_right, size_t cam_id_left, size_t cam_id_right) override {
    printf("[TrackFeature]: THIS feed_stereo SHOULD NEVER HAPPEN!\n");
    std::exit(EXIT_FAILURE);
  }

  /**
   * @brief Feed function for given tracked features (either from simulation or another tracker)
   * @param data time, features that each one has u,v,id 
   */
  void feed_features(FeatureMsg &data) override;

protected:
  /// Width and height of our cameras
  std::map<size_t, std::pair<int, int>> camera_wh;
};

} // namespace msckf_dvio

#endif /* OV_CORE_TRACK_FEATURE_H */
