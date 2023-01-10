#include "TrackFeature.h"

using namespace msckf_dvio;

//! TODO: only support mono camera features
void TrackFeature::feed_features(FeatureMsg &data) {
  // make sure all the data size are same
  assert(data.id.size == data.u.size());
  assert(data.u.size == data.v.size());

  // only support mono camera features
  int cam_id = 0;

  // Our good ids and points
  std::vector<cv::KeyPoint> good_left;
  std::vector<size_t> good_ids_left; 

  for(size_t i = 0; i < data.id.size(); i++ ) {
    // id
    size_t id = data.id[i] + currid;

    // Create the keypoint
    cv::KeyPoint kpt;
    kpt.pt.x = data.u[i];
    kpt.pt.y = data.v[i];

    // save
    good_left.push_back(kpt);
    good_ids_left.push_back(id);

    // Append to the database
    cv::Point2f npt_l = undistort_point(kpt.pt, cam_id);
    database->update_feature(id, data.time, cam_id, kpt.pt.x, kpt.pt.y, npt_l.x, npt_l.y);
  }

  // Get our width and height
  auto wh = camera_wh.at(cam_id);

  // Move forward in time
  img_last[cam_id] = cv::Mat::zeros(cv::Size(wh.first, wh.second), CV_8UC1);
  pts_last[cam_id] = good_left;
  ids_last[cam_id] = good_ids_left;
}