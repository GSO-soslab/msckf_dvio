
#ifndef OV_CORE_GRIDER_STC_H
#define OV_CORE_GRIDER_STC_H

#include <Eigen/Eigen>
#include <functional>
#include <iostream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "utils/lambda_body.h" 

namespace msckf_dvio {

/**
 * @brief Extracts shi-tomasi corner (STC) features in a grid pattern.
 *
 * As compared to just extracting shi-tomasi corner features over the entire image,
 * we want to have as uniform of extractions as possible over the image plane.
 * Thus we split the image into a bunch of small grids, and extract points in each.
 * We then pick enough top points in each grid so that we have the total number of desired points.
 */
class Grider_STC {

public:

  /**
   * @brief This function will perform grid extraction using FAST.
   * @param img Image we will do FAST extraction on
   * @param pts vector of extracted points we will return
   * @param num_features max number of features we want to extract
   * @param grid_x size of grid in the x-direction / u-direction
   * @param grid_y size of grid in the y-direction / v-direction
   * @param min_dist Minimum possible Euclidean distance of corners
   *
   * Given a specified grid size, this will try to extract fast features from each grid.
   * It will then return the best from each grid in the return vector.
   */
  static void perform_griding(const cv::Mat &img, std::vector<cv::KeyPoint> &pts, int num_features, 
                              int grid_x, int grid_y, int min_dist) {

    // Calculate the size our extraction boxes should be
    int size_x = img.cols / grid_x;
    int size_y = img.rows / grid_y;

    // Make sure our sizes are not zero
    assert(size_x > 0);
    assert(size_y > 0);

    // We want to have equally distributed features
    auto num_features_grid = (int)(num_features / (grid_x * grid_y)) + 1;

    // Parallelize our 2d grid extraction!!
    int ct_cols = std::floor(img.cols / size_x);
    int ct_rows = std::floor(img.rows / size_y);
    std::vector<std::vector<cv::KeyPoint>> collection(ct_cols * ct_rows);

    parallel_for_(cv::Range(0, ct_cols * ct_rows), LambdaBody([&](const cv::Range &range) {
                    for (int r = range.start; r < range.end; r++) {
                      // Calculate what cell xy value we are in
                      int x = r % ct_cols * size_x;
                      int y = r / ct_cols * size_y;

                      // Skip if we are out of bounds
                      if (x + size_x > img.cols || y + size_y > img.rows)
                        continue;

                      // Calculate where we should be extracting from
                      cv::Rect img_roi = cv::Rect(x, y, size_x, size_y);

                      // Extract shi-tomasi corner features for this part of the image
                      std::vector<cv::Point2f> pts_new;
                      cv::goodFeaturesToTrack(img(img_roi), pts_new, num_features_grid, 0.01, 
                                              min_dist, cv::Mat(), 7, false, 0.04);

                      // Append the pts to our vector
                      // Note that we need to "correct" the point u,v since we extracted it in a ROI
                      // So we should append the location of that ROI in the image
                      for (size_t i =0; i < pts_new.size(); i++) {
                        cv::KeyPoint pt_cor;
                        pt_cor.pt.x = pts_new.at(i).x + (float)x;
                        pt_cor.pt.y = pts_new.at(i).y + (float)y;
                        collection.at(r).push_back(pt_cor);
                      }
                    }
                  }));    

    // Combine all the collections into our single vector
    for (size_t r = 0; r < collection.size(); r++) {
      pts.insert(pts.end(), collection.at(r).begin(), collection.at(r).end());
    }

    // Return if no points
    if (pts.empty())
      return;

    // Sub-pixel refinement parameters
    cv::Size win_size = cv::Size(5, 5);
    cv::Size zero_zone = cv::Size(-1, -1);
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.001);

    // Get vector of points
    std::vector<cv::Point2f> pts_refined;
    for (size_t i = 0; i < pts.size(); i++) {
      pts_refined.push_back(pts.at(i).pt);
    }

    // Finally get sub-pixel for all extracted features
    cv::cornerSubPix(img, pts_refined, win_size, zero_zone, term_crit);

    // Save the refined points!
    for (size_t i = 0; i < pts.size(); i++) {
      pts.at(i).pt = pts_refined.at(i);
    }

  }

};

} // namesapce msckf_dvio

#endif /* OV_CORE_GRIDER_STC_H */
