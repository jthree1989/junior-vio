#pragma once

#include <vector>

#include <Eigen/Core>

#ifdef WITH_OPENCV
#include <opencv2/core.hpp>
#endif

namespace junior_vio {
class AbstractCamera {
 public:
  AbstractCamera() {}

  AbstractCamera(const std::vector<float> &parameters)
      : parameters_(parameters) {}

  ~AbstractCamera() {}

  virtual Eigen::Vector2f project(const Eigen::Vector3f &point_3d) = 0;

#ifdef WITH_OPENCV
  virtual cv::Point2f project(const cv::Point3f &point_3d) = 0;

  virtual cv::Point3f unproject(const cv::Point2f &point_2d) = 0;
#endif

  static uint8_t camera_id_counter_;

 protected:
  std::vector<float> parameters_;

  uint8_t camera_id_;

  uint8_t camera_type_;
};
}  // namespace junior_vio
