#pragma once

#include <assert.h>
#include "CameraBase.h"

namespace junior_vio {
class PinholeCamera : public CameraBase {
 public:
  using Ptr = std::shared_ptr<PinholeCamera>;

  explicit PinholeCamera(int8_t camera_id) 
  :CameraBase(camera_id){
    parameters_.resize(4);
    camera_type_ = kCameraModel::kPinHole;
  }

  PinholeCamera(int8_t camera_id, const std::vector<float>& parameters)
      : CameraBase(camera_id, parameters) {
    assert(parameters_.size() == 4);
    camera_type_ = kCameraModel::kPinHole;
  }

  virtual Eigen::Vector2f project(const Eigen::Vector3f &point_3d) override;

  virtual Eigen::Matrix<float, 2, 3> projectJacobian(const Eigen::Vector3f &point_3d) override;

  virtual Eigen::Vector3f unproject(const Eigen::Vector2f &point_2d) override;

  virtual Eigen::Matrix<float, 3, 2> unprojectJacobian(const Eigen::Vector2f &point_2d) override;


#ifdef WITH_OPENCV
  virtual cv::Point2f project(const cv::Point3f &point_3d) override;

  virtual cv::Mat projectJacobian(const cv::Point3f &point_3d) override;

  virtual cv::Point3f unproject(const cv::Point2f &point_2d) override;

  virtual cv::Mat unprojectJacobian(const cv::Point2f &point_2d) override;
#endif
};
}  // namespace junior_vio