#pragma once

#include <vector>

#include <memory>

#include <Eigen/Core>

#ifdef WITH_OPENCV
#include <opencv2/core.hpp>
#endif

namespace junior_vio {
class CameraBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
  using Ptr = std::shared_ptr<CameraBase>;

  enum kCameraModel {
    kPinHole = 0,
    kKB8
  };

  CameraBase(uint8_t camera_id) : camera_id_(camera_id){}

  CameraBase(uint8_t camera_id, const std::vector<float> &parameters)
      : camera_id_(camera_id), parameters_(parameters) {}

  ~CameraBase() {}

  virtual Eigen::Vector2f project(const Eigen::Vector3f &point_3d) = 0;

  virtual Eigen::Matrix<float, 2, 3> projectJacobian(const Eigen::Vector3f &point_3d) = 0;

  virtual Eigen::Vector3f unproject(const Eigen::Vector2f &point_2d) = 0;

  virtual Eigen::Matrix<float, 3, 2> unprojectJacobian(const Eigen::Vector2f &point_2d) = 0;

  float getParameterByIndex(const size_t index) const { return parameters_[index]; }

  const std::vector<float> parameters() const { return parameters_; }

  void setParameterByIndex(const float value, const size_t index) { parameters_[index] = value; }

  size_t size() const { return parameters_.size(); }

  int8_t getCameraID() const { return camera_id_; }

  kCameraModel getCameraType() const { return camera_type_; }

#ifdef WITH_OPENCV
  virtual cv::Point2f project(const cv::Point3f &point_3d) = 0;

  virtual cv::Mat projectJacobian(const cv::Point3f &point_3d) = 0;

  virtual cv::Point3f unproject(const cv::Point2f &point_2d) = 0;

  virtual cv::Mat unprojectJacobian(const cv::Point2f &point_2d) = 0;
#endif

 protected:
  uint8_t camera_id_;

  kCameraModel camera_type_;
  
  std::vector<float> parameters_;
};
}  // namespace junior_vio
