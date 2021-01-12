#pragma once

#include "CameraBase.h"

namespace junior_vio {
class KanalaBrandt8Camera : public CameraBase {
  /* 
   * KanalaBrandt model is used in OpenCV fisheye module.
   * https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
   */
 public:
  explicit KanalaBrandt8Camera(uint8_t camera_id, const float precision = 1.0e-6)
      : CameraBase(camera_id), precision_(precision) {
    parameters_.resize(8);
    camera_type_ = kKB8;
  }

  explicit KanalaBrandt8Camera(uint8_t camera_id, const std::vector<float>& parameters, const float precision = 1.0e-6)
      : CameraBase(camera_id, parameters), precision_(precision) {
    assert(parameters_.size() == 8);
    camera_type_ = kKB8;
  }

  explicit KanalaBrandt8Camera(uint8_t camera_id, KanalaBrandt8Camera kb_camera)
    : CameraBase(camera_id), precision_(kb_camera.precision()){
    assert(kb_camera.getCameraType() == kKB8);
    assert(kb_camera.parameters().size() == 8);
    
    parameters_ = kb_camera.parameters();
    camera_type_ = kb_camera.getCameraType();
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

  float precision() const { return precision_; }
private:
  const float precision_;
};
}  // namespace junior_vio