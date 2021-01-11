#include "PinholeCamera.h"

using namespace junior_vio;

Eigen::Vector2f PinholeCamera::project(const Eigen::Vector3f &point_3d) {
  // parameters_: [fx, fy, cx, cy]
  Eigen::Vector2f point_2d = {
      parameters_[0] * point_3d(0) / point_3d(2) + parameters_[2],
      parameters_[1] * point_3d(0) / point_3d(2) + parameters_[3]};

  return point_2d;
}

Eigen::Matrix<float, 2, 3> PinholeCamera::projectJacobian(
    const Eigen::Vector3f &point_3d) {
  // parameters_: [fx, fy, cx, cy]
  float fx = parameters_[0], fy = parameters_[1];

  float X = point_3d(0), Y = point_3d(1), Z = point_3d(2);

  Eigen::Matrix<float, 2, 3> project_jac = Eigen::Matrix<float, 2, 3>::Zero(2, 3);
  project_jac(0, 0) = fx / Z;
  project_jac(0, 2) = -fx * X / (Z * Z);
  project_jac(1, 1) = fy / Z;
  project_jac(1, 2) = -fy * Y / (Z * Z);

  return project_jac;
}

Eigen::Vector3f PinholeCamera::unproject(const Eigen::Vector2f &point_2d) {
  // parameters_: [fx, fy, cx, cy]
  Eigen::Vector3f point_3d = {(point_2d(0) - parameters_[2]) / parameters_[0],
                              (point_2d(1) - parameters_[3]) / parameters_[1],
                              1.0f};

  return point_3d;
}

Eigen::Matrix<float, 3, 2> PinholeCamera::unprojectJacobian(
    const Eigen::Vector2f &point_2d) {
  // parameters_: [fx, fy, cx, cy]
  float fx = parameters_[0], fy = parameters_[1];

  Eigen::Matrix<float, 3, 2> unproject_jac = Eigen::Matrix<float, 3, 2>::Zero(3, 2);

  unproject_jac(0, 0) = 1.0f / fx;
  unproject_jac(1, 1) = 1.0f / fy;

  return unproject_jac;
}

#ifdef WITH_OPENCV
cv::Point2f PinholeCamera::project(const cv::Point3f &point_3d) {
  // parameters_: [fx, fy, cx, cy]
  cv::Point2f point_2d = {
      parameters_[0] * point_3d.x / point_3d.z + parameters_[2],
      parameters_[1] * point_3d.y / point_3d.z + parameters_[3]};

  return point_2d;
}

cv::Mat PinholeCamera::projectJacobian(const cv::Point3f &point_3d){
    // parameters_: [fx, fy, cx, cy]
  float fx = parameters_[0], fy = parameters_[1];

  float X = point_3d.x, Y = point_3d.y, Z = point_3d.z;
  
  cv::Mat project_jac(2, 3, CV_32F, 0.0f);

  project_jac.at<float>(0, 0) = fx / Z;  
  project_jac.at<float>(0, 2) = -fx * X / (Z * Z);
  project_jac.at<float>(1, 1) = fy / Z;
  project_jac.at<float>(1, 2) = -fy * Y / (Z * Z); 

  return project_jac;

}

cv::Point3f PinholeCamera::unproject(const cv::Point2f &point_2d) {
  // parameters_: [fx, fy, cx, cy]
  cv::Point3f point_3d = {(point_2d.x - parameters_[2]) / parameters_[0],
                          (point_2d.y - parameters_[3]) / parameters_[1], 1.0f};

  return point_3d;
}

cv::Mat PinholeCamera::unprojectJacobian(const cv::Point2f &point_2d){
  // parameters_: [fx, fy, cx, cy]
  float fx = parameters_[0], fy = parameters_[1];

  cv::Mat unproject_jac(3, 2, CV_32F, 0.0f);

  unproject_jac.at<float>(0, 0) = 1.0f / fx;
  unproject_jac.at<float>(1, 1) = 1.0f / fy;

  return unproject_jac;
}
#endif