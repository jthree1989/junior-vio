#include "KanalaBrandt8Camera.h"
#include <cmath>

using namespace junior_vio;

Eigen::Vector2f KanalaBrandt8Camera::project(const Eigen::Vector3f &point_3d) {
  // 1. Set meaningful name for parameters_: [fx, fy, cx, cy, k1 ,k2, k3, k4]
  float fx = parameters_[0], fy = parameters_[1], cx = parameters_[2],
        cy = parameters_[3];
  float k1 = parameters_[4], k2 = parameters_[5], k3 = parameters_[6],
        k4 = parameters_[7];
  // 2. Normalize 3D point and add distortion to it
  float a = point_3d(0), b = point_3d(1);
  float r = sqrtf(a * a + b * b);
  float theta = atan2f(r, point_3d(2));
  float theta2 = theta * theta, theta4 = theta2 * theta2,
        theta6 = theta2 * theta4, theta8 = theta4 * theta4;
  float distorted_theta =
      theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
  assert(r > precision_);
  float distorted_x = (distorted_theta / r) * a,
        distorted_y = (distorted_theta / r) * b;
  // 3. Project to image coordinate
  Eigen::Vector2f uv = {fx * distorted_x + cx, fy * distorted_y + cy};

  return uv;
}

Eigen::Matrix<float, 2, 3> KanalaBrandt8Camera::projectJacobian(const Eigen::Vector3f &point_3d){
  // 1. Set meaningful name for parameters_: [fx, fy, cx, cy, k1 ,k2, k3, k4]
  float fx = parameters_[0], fy = parameters_[1];
  float k1 = parameters_[4], k2 = parameters_[5], k3 = parameters_[6],
        k4 = parameters_[7];
  float x = point_3d(0) , y = point_3d(1), z = point_3d(2), x2 = x * x, y2 = y * y, z2 = z * z;
  float r = sqrtf(x2 + y2), r2 = r * r, r3 = r2 * r;
  float theta = atan2f(r, z);
  float theta2 = theta * theta, theta3 = theta2 * theta, 
        theta4 = theta2 * theta2, theta5 = theta4 * theta,
        theta6 = theta2 * theta4, theta7 = theta6 * theta, 
        theta8 = theta4 * theta4, theta9 = theta8 * theta;

  float f = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
  float df_dtheta = 1 + 3 * k1 * theta2 + 5 * k2 * theta4 + 7 * k3 * theta6 + 9 * k4 * theta8;

  Eigen::Matrix<float, 2, 3> project_jac;
  project_jac(0, 0) = fx * (df_dtheta * z * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
  project_jac(0, 1) = fx * (df_dtheta * x * y * z / (r2 * (r2 + z2)) - f * x * y / r3);
  project_jac(0, 2) = -fx * df_dtheta * x / (r2 + z2);
  project_jac(1, 0) = fy * (df_dtheta * x * y * z / (r2 * (r2 + z2)) - f * x * y / r3);
  project_jac(1, 1) = fy * (df_dtheta * z * y2 / (r2 * (r2 + z2)) + f * x2 / r3);
  project_jac(1, 2) = -fy * df_dtheta * y / (r2 + z2);

  return project_jac;
}