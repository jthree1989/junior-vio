#pragma once

#include <assert.h>
#include "AbstractCamera.h"

namespace junior_vio {
class PinholeCamera : public AbstractCamera {
  PinholeCamera() {
    parameters_.resize(4);
    // -1 means invalid camera id, need to set id by user
    camera_id_ = -1;
    camera_type_ = kCameraModel::kPinHole;
  }

  PinholeCamera(const std::vector<float>& parameters)
      : AbstractCamera(parameters) {
    assert(parameters_.size() == 4);
    // -1 means invalid camera id, need to set id by user
    camera_id_ = -1;
    camera_type_ = kCameraModel::kPinHole;
  }
};
}  // namespace junior_vio