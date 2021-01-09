#pragma once

#include "AbstractCamera.h"

namespace junior_vio {
class PinholeCamera : public AbstractCamera {
    PinholeCamera(){
        parameters_.resize(4);
    }

};
}  // namespace junior_vio