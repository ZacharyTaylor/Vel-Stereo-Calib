#include "StereoProperties.h"

StereoProperties::StereoProperties(float fx, float fy, float cx, float cy, float baseline_in, Eigen::Affine3f tform_in) :
    CameraProperties(fx, fy, cx, cy, tform_in), baseline {baseline_in}
{
}

CameraProperties StereoProperties::getLeftCamera(void) const
{
  return CameraProperties(this->intrinsics, this->tform);
}

CameraProperties StereoProperties::getRightCamera(void) const
{
  Eigen::Affine3f tform_right {this->tform};
  tform_right = tform_right * Eigen::Translation3f(-this->baseline, 0.0, 0.0);
  return CameraProperties(this->intrinsics, tform_right);
}
