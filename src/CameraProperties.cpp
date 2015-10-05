#include "CameraProperties.h"

CameraProperties::CameraProperties(float fx, float fy, float cx, float cy, Eigen::Affine3f tform_in) :
    intrinsics {initIntrinsics(fx, fy, cx, cy)}, tform {tform_in}
{
}

CameraProperties::CameraProperties(Eigen::Affine3f intrinsics_in, Eigen::Affine3f tform_in) :
    intrinsics{intrinsics_in}, tform{tform_in}
{
}

Eigen::Affine3f CameraProperties::initIntrinsics(float fx, float fy, float cx, float cy)
{
  Eigen::Affine3f mat {Eigen::Affine3f::Identity()};

  mat(0, 0) = fx;
  mat(1, 1) = fy;
  mat(0, 2) = cx;
  mat(1, 2) = cy;

  return mat;
}
