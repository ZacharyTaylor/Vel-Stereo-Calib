//============================================================================
// Name        : Vel-Stereo-Calib.cpp
// Author      : Zachary Taylor
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "VelStereoMatcher.h"


int main(int argc, char* argv[])
{
  VelStereoMatcher matcher;
  matcher.addFrame(argv[1], argv[2], argv[3]);

  Eigen::Affine3f tform = Eigen::Affine3f::Identity();
  tform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
  tform.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));
  tform.rotate(Eigen::AngleAxisf(-0.785, Eigen::Vector3f::UnitZ()));
  tform.translation() << 0.21, -0.41, 0.06;

  //StereoOptVec vec(450,450,512,384,tform);

  //auto v = matcher.findOptimalStereo(vec);
  //auto v = matcher.evalTform(tform, true);
  //std::cout << v << std::endl;

}
