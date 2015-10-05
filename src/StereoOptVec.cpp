/*
 * StereoOptVec.cpp
 *
 *  Created on: 3 Oct 2015
 *      Author: z
 */

#include "StereoOptVec.h"

StereoOptVec::StereoOptVec(const StereoProperties init, const bool opt_intrinsics) :
    StereoProperties {init}
{
  for (auto& i : this->extrinsic_flags)
    opt_flags[i] = true;

  if (opt_intrinsics)
    for (auto& i : this->intrinsic_flags)
      opt_flags[i] = true;
  else
    for (auto& i : this->intrinsic_flags)
      opt_flags[i] = false;
}

StereoOptVec::StereoOptVec(const StereoProperties init, const std::map<StereoOptFlag, bool> opt_flags_in) :
    StereoProperties {init}, opt_flags {opt_flags_in}
{
}

gsl_vector* StereoOptVec::getGSLVector()
{

  size_t vec_length {std::count_if(opt_flags.begin(), opt_flags.end(), [](std::pair<flag, bool> p)
  { return p.second;})};
  gsl_vector* vec {gsl_vector_alloc(vec_length)};

  std::map<flag, float> value_map {this->getValueMap()};

  size_t idx {0};
  for (auto& i : this->all_flags)
    if (opt_flags[i])
      gsl_vector_set(vec, idx++, value_map[i]);

  return vec;
}

void StereoOptVec::setUsingGSLVector(gsl_vector* vec)
{

  std::map<flag, float> value_map {this->getValueMap()};

  size_t idx {0};
  for (auto& i : this->all_flags)
    if (opt_flags[i])
    {
      if (idx >= vec->size)
        throw std::runtime_error("Insufficient GSL vector elements to set stereo elements");
      else
        value_map[i] = vec->data[idx++];
    }

  //normalize axis
  float r {std::sqrt(
      std::pow(value_map[flag::RX], 2) + std::pow(value_map[flag::RY], 2) + std::pow(value_map[flag::RZ], 2))};

  this->tform = Eigen::Affine3f::Identity();
  this->tform.translation() << value_map[flag::X], value_map[flag::Y], value_map[flag::Z];
  this->tform.rotate(
      Eigen::AngleAxisf(r, Eigen::Vector3f(value_map[flag::RX] / r, value_map[flag::RY] / r, value_map[flag::RZ] / r)));

  this->intrinsics(0, 0) = value_map[flag::FX];
  this->intrinsics(1, 1) = value_map[flag::FY];
  this->intrinsics(0, 2) = value_map[flag::CX];
  this->intrinsics(1, 2) = value_map[flag::CY];

}

std::map<StereoOptFlag, float> StereoOptVec::getValueMap(void)
{

  std::map<flag, float> map;
  Eigen::Affine3f tform {this->getLeftCamera().tform};

  Eigen::Vector3f tran {tform.translation()};
  map[flag::X] = tran[0];
  map[flag::Y] = tran[1];
  map[flag::Z] = tran[2];

  Eigen::AngleAxisf axis;
  axis = tform.rotation();
  map[flag::RX] = axis.axis()[0] * axis.angle();
  map[flag::RY] = axis.axis()[1] * axis.angle();
  map[flag::RZ] = axis.axis()[2] * axis.angle();

  Eigen::Matrix4f intrinsics {this->getLeftCamera().intrinsics.matrix()};
  map[flag::FX] = intrinsics(0, 0);
  map[flag::FY] = intrinsics(1, 1);
  map[flag::CX] = intrinsics(0, 2);
  map[flag::CY] = intrinsics(1, 2);

  map[flag::BASELINE] = this->baseline;

  return map;
}

