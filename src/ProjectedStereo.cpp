/*
 * ProjcetedStereo.cpp
 *
 *  Created on: 30 Sep 2015
 *      Author: z
 */

#include "ProjectedStereo.h"

ProjectedStereo::ProjectedStereo(pcl::PointXYZI raw_point, const cv::Mat& image_left, const cv::Mat& image_right,
                                 const size_t patch_size, const StereoProperties& stereo) :
    left {raw_point, image_left, patch_size, stereo.getLeftCamera()}, right {raw_point, image_right, patch_size,
                                                                             stereo.getRightCamera()}
{
}

bool ProjectedStereo::valid(void) const
{
  return this->left.valid && this->right.valid;
}

float ProjectedStereo::meanAbsDiff(void) const
{
  cv::Mat diff;
  cv::absdiff(left.patch, right.patch, diff);
  cv::Scalar m_val, s_val, m_val_l, s_val_l, m_val_r, s_val_r;

  cv::meanStdDev(left.patch, m_val_l, s_val_l);
  cv::meanStdDev(right.patch, m_val_r, s_val_r);
  cv::meanStdDev(diff, m_val, s_val);

  float s = s_val_l[0] + s_val_r[0];

  if (s < 1)
  {
    s = 1;
  }

  return m_val[0] / s;
}
