/*
 * ProjectedPatch.cpp
 *
 *  Created on: 30 Sep 2015
 *      Author: z
 */

#include "ProjectedPatch.h"

ProjectedPatch::ProjectedPatch(pcl::PointXYZI raw_point, const cv::Mat& image, const size_t& patch_size,
                               const CameraProperties& camera) :
    intensity {raw_point.intensity}
{
  raw_point = pcl::transformPoint(raw_point, camera.tform);

  if (!(this->valid = (raw_point.z > 0)))
    return;

  //project onto plane
  raw_point = pcl::transformPoint(raw_point, camera.intrinsics);
  this->point.x = raw_point.x / raw_point.z;
  this->point.y = raw_point.y / raw_point.z;

  //check if inside image
  if ((this->point.x < patch_size) || (this->point.y < patch_size)
      || (this->point.x >= (image.size().width - patch_size)) || (this->point.y >= (image.size().height - patch_size)))
  {
    this->valid = false;
    return;
  }

  //extract patch
  cv::Rect roi = cv::Rect(this->point.x - patch_size, this->point.y - patch_size, patch_size, patch_size);
  this->patch = image(roi);
}

