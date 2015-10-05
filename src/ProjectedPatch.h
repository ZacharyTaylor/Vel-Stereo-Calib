#ifndef PROJECTEDPATCH_H_
#define PROJECTEDPATCH_H_

#include <opencv2/core/core.hpp>
#include <pcl/common/transforms.h>

#include "CameraProperties.h"

/// Projects a 3D point onto an image and stores the properties of interest.
struct ProjectedPatch
{
  bool valid; ///< True if the entire patch around the projected point is on the image, false otherwise. __If false all other variables and methods may give undefined behavior__
  cv::Point2f point; ///< Location of projected point in the image coordinate system.
  float intensity; ///< Intensity of the lidar's return.
  cv::Mat patch; ///< Intensity of the image in a patch around the point.

  /// Constructor that projects a raw point
  /**
   * @param raw_point raw 3D point.
   * @param image image to project point onto.
   * @param patch_size patch to take around the point (note this is the distance from the center, actual width/height is 2*patch_size + 1).
   * @param camera camera model to use in projecting the raw point onto the image.
   */
  ProjectedPatch(pcl::PointXYZI raw_point, const cv::Mat& image, const size_t& patch_size,
                 const CameraProperties& camera);
};

#endif /* PROJECTEDPATCH_H_ */
