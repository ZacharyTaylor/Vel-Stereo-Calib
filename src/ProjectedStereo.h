#ifndef PROJECTEDSTEREO_H_
#define PROJECTEDSTEREO_H_

#include <opencv2/core/core.hpp>
#include <pcl/common/transforms.h>

#include "ProjectedPatch.h"
#include "StereoProperties.h"

/// Projects a point onto a pair of stereo images and compares the difference in image intensity at this location.
struct ProjectedStereo
{
  ProjectedPatch left; ///< Handles projection of the left image.
  ProjectedPatch right; ///< Handles projection of the right image.

  /// Constructor that projects a raw point
  /**
   * Builds a ProjectedPatch for each stereo image.
   *
   * @param raw_point raw_point raw 3D point.
   * @param image_left the left stereo image.
   * @param image_right the right stereo image.
   * @param patch_size for details see ProjectedPatch
   * @param stereo stereo camera model used for the projection.
   */
  ProjectedStereo(pcl::PointXYZI raw_point, const cv::Mat& image_left, const cv::Mat& image_right,
                  const size_t patch_size, const StereoProperties& stereo);

  /// Returns if a point is valid in both the left and right projection.
  bool valid(void) const;
  /// Returns the mean abs difference in the patch intensities between the left and right projection.
  float meanAbsDiff(void) const;
};

#endif /* PROJECTEDSTEREO_H_ */
