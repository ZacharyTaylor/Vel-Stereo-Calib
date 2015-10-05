#ifndef STEREOPROPERTIES_H_
#define STEREOPROPERTIES_H_

#include <pcl/common/transforms.h>

#include "CameraProperties.h"

/// Extends CameraProperties to incorporate the additional constraints of a stereo rig.
class StereoProperties : public CameraProperties
{
public:
  float baseline; ///< baseline between the two cameras in meters.

  /// Constructor that builds camera matrix from its components.
  /**
   * @param fx focal length in x direction in pixels.
   * @param fy focal length in y direction in pixels.
   * @param cx x position of center of focus in pixels.
   * @param cy y position of center of focus in pixels.
   * @param baseline_in baseline between cameras in meters
   * @param tform_in transformation matrix describing the __left__ camera's position with respect to the world coordinate system.
   */
  StereoProperties(float fx, float fy, float cx, float cy, float baseline_in, Eigen::Affine3f tform_in =
                       Eigen::Affine3f::Identity());

  /// Creates a CameraProperties object describing the left camera.
  CameraProperties getLeftCamera(void) const;
  /// Creates a CameraProperties object describing the right camera.
  CameraProperties getRightCamera(void) const;

};

#endif /* STEREOPROPERTIES_H_ */
