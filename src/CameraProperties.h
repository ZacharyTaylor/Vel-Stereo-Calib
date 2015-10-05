#ifndef CAMERAPROPERTIES_H_
#define CAMERAPROPERTIES_H_

#include <pcl/common/transforms.h>

/// Holds all the relevant properties of a camera.
class CameraProperties
{
public:
  Eigen::Affine3f intrinsics; ///< Intrinsic camera matrix.
  Eigen::Affine3f tform; ///< Transformation matrix from world to camera coordinates.

  /// Constructor that builds the intrinsic matrix from its components.
  /**
   * @param fx focal length in x direction in pixels.
   * @param fy focal length in y direction in pixels.
   * @param cx x position of center of focus in pixels.
   * @param cy y position of center of focus in pixels.
   * @param tform_in transformation matrix describing camera's position with respect to the world coordinate system.
   */
  CameraProperties(float fx, float fy, float cx, float cy, Eigen::Affine3f tform_in = Eigen::Affine3f::Identity());

  /// Constructor that takes a predefined intrinsic camera matrix.
  /**
   * @param intrinsics_in camera matrix describing its intrinsics.
   * @param tform_in transformation matrix describing camera position with respect to the world coordinate system.
   */
  CameraProperties(Eigen::Affine3f intrinsics_in, Eigen::Affine3f tform_in = Eigen::Affine3f::Identity());

private:
  /// Builds an intrinsic matrix from its components
  /**
   *
   * @param fx focal length in x direction in pixels.
   * @param fy focal length in y direction in pixels.
   * @param cx x position of center of focus in pixels.
   * @param cy y position of center of focus in pixels.
   * @return intrinsic camera matrix
   */
  Eigen::Affine3f initIntrinsics(float fx, float fy, float cx, float cy);
};

#endif /* CAMERAPROPERTIES_H_ */
