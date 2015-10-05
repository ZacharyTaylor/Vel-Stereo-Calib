#ifndef DATAFRAME_H_
#define DATAFRAME_H_

#include <opencv2/core/core.hpp>
#include <pcl/common/transforms.h>

/// Loads and holds pre-synchronized stereo and Velodyne scan data.
class DataFrame
{
public:

  cv::Mat image_left; ///< image captured by the left stereo camera.
  cv::Mat image_right; ///< image captured by the right stereo camera.
  pcl::PointCloud<pcl::PointXYZI> vel_scan; ///< Velodyne scan (pre-synchronized to coincide with stereo images).

  /// Loads one frame of information from given files.
  /**
   * @param image_left_path path to left stereo image.
   * @param image_right_path path to right stereo image.
   * @param vel_scan_path path to Velodyne scan (currently only accepts .ply files with X,Y,Z and intensity fields).
   */
  DataFrame(const std::string image_left_path, const std::string image_right_path, const std::string vel_scan_path);

private:

  /// Loads the specified image.
  /**
   * @param image_path path to image.
   * @return loaded image.
   */
  cv::Mat static loadImage(const std::string image_path);

  /// Loads the specified Velodyne point cloud.
  /**
   * @param vel_scan_path path to Velodyne scan (currently only accepts .ply files with X,Y,Z and intensity fields).
   * @return loaded scan.
   */
  pcl::PointCloud<pcl::PointXYZI> static loadVelScan(const std::string vel_scan_path);
};

#endif /* DATAFRAME_H_ */
