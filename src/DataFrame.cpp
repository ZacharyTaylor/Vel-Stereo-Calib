#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/ply_io.h>

#include "DataFrame.h"

DataFrame::DataFrame(const std::string image_left_path, const std::string image_right_path,
                     const std::string vel_scan_path) :
    image_left {loadImage(image_left_path)}, image_right {loadImage(image_right_path)}, vel_scan {loadVelScan(
        vel_scan_path)}
{
}

cv::Mat DataFrame::loadImage(const std::string image_path)
{
  cv::Mat image {cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE)};
  if (!image.data)
    throw std::runtime_error("Could not open or find the left image");

  return image;
}

pcl::PointCloud<pcl::PointXYZI> DataFrame::loadVelScan(const std::string vel_scan_path)
{
  pcl::PointCloud<pcl::PointXYZI> vel_scan;
  if (pcl::io::loadPLYFile(vel_scan_path, vel_scan) < 0)
    throw std::runtime_error("Could not open or find the velodyne scan");

  return vel_scan;
}
