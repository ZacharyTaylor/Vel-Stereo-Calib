//============================================================================
// Name        : Vel-Stereo-Calib.cpp
// Author      : Zachary Taylor
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#define DEFAULT_BLOCK_SIZE 5
#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 768

bool invalidImageLoc(const std::pair<cv::Point2f, cv::Point2f> point)
{
  const unsigned int min_width = DEFAULT_BLOCK_SIZE;
  const unsigned int min_height = DEFAULT_BLOCK_SIZE;

  const unsigned int max_width = IMAGE_WIDTH - DEFAULT_BLOCK_SIZE;
  const unsigned int max_height = IMAGE_HEIGHT - DEFAULT_BLOCK_SIZE;

  if((point.first.x >= min_width) &&
      (point.first.y >= min_height) &&
      (point.first.x < max_width) &&
      (point.first.y < max_height) &&
      (point.second.x >= min_width) &&
      (point.second.y >= min_height) &&
      (point.second.x < max_width) &&
      (point.second.y < max_height))
  {
    return false;
  }
  else
  {
    return true;
  }
}

class VelStereoMatcher
{
  cv::Mat camera_intrinsics;
  float baseline;

  unsigned int block_size = DEFAULT_BLOCK_SIZE;

  std::vector<cv::Mat> left_images;
  std::vector<cv::Mat> right_images;

  std::vector<pcl::PointCloud<pcl::PointXYZ> > vel_scans;

  cv::Mat VecToTformMat(const std::vector<float>& r_vec, const std::vector<float>& t_vec)
  {
    Eigen::AngleAxis<float> aa(angle_in_radian, Vector3f(ax,ay,az));

    cv::Mat tform = cv::Mat::zeros(4, 4, CV_32F);
    cv::Rodrigues(r_vec, tform);

    for(size_t i = 0; i < 3; i++)
      tform.at<float>(i,3) = t_vec[i];
    tform.at<float>(3,3) = 1;

    return tform;
  }

  std::vector<cv::Point2f> ProjectScan(const std::vector<cv::Point3f>& scan,
                                        const std::vector<float>& r_vec,
                                        const std::vector<float>& t_vec)
  {
    auto tform = VecToTformMat(r_vec, t_vec);

    cv::Point3f a;



    return projected;
  }

public:

  void printProjectedSize(void)
  {
    auto projected = ProjectScan(vel_scans[0], {-1.5,0,0},{0,0,0});

    cv::Mat out = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_32F);
    for(auto &i : projected){
      size_t x = i.first.x;
      size_t y = i.first.y;
      out.at<float>(y,x) = 1;
    }
    std::cout << projected.size() << std::endl;

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", out );                   // Show our image inside it.

    cv::waitKey(0);
  }

  void setBlockSize(const unsigned int block_size_in)
  {
    block_size = block_size_in;
  }

  void setCameraParameters(const float baseline_in,
                           const float fx,
                           const float fy,
                           const float cx,
                           const float cy)
  {
    baseline = baseline_in;

    camera_intrinsics = cv::Mat::zeros(3, 3, CV_32F);
    camera_intrinsics.at<float>(0, 0) = fx;
    camera_intrinsics.at<float>(1, 1) = fy;
    camera_intrinsics.at<float>(0, 2) = cx;
    camera_intrinsics.at<float>(1, 2) = cy;
    camera_intrinsics.at<float>(2, 2) = 1;
  }

  int addFrame(const std::string left_image_path,
               const std::string right_image_path,
               const std::string vel_scan_path)
  {
    //load data
    std::cout << left_image_path << std::endl;
    cv::Mat left_image = cv::imread(left_image_path, CV_LOAD_IMAGE_GRAYSCALE);
    if(!left_image.data)
    {
      std::cout << "Could not open or find the left image" << std::endl;
      return -1;
    }

    cv::Mat right_image = cv::imread(right_image_path, CV_LOAD_IMAGE_GRAYSCALE);
    if(!right_image.data)
    {
      std::cout << "Could not open or find the right image" << std::endl;
      return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> vel_scan;
    if (pcl::io::loadPLYFile(vel_scan_path, vel_scan) < 0)
    {
      std::cout << "Could not open or find the velodyne scan" << std::endl;
      return -1;
    }

    //store data
    left_images.push_back(left_image);
    right_images.push_back(right_image);
    vel_scans.push_back(vel_scan);

    return 0;
  }

};

int main(int argc, char* argv[])
{
  VelStereoMatcher matcher;
  float baseline = 0.119658, fx = 1024*0.436783, fy = 768*0.582378, cx = 1024*0.502834, cy = 768*0.527529;
  matcher.setCameraParameters(baseline,fx,fy,cx,cy);
  matcher.addFrame(argv[1],argv[2],argv[3]);
  matcher.printProjectedSize();

}
