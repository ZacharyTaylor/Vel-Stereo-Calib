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
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#define DEFAULT_PATCH_SIZE 5
#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 768

struct CameraProperties
{
  const Eigen::Affine3f intrinsics;
  Eigen::Affine3f tform;

  CameraProperties(float fx, float fy, float cx, float cy, Eigen::Affine3f tform_in = Eigen::Affine3f::Identity()) :
      intrinsics(initIntrinsics(fx, fy, cx, cy)), tform(tform_in)
  {
  }

  CameraProperties(Eigen::Affine3f intrinsics_in, Eigen::Affine3f tform_in = Eigen::Affine3f::Identity()) :
      intrinsics(intrinsics_in), tform(tform_in)
  {
  }

private:
  Eigen::Affine3f initIntrinsics(float fx, float fy, float cx, float cy)
  {
    Eigen::Affine3f mat = Eigen::Affine3f::Identity();

    mat(0, 0) = fx;
    mat(1, 1) = fy;
    mat(0, 2) = cx;
    mat(1, 2) = cy;

    return mat;
  }
};

struct StereoProperties : private CameraProperties
{
  float baseline;

  CameraProperties get_left_camera(void) const
  {
    return CameraProperties(this->intrinsics, this->tform);
  }

  CameraProperties get_right_camera(void) const
  {
    auto tform_right = this->tform;
    tform_right.translation() << -this->baseline, 0.0, 0.0;
    return CameraProperties(this->intrinsics, tform_right);
  }

  void setLeftTform(const Eigen::Affine3f& tform_in)
  {
    tform = tform_in;
  }

  StereoProperties(float fx, float fy, float cx, float cy, float baseline_in, Eigen::Affine3f tform_in =
                       Eigen::Affine3f::Identity()) :
      CameraProperties(fx, fy, cx, cy, tform_in), baseline(baseline_in)
  {
  }
};

struct ProjectedPatch
{
  cv::Point2f point;
  float intensity;
  cv::Mat patch;
  bool valid;

  ProjectedPatch(pcl::PointXYZI raw_point, const cv::Mat& image, const size_t patch_size, CameraProperties camera)
  {

    raw_point = pcl::transformPoint(raw_point, camera.tform);

    this->intensity = raw_point.intensity;

    if (!(this->valid = (raw_point.z > 0)))
      return;

    //project onto plane
    raw_point = pcl::transformPoint(raw_point, camera.intrinsics);
    this->point.x = raw_point.x / raw_point.z;
    this->point.y = raw_point.y / raw_point.z;

    //check if inside image
    if ((this->point.x < patch_size) || (this->point.y < patch_size)
        || (this->point.x >= (image.size().width - patch_size))
        || (this->point.y >= (image.size().height - patch_size)))
    {
      this->valid = false;
      return;
    }

    //extract patch
    cv::Rect roi = cv::Rect(this->point.x - patch_size, this->point.y - patch_size, patch_size, patch_size);
    this->patch = image(roi);
  }

};

struct ProjectedStereo
{
  ProjectedPatch left;
  ProjectedPatch right;

  ProjectedStereo(pcl::PointXYZI raw_point, const cv::Mat& image_left, const cv::Mat& image_right,
                  const size_t patch_size, const StereoProperties& stereo) :
      left(raw_point, image_left, patch_size, stereo.get_left_camera()), right(raw_point, image_right, patch_size,
                                                                               stereo.get_right_camera())
  {
  }

  bool valid(void) const
  {
    return this->left.valid && this->right.valid;
  }

  cv::Scalar meanAbsDiff(void) const
  {
    cv::Mat diff;
    cv::absdiff(left.patch, right.patch, diff);
    return cv::mean(diff);
  }
};

struct DataFrame
{
  cv::Mat image_left;
  cv::Mat image_right;
  pcl::PointCloud<pcl::PointXYZI> vel_scan;

  DataFrame(const std::string image_left_path, const std::string image_right_path, const std::string vel_scan_path) :
      image_left(loadImage(image_left_path)), image_right(loadImage(image_right_path)), vel_scan(
          loadVelScan(vel_scan_path))
  {
  }

private:

  cv::Mat loadImage(const std::string image_path)
  {
    auto image = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);
    if (!image.data)
    {
      throw std::runtime_error("Could not open or find the left image");
    }

    return image;
  }

  pcl::PointCloud<pcl::PointXYZI> loadVelScan(const std::string vel_scan_path)
  {
    pcl::PointCloud<pcl::PointXYZI> vel_scan;
    if (pcl::io::loadPLYFile(vel_scan_path, vel_scan) < 0)
    {
      throw std::runtime_error("Could not open or find the velodyne scan");
    }

    return vel_scan;
  }
};

class VelStereoMatcher
{

  cv::Scalar evalStereoError(const std::vector<std::vector<ProjectedStereo> >& proj_stereo)
  {
    cv::Scalar err;
    size_t len = 0;
    for (auto &points : proj_stereo)
    {
      for (auto &point : points)
      {
        if (point.valid())
        {
          err += point.meanAbsDiff();
          ++len;
        }
      }
    }

    cv::Scalar result;
    cv::divide(err, len, result);

    return result;
  }

public:

  StereoProperties stereo_prop;
  unsigned int patch_size;
  std::vector<DataFrame> frames;

  VelStereoMatcher(float fx, float fy, float cx, float cy, float baseline, size_t patch_size_in = DEFAULT_PATCH_SIZE) :
      stereo_prop(fx, fy, cx, cy, baseline), patch_size(patch_size_in)
  {
  }

  void addFrame(const std::string image_left_path, const std::string image_right_path, const std::string vel_scan_path)
  {
    frames.push_back(DataFrame(image_left_path,image_right_path,vel_scan_path));
  }

  cv::Scalar evalTform(const Eigen::Affine3f& tform, bool plot = false)
  {
    this->stereo_prop.setLeftTform(tform);
    std::vector<std::vector<ProjectedStereo> > proj_stereo_frames;

    for (auto &frame : this->frames)
    {
      std::vector<ProjectedStereo> proj_stereo_frame;
      for (auto &point : frame.vel_scan)
      {
        proj_stereo_frame.push_back(
            ProjectedStereo(point, frame.image_left, frame.image_right, this->patch_size, this->stereo_prop));
      }

      proj_stereo_frames.push_back(proj_stereo_frame);

      if (plot && proj_stereo_frames.size() == 1)
      {
        PlotProject(proj_stereo_frame, frame.image_left, frame.image_right);
      }

    }
    return evalStereoError(proj_stereo_frames);
  }

  void PlotProject(std::vector<ProjectedStereo> projected, cv::Mat image_left, cv::Mat image_right)
  {
    cv::Mat vel_image_left(image_left.size(), image_left.type());
    vel_image_left.setTo(0);
    cv::Mat vel_image_right(image_right.size(), image_right.type());
    vel_image_right.setTo(0);

    for (auto &i : projected)
    {
      if (i.left.valid)
      {
        size_t x = i.left.point.x;
        size_t y = i.left.point.y;
        uchar v = i.left.intensity;
        vel_image_left.at<uchar>(y, x) = v;
      }
      if (i.right.valid)
      {
        size_t x = i.right.point.x;
        size_t y = i.right.point.y;
        uchar v = i.right.intensity;
        vel_image_right.at<uchar>(y, x) = v;
      }
    }

    cv::dilate(vel_image_left, vel_image_left, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    cv::dilate(vel_image_right, vel_image_right, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

    image_left /= 2;
    image_left += 2*vel_image_left;

    image_right /= 2;
    image_right += 2*vel_image_right;

    cv::namedWindow("left window", cv::WINDOW_AUTOSIZE);
    cv::imshow("left window", image_left);
    cv::namedWindow("right window", cv::WINDOW_AUTOSIZE);
    cv::imshow("right window", image_right);

    cv::waitKey(0);
  }

  void printProjectedSize(void)
  {
    Eigen::Affine3f tform = Eigen::Affine3f::Identity();
    tform.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
    tform.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));
    tform.rotate(Eigen::AngleAxisf(-0.9, Eigen::Vector3f::UnitZ()));

    evalTform(tform, true);
  }
};

int main(int argc, char* argv[])
{
  float baseline = 0.119658, fx = 1024 * 0.436783, fy = 768 * 0.582378, cx = 1024 * 0.502834, cy = 768 * 0.527529;
  VelStereoMatcher matcher(fx, fy, cx, cy, baseline);
  matcher.addFrame(argv[1], argv[2], argv[3]);
  matcher.printProjectedSize();

}
