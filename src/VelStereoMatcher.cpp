/*
 * VelStereoMatcher.cpp
 *
 *  Created on: 30 Sep 2015
 *      Author: z
 */

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "VelStereoMatcher.h"

VelStereoMatcher::VelStereoMatcher(size_t patch_size_in) :
    patch_size(patch_size_in)
{
}

void VelStereoMatcher::addFrame(const std::string image_left_path, const std::string image_right_path,
                                const std::string vel_scan_path)
{
  frames.push_back(DataFrame(image_left_path, image_right_path, vel_scan_path));
}

float VelStereoMatcher::evalStereo(const StereoProperties& stereo, bool plot)
{
  std::vector<std::vector<ProjectedStereo> > proj_stereo_frames;

  for (auto &frame : this->frames)
  {
    std::vector<ProjectedStereo> proj_stereo_frame;
    for (auto &point : frame.vel_scan)
    {
      proj_stereo_frame.push_back(
          ProjectedStereo(point, frame.image_left, frame.image_right, this->patch_size, stereo));
    }

    proj_stereo_frames.push_back(proj_stereo_frame);

    if (plot && proj_stereo_frames.size() == 1)
    {
      PlotProject(proj_stereo_frame, frame.image_left, frame.image_right);
    }

  }
  return evalStereoError(proj_stereo_frames);
}

double VelStereoMatcher::optFuncWrapper(const gsl_vector *v, void *params)
{
  //evaluate
  VelStereoMatcher* curr = static_cast<VelStereoMatcher*>(params);
  return curr->evalStereo(vecToStereo(v));
}

gsl_vector* VelStereoMatcher::stereoToVec(const StereoProperties stereo)
{
  Eigen::Affine3f tform = stereo.getLeftCamera().tform;
  Eigen::Matrix4f intrinsics = stereo.getLeftCamera().intrinsics.matrix();

  Eigen::Vector3f tran;
  tran = tform.translation();
  float x = tran[0];
  float y = tran[1];
  float z = tran[2];

  Eigen::Matrix3f mat = tform.rotation();
  Eigen::AngleAxisf axis;
  axis = mat;
  float ax = axis.axis()[0] * axis.angle();
  float ay = axis.axis()[1] * axis.angle();
  float az = axis.axis()[2] * axis.angle();

  float fx = intrinsics(0,0);
  float fy = intrinsics(1,1);
  float cx = intrinsics(0,2);
  float cy = intrinsics(1,2);

  float baseline = stereo.baseline;

  gsl_vector* vec = gsl_vector_alloc(11);
  gsl_vector_set(vec, 0, x);
  gsl_vector_set(vec, 1, y);
  gsl_vector_set(vec, 2, z);
  gsl_vector_set(vec, 3, ax);
  gsl_vector_set(vec, 4, ay);
  gsl_vector_set(vec, 5, az);

  gsl_vector_set(vec, 6, fx);
  gsl_vector_set(vec, 7, fy);
  gsl_vector_set(vec, 8, cx);
  gsl_vector_set(vec, 9, cy);
  gsl_vector_set(vec, 10, baseline);

  return vec;

}

StereoProperties VelStereoMatcher::vecToStereo(const gsl_vector* vec)
{

  float x = gsl_vector_get(vec, 0);
  float y = gsl_vector_get(vec, 1);
  float z = gsl_vector_get(vec, 2);
  float ax = gsl_vector_get(vec, 3);
  float ay = gsl_vector_get(vec, 4);
  float az = gsl_vector_get(vec, 5);
  float r = std::sqrt(ax * ax + ay * ay + az * az);

  float fx = gsl_vector_get(vec, 6);
  float fy = gsl_vector_get(vec, 7);
  float cx = gsl_vector_get(vec, 8);
  float cy = gsl_vector_get(vec, 9);
  float baseline = gsl_vector_get(vec, 10);

  //normalize axis
  ax /= r;
  ay /= r;
  az /= r;

  //create tform
  Eigen::Affine3f tform = Eigen::Affine3f::Identity();
  tform.translation() << x, y, z;
  tform.rotate(Eigen::AngleAxisf(r, Eigen::Vector3f(ax, ay, az)));

  //create stereo
  return StereoProperties(fx,fy,cx,cy,baseline,tform);
}

int VelStereoMatcher::findOptimalStereo(StereoProperties init)
{

  const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
  gsl_multimin_fminimizer *s = NULL;
  gsl_vector *ss, *x;
  gsl_multimin_function minex_func;

  size_t iter = 0;
  int status;
  double size;

  // Starting point
  x = stereoToVec(init);

  // Set initial step sizes to 1
  ss = gsl_vector_alloc(11);
  gsl_vector_set(ss, 0, 0.1);
  gsl_vector_set(ss, 1, 0.1);
  gsl_vector_set(ss, 2, 0.1);
  gsl_vector_set(ss, 3, 0.05);
  gsl_vector_set(ss, 4, 0.05);
  gsl_vector_set(ss, 5, 0.05);
  gsl_vector_set(ss, 6, 5);
  gsl_vector_set(ss, 7, 5);
  gsl_vector_set(ss, 8, 5);
  gsl_vector_set(ss, 9, 5);
  gsl_vector_set(ss, 10, 0.001);

  // Initialize method and iterate
  minex_func.n = 11;
  minex_func.f = this->optFuncWrapper;
  minex_func.params = this;

  s = gsl_multimin_fminimizer_alloc(T, 11);
  gsl_multimin_fminimizer_set(s, &minex_func, x, ss);

  do
  {
    iter++;
    status = gsl_multimin_fminimizer_iterate(s);

    if (status)
      break;

    size = gsl_multimin_fminimizer_size(s);
    status = gsl_multimin_test_size(size, 1e-4);

    if (status == GSL_SUCCESS)
    {
      printf("converged to minimum at\n");
    }

    printf("%5d x:%2.3f y:%2.3f z:%2.3f rx:%2.3f ry:%2.3f rz:%2.3f fx:%3.1f fy:%3.1f cx:%3.1f cy:%3.1f baseline:%2.3f f() = %7.3f size = %.3f\n", iter,
           gsl_vector_get(s->x, 0), gsl_vector_get(s->x, 1), gsl_vector_get(s->x, 2), gsl_vector_get(s->x, 3),
           gsl_vector_get(s->x, 4), gsl_vector_get(s->x, 5), gsl_vector_get(s->x, 6), gsl_vector_get(s->x, 7), gsl_vector_get(s->x, 8), gsl_vector_get(s->x, 9), gsl_vector_get(s->x, 10), s->fval, size);
  } while (status == GSL_CONTINUE && iter < 100);

  gsl_vector_free(x);
  gsl_vector_free(ss);
  gsl_multimin_fminimizer_free(s);

  return status;
}

void VelStereoMatcher::PlotProject(std::vector<ProjectedStereo> projected, cv::Mat image_left, cv::Mat image_right)
{
  cv::Mat vel_image_left(image_left.size(), image_left.type());
  vel_image_left.setTo(0);
  cv::Mat vel_image_right(image_right.size(), image_right.type());
  vel_image_right.setTo(0);

  size_t j = 0;
  for (auto &i : projected)
  {

    if (i.left.valid)
    {
      j++;
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
  image_left += 2 * vel_image_left;

  image_right /= 2;
  image_right += 2 * vel_image_right;

  cv::namedWindow("left window", cv::WINDOW_AUTOSIZE);
  cv::imshow("left window", image_left);
  cv::namedWindow("right window", cv::WINDOW_AUTOSIZE);
  cv::imshow("right window", image_right);

  cv::waitKey(0);
}

float VelStereoMatcher::evalStereoError(const std::vector<std::vector<ProjectedStereo> >& proj_stereo)
{
  float err;
  size_t len = 0;
  for (auto &points : proj_stereo)
  {
    for (auto &point : points)
    {
      if (point.valid())
      {
        err += (point.meanAbsDiff());
        ++len;
      }
    }
  }
  if (len < 25000)
  {
    return err;
  }
  //std::cout << len << std::endl;
  return err/len;
}

