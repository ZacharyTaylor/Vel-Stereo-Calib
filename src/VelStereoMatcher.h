/*
 * VelStereoMatcher.h
 *
 *  Created on: 30 Sep 2015
 *      Author: z
 */

#ifndef VELSTEREOMATCHER_H_
#define VELSTEREOMATCHER_H_

#define DEFAULT_PATCH_SIZE 5

#include <string>

#include <gsl/gsl_multimin.h>
#include <pcl/common/transforms.h>
#include <opencv2/core/core.hpp>

#include "ProjectedStereo.h"
#include "DataFrame.h"


class VelStereoMatcher
{

public:

  //StereoProperties stereo_prop;
  unsigned int patch_size;
  std::vector<DataFrame> frames;

  VelStereoMatcher(size_t patch_size_in = DEFAULT_PATCH_SIZE);

  void addFrame(const std::string image_left_path, const std::string image_right_path, const std::string vel_scan_path);
  float evalStereo(const StereoProperties& stereo, bool plot = false);

  static gsl_vector* stereoToVec(const StereoProperties stereo);
  static StereoProperties vecToStereo(const gsl_vector* vec);

  int findOptimalStereo(StereoProperties init);

  void PlotProject(std::vector<ProjectedStereo> projected, cv::Mat image_left, cv::Mat image_right);

private:

  float evalStereoError(const std::vector<std::vector<ProjectedStereo> >& proj_stereo);
  double static optFuncWrapper(const gsl_vector *v, void *params);
};

#endif /* VELSTEREOMATCHER_H_ */
