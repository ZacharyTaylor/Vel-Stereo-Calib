/*
 * StereoOptVec.h
 *
 *  Created on: 3 Oct 2015
 *      Author: z
 */

#ifndef STEREOOPTVEC_H_
#define STEREOOPTVEC_H_

#include <array>

#include <gsl/gsl_multimin.h>

#include "StereoProperties.h"

/// Holds all of the stereo parameters it is possible to optimize over.
enum class StereoOptFlag
{
  X, Y, Z, RX, RY, RZ, FX, FY, CX, CY, BASELINE
};

/// Allows the stereo system to be processed using the gsl optimization library.
class StereoOptVec : private StereoProperties
{
public:

  std::map<StereoOptFlag, bool> opt_flags; ///< holds flags indicating which parameters will be optimized.

  StereoOptVec(const StereoProperties init, const bool opt_intrinsics = false);
  StereoOptVec(const StereoProperties init, const std::map<StereoOptFlag, bool> opt_flags_in);

  /// Creates a StereoProperties object from the system's properties.
  StereoProperties getStereoProperties(void) const;
  /// Creates a gsl_vector from the system's properties.
  gsl_vector* getGSLVector(void) const;

  /// Updates the properties given by opt_flag according to vec
  /**
   * __Important__: To function correctly vec must have been generated from a StereoOptVec that has opt_flags in the same state.
   * @param vec vector of values corresponding to the true values of opt_flags.
   */
  void setUsingGSLVector(gsl_vector* vec);

private:
  using flag = StereoOptFlag;
  const std::vector<flag> extrinsic_flags = {flag::X, flag::Y, flag::Z, flag::RX, flag::RY, flag::RZ};
  const std::vector<flag> intrinsic_flags = {flag::FX, flag::FY, flag::CX, flag::CY, flag::BASELINE};
  const std::vector<flag> all_flags = {flag::X, flag::Y, flag::Z, flag::RX, flag::RY, flag::RZ, flag::FX, flag::FY,
                                       flag::CX, flag::CY, flag::BASELINE};

  /// Maps the internal StereoProperties to each of the parameters given by StereoOptFlag
  std::map<flag, float> getValueMap(void) const;
};

#endif /* STEREOOPTVEC_H_ */
