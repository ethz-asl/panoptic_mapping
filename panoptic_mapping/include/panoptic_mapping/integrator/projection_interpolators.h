#ifndef PANOPTIC_MAPPING_INTEGRATOR_PROJECTION_INTERPOLATORS_H_
#define PANOPTIC_MAPPING_INTEGRATOR_PROJECTION_INTERPOLATORS_H_

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/core/common.h"

namespace panoptic_mapping {

class InterpolatorBase;

/**
 * Different ways to interpolate.
 */
class InterpolatorBase {
 public:
  // set up the interpolator
  virtual void computeWeights(float u, float v, int id, bool *belongs_to_this_submap, const Eigen::MatrixXf &range_image, const cv::Mat &id_image) = 0;

  // use these to read out the values
  virtual float interpolateDepth(const Eigen::MatrixXf &range_image) = 0;
  virtual Color interpolateColor(const cv::Mat &color_image) = 0;
};

/**
 * Creation Utility.
 */
class InterpolatorFactory {
 public:
  static std::unique_ptr<InterpolatorBase> create(const std::string &type);
 private:
  InterpolatorFactory() = default;
};

/**
 * Nearest neighbor point.
 */
class InterpolatorNearest : public InterpolatorBase {
 public:
  void computeWeights(float u, float v, int id, bool *belongs_to_this_submap, const Eigen::MatrixXf &range_image, const cv::Mat &id_image) override;
  float interpolateDepth(const Eigen::MatrixXf &range_image) override;
  Color interpolateColor(const cv::Mat &color_image) override;

 protected:
  int u_;
  int v_;
};

/**
 * Use Bilinear interpolation for everything.
 */
class InterpolatorBilinear : public InterpolatorBase {
 public:
  void computeWeights(float u, float v, int id, bool *belongs_to_this_submap, const Eigen::MatrixXf &range_image, const cv::Mat &id_image) override;
  float interpolateDepth(const Eigen::MatrixXf &range_image) override;
  Color interpolateColor(const cv::Mat &color_image) override;

 protected:
  int u_;
  int v_;
  float weight_[4];
};

/**
 * Use Bilinear interpolation if depth and semantics are all close, otherwise nearest
 */
class InterpolatorAdaptive : public InterpolatorBilinear {
 public:
  void computeWeights(float u, float v, int id, bool *belongs_to_this_submap, const Eigen::MatrixXf &range_image, const cv::Mat &id_image) override;
  float interpolateDepth(const Eigen::MatrixXf &range_image) override;
  Color interpolateColor(const cv::Mat &color_image) override;

 protected:
  int u_;
  int v_;
  int u_offset_[4] = {0, 0, 1, 1};
  int v_offset_[4] = {0, 1, 0, 1};
  float weight_[4];
  bool use_bilinear_;
};

/**
 * Count all points that match the label.
 */
class InterpolatorSemantic : public InterpolatorBase {
 public:
  void computeWeights(float u, float v, int id, bool *belongs_to_this_submap, const Eigen::MatrixXf &range_image, const cv::Mat &id_image) override;
  float interpolateDepth(const Eigen::MatrixXf &range_image) override;
  Color interpolateColor(const cv::Mat &color_image) override;

 protected:
  int u_;
  int v_;
  int u_offset_[4] = {0, 0, 1, 1};
  int v_offset_[4] = {0, 1, 0, 1};
  float weight_[4];
  float total_weight_;
};

}  // namespace panoptic_mapping

#endif // PANOPTIC_MAPPING_INTEGRATOR_PROJECTIVE_INTEGRATOR_H_