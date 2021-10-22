#ifndef PANOPTIC_MAPPING_INTEGRATION_PROJECTION_INTERPOLATORS_H_
#define PANOPTIC_MAPPING_INTEGRATION_PROJECTION_INTERPOLATORS_H_

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

/**
 * @brief Interface for different ways to interpolate the necessary values.
 * Use computeWeights() first to setup the interpolator, then use the
 * other functions to access the relevant data.
 */
class InterpolatorBase {
 public:
  /**
   * @brief Sets up the interpolator for a specific lookup and stores all
   * relevant information in the internal state. This method assumes that u and
   * v are fully within the bounds of the range image, which is not re-checked.
   *
   * @param u Horizontal position in image space of the point to interpolate.
   * @param v Vertical position in image space of the point to interpolate.
   * @param range_image Range image used to interpolate the depth.
   */
  virtual void computeWeights(float u, float v,
                              const Eigen::MatrixXf& range_image) = 0;

  /**
   * @brief Compute the depth based on the internally cached weights.
   *
   * @param range_image Range image to interpolate in.
   * @return float The interpolated range value.
   */
  virtual float interpolateRange(const Eigen::MatrixXf& range_image) = 0;

  /**
   * @brief Compute the color based on the internally cached weights.
   *
   * @param color_image Color image to interpolate in.
   * @return Color The interpolated color value.
   */
  virtual Color interpolateColor(const cv::Mat& color_image) = 0;

  /**
   * @brief Compute the submapID based on the internally cached weights.
   *
   * @param id_image ID image to interpolate in.
   * @return int The interpolated ID value.
   */
  virtual int interpolateID(const cv::Mat& id_image) = 0;

    /**
   * @brief Compute the uncertainty based on the internally cached weights.
   *
   * @param uncertainty_image Uncertainty Image image to interpolate in.
   * @return Color The interpolated uncertainty value.
   */
  virtual float interpolateUncertainty(const cv::Mat& uncertainty_image) = 0;
};

/**
 * @brief Interpolates values by selecting the nearest neighbor. Use
 computeWeights() first to setup the interpolator, then use the other functions
 to access the relevant data.
 */
class InterpolatorNearest : public InterpolatorBase {
 public:
  void computeWeights(float u, float v,
                      const Eigen::MatrixXf& range_image) override;
  float interpolateRange(const Eigen::MatrixXf& range_image) override;
  Color interpolateColor(const cv::Mat& color_image) override;
  float interpolateUncertainty(const cv::Mat& uncertainty_image)  override;
  int interpolateID(const cv::Mat& id_image) override;

 protected:
  int u_;
  int v_;

 private:
  static config_utilities::Factory::Registration<InterpolatorBase,
                                                 InterpolatorNearest>
      registration_;
};

/**
 * @brief Interpolates values using bilinear interpolation. Use computeWeights()
 first to setup the interpolator, then use the other functions to access the
 relevant data.
 */
class InterpolatorBilinear : public InterpolatorBase {
 public:
  void computeWeights(float u, float v,
                      const Eigen::MatrixXf& range_image) override;
  float interpolateRange(const Eigen::MatrixXf& range_image) override;
  float interpolateUncertainty(const cv::Mat& uncertainty_image)  override;
  Color interpolateColor(const cv::Mat& color_image) override;
  int interpolateID(const cv::Mat& id_image) override;

 protected:
  int u_;
  int v_;
  float weight_[4];

 private:
  static config_utilities::Factory::Registration<InterpolatorBase,
                                                 InterpolatorBilinear>
      registration_;
};

/**
 * @brief Use bilinear interpolation if the range values are all close,
 * otherwise use nearest neighbor lookup. This behavior should capture surface
 * discontinuities which would otherwise be interpolated. Use computeWeights()
 first to setup the interpolator, then use the other functions to access the
 relevant data.
 */
class InterpolatorAdaptive : public InterpolatorBilinear {
 public:
  void computeWeights(float u, float v,
                      const Eigen::MatrixXf& range_image) override;
  float interpolateRange(const Eigen::MatrixXf& range_image) override;
  Color interpolateColor(const cv::Mat& color_image) override;
  int interpolateID(const cv::Mat& id_image) override;
  float interpolateUncertainty(const cv::Mat& uncertainty_image)  override;

 protected:
  int u_;
  int v_;
  int u_offset_[4] = {0, 0, 1, 1};
  int v_offset_[4] = {0, 1, 0, 1};
  float weight_[4];
  bool use_bilinear_;

 private:
  static config_utilities::Factory::Registration<InterpolatorBase,
                                                 InterpolatorAdaptive>
      registration_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_INTEGRATION_PROJECTION_INTERPOLATORS_H_
