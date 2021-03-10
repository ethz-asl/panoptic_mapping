#ifndef PANOPTIC_MAPPING_PREPROCESSING_GEOMETRIC_EDGE_REFINER_H_
#define PANOPTIC_MAPPING_PREPROCESSING_GEOMETRIC_EDGE_REFINER_H_

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"

namespace panoptic_mapping {

/**
 * Uses the approach described here
 * https://ieeexplore.ieee.org/abstract/document/8613746 to compute depth
 * discontinuity and convexity terms for a given depth image. This value can
 * then be thresholded to extract patches of continuous surfaces to refine the
 * edges of a semantic prediction.
 */

class GeomtricEdgeRefiner {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("GeomtricEdgeRefiner"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit GeomtricEdgeRefiner(const Config& config);
  ~GeomtricEdgeRefiner() = default;

  void refinePrediction(const cv::Mat& depth_image, const Camera& camera,
                        cv::Mat* id_image);
  const cv::Mat& getNormalMap() const { return normals_; }
  const cv::Mat& getEdginessMap() const { return edginess_map_; }

 private:
  const Config config_;

  // Cached data
  cv::Mat normals_;
  cv::Mat edginess_map_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_PREPROCESSING_GEOMETRIC_EDGE_REFINER_H_
