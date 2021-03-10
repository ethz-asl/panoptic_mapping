#include "panoptic_mapping/preprocessing/geometric_edge_refiner.h"

namespace panoptic_mapping {

void GeomtricEdgeRefiner::Config::checkParams() const {
  //  checkParamGT(voxels_per_side, 0, "voxels_per_side");
}

void GeomtricEdgeRefiner::Config::setupParamsAndPrinting() {
  //  setupParam("verbosity", &verbosity);
}

GeomtricEdgeRefiner::GeomtricEdgeRefiner(const Config& config)
    : config_(config.checkValid()) {}

void GeomtricEdgeRefiner::refinePrediction(const cv::Mat& depth_image,
                                           const Camera& camera,
                                           cv::Mat* id_image) {
  // Compute the normal map.
}

}  // namespace panoptic_mapping
