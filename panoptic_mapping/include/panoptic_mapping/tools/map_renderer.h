#ifndef PANOPTIC_MAPPING_TOOLS_MAP_RENDERER_H_
#define PANOPTIC_MAPPING_TOOLS_MAP_RENDERER_H_

#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/core/camera.h"
#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/core/submap.h"
#include "panoptic_mapping/core/submap_collection.h"

namespace panoptic_mapping {

class MapRenderer {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    Camera::Config camera;

    Config() { setConfigName("MapRenderer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit MapRenderer(const Config& config);
  virtual ~MapRenderer() = default;

  // tools
  cv::Mat renderActiveSubmapIDs(const SubmapCollection& submaps,
                                const Transformation& T_M_C);

 private:
  const Config config_;
  const Camera camera_;

  Eigen::MatrixXf range_image_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_MAP_RENDERER_H_
