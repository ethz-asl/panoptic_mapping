#ifndef PANOPTIC_MAPPING_TOOLS_MAP_RENDERER_H_
#define PANOPTIC_MAPPING_TOOLS_MAP_RENDERER_H_

#include <voxblox/utils/color_maps.h>
#include <opencv2/core/mat.hpp>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

class MapRenderer {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    Camera::Config camera;
    bool impaint_voxel_size = false;

    Config() { setConfigName("MapRenderer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit MapRenderer(const Config& config);
  virtual ~MapRenderer() = default;

  // Tools.
  cv::Mat renderActiveSubmapIDs(const SubmapCollection& submaps,
                                const Transformation& T_M_C);
  cv::Mat renderActiveSubmapClasses(const SubmapCollection& submaps,
                                    const Transformation& T_M_C);
  cv::Mat colorIdImage(const cv::Mat& id_image, int colors_per_revolution = 10);

 private:
  const Config config_;
  const Camera camera_;

  //
  Eigen::MatrixXf range_image_;
  voxblox::ExponentialOffsetIdColorMap id_color_map_;

  // Methods.
  cv::Mat render(const SubmapCollection& submaps, const Transformation& T_M_C,
                 bool only_active_submaps, int (*paint)(const Submap&));
  static int paintSubmapID(const Submap& submap);
  static int paintClass(const Submap& submap);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_MAP_RENDERER_H_
