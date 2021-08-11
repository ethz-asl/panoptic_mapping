#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_LAYER_MANIPULATOR_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_LAYER_MANIPULATOR_H_

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * @brief This class performs all sort of submap and layer manipulations, such
 * as smoothing, merging, pruning, integrating classification into TSDF.
 */

class LayerManipulator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    bool use_instance_classification =
        false;  // How to interpret the class layer data.

    Config() { setConfigName("LayerManipulator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit LayerManipulator(const Config& config);
  virtual ~LayerManipulator() = default;

  /* Tools */
  // Trim the TSDF layer according to the provided class layer. Tsdf and class
  // layer are expected to have identical layout, extent and transformation.
  void applyClassificationLayer(TsdfLayer* tsdf_layer,
                                const ClassLayer& class_layer,
                                float truncation_distance) const;

  void mergeSubmapAintoB(const Submap& A, Submap* B) const;

 private:
  const Config config_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_LAYER_MANIPULATOR_H_
