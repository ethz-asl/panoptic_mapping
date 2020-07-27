#ifndef PANOPTIC_MAPPING_REGISTRATION_TSDF_REGISTRATOR_H_
#define PANOPTIC_MAPPING_REGISTRATION_TSDF_REGISTRATOR_H_

#include "panoptic_mapping/core/submap.h"

namespace panoptic_mapping {

/**
 * This class directly compares TSDF volumes of two submaps with each other,
 * using the registration constraints adapted from voxgraph to detect matches
 * and solve for transformations using ceres.
 */
class TsdfRegistrator {
 public:
  struct Config {
    [[nodiscard]] Config isValid() const;
  };
  explicit TsdfRegistrator(const Config& config);
  virtual ~TsdfRegistrator() = default;

  // check whether there is significant difference between the two submaps
  bool checkSubmapAlignment(Submap* reference, Submap* other);

 private:
  const Config config_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_REGISTRATION_TSDF_REGISTRATOR_H_
