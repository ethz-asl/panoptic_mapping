#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_ACTIVITY_MANAGER_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_ACTIVITY_MANAGER_H_

#include <voxblox/interpolator/interpolator.h>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * This class checks when to deactivate inactive submaps
 */

class ActivityManager {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // New Submaps.
    int required_reobservations = 0;  // Remove submaps that are not observed in
                                      // x consecutive frames after allocation.

    // Deactivation.
    int deactivate_after_missed_detections =
        0;  // Deactivate submaps when not observed for x frames. Use 0 to
            // ignore.

    Config() { setConfigName("ActivityManager"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit ActivityManager(const Config& config);
  virtual ~ActivityManager() = default;

  // Check whether there is significant difference between the two submaps.
  void processSubmaps(SubmapCollection* submaps);

 private:
  const Config config_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_ACTIVITY_MANAGER_H_
