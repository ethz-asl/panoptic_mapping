#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/map_management/activity_manager.h"
#include "panoptic_mapping/map_management/layer_manipulator.h"
#include "panoptic_mapping/map_management/map_manager_base.h"
#include "panoptic_mapping/map_management/tsdf_registrator.h"

namespace panoptic_mapping {

/**
 * @brief High level class that wraps all map management actions and tools.
 */
class MapManager : public MapManagerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Perform actions every n ticks (frames), set 0 to turn off.
    int prune_active_blocks_frequency = 0;
    int change_detection_frequency = 0;
    int activity_management_frequency = 0;

    // Prune tracked instances in single TSDF mode with class layer
    int prune_tracked_instances_frequency = 0;

    // If true, submaps that are deactivated are checked for alignment with
    // inactive maps and merged together if a match is found.
    bool merge_deactivated_submaps_if_possible = false;

    // If true, the class layer will be integrated into the TSDF layer and
    // discraded afterwards when submaps are deactivated. This saves memory at
    // the loss of classification information.
    bool apply_class_layer_when_deactivating_submaps = false;


    // Member configs.
    TsdfRegistrator::Config tsdf_registrator_config;
    ActivityManager::Config activity_manager_config;
    LayerManipulator::Config layer_manipulator_config;

    Config() { setConfigName("MapManager"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit MapManager(const Config& config);
  virtual ~MapManager() = default;

  // Perform all actions when with specified timings.
  void tick(SubmapCollection* submaps) override;
  void finishMapping(SubmapCollection* submaps) override;

  // Perform specific tasks.
  void pruneActiveBlocks(SubmapCollection* submaps);
  void manageSubmapActivity(SubmapCollection* submaps);
  void performChangeDetection(SubmapCollection* submaps);
  void pruneTrackedInstances(SubmapCollection* submaps);

  // Tools.
  bool mergeSubmapIfPossible(SubmapCollection* submaps, int submap_id,
                             int* merged_id = nullptr);

 protected:
  std::string pruneBlocks(Submap* submap) const;

 private:
  static config_utilities::Factory::RegistrationRos<MapManagerBase, MapManager>
      registration_;
  // Members.
  const Config config_;

  std::shared_ptr<ActivityManager> activity_manager_;
  std::shared_ptr<TsdfRegistrator> tsdf_registrator_;
  std::shared_ptr<LayerManipulator> layer_manipulator_;

  // Action tick counters.
  class Ticker {
   public:
    Ticker(unsigned int max_ticks,
           std::function<void(SubmapCollection* submaps)> action)
        : max_ticks_(max_ticks), action_(std::move(action)) {}
    void tick(SubmapCollection* submaps);

   private:
    unsigned int current_tick_ = 0;
    const unsigned int max_ticks_;
    const std::function<void(SubmapCollection* submaps)> action_;
  };
  std::vector<Ticker> tickers_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_H_
