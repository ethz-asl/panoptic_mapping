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

namespace panoptic_mapping {

/**
 * High level class that wraps all map management actions and tools.
 */

class MapManager {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Perform actions every n ticks, 0 to ignore.
    int prune_active_blocks_frequency = 0;

    Config() { setConfigName("MapManager"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  MapManager(const Config& config, std::shared_ptr<SubmapCollection> map);
  virtual ~MapManager() = default;

  // Perform all required actions.
  void tickMapManagement();

  // Access to specific tasks.
  void pruneActiveBlocks();

 protected:
  std::string pruneBlocks(Submap* submap);

 private:
  // Members.
  const Config config_;
  const std::shared_ptr<SubmapCollection> map_;

  // Action tick counters.
  class Ticker {
   public:
    Ticker(unsigned int max_ticks, std::function<void()> action)
        : max_ticks_(max_ticks), action_(std::move(action)) {}
    void tick();

   private:
    unsigned int current_tick_ = 0;
    const unsigned int max_ticks_;
    const std::function<void()> action_;
  };
  std::vector<Ticker> tickers_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_H_
