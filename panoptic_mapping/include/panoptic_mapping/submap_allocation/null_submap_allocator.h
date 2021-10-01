#ifndef PANOPTIC_MAPPING_SUBMAP_ALLOCATION_NULL_SUBMAP_ALLOCATOR_H_
#define PANOPTIC_MAPPING_SUBMAP_ALLOCATION_NULL_SUBMAP_ALLOCATOR_H_

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/labels/label_entry.h"
#include "panoptic_mapping/submap_allocation/freespace_allocator_base.h"
#include "panoptic_mapping/submap_allocation/submap_allocator_base.h"

namespace panoptic_mapping {

/**
 * @brief This submap allocator does not allocate any submaps.
 */
class NullSubmapAllocator : public SubmapAllocatorBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    Config() { setConfigName("NullSubmapAllocator"); }

   protected:
    void setupParamsAndPrinting() override;
  };
  explicit NullSubmapAllocator(const Config& config);
  ~NullSubmapAllocator() override = default;

  Submap* allocateSubmap(SubmapCollection* /* submaps */,
                         InputData* /* input */, int /* input_id */,
                         const LabelEntry& label) override {
    return nullptr;
  }

 private:
  static config_utilities::Factory::RegistrationRos<SubmapAllocatorBase,
                                                    NullSubmapAllocator>
      registration_;
  const Config config_;
};

/**
 * @brief This free space allocator does not allocate any submap.
 */
class NullFreespaceAllocator : public FreespaceAllocatorBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    Config() { setConfigName("NullFreespaceAllocator"); }

   protected:
    void setupParamsAndPrinting() override;
  };

  // Construction.
  explicit NullFreespaceAllocator(const Config& config);
  ~NullFreespaceAllocator() override = default;

  Submap* allocateSubmap(SubmapCollection* /* submaps */,
                         InputData* /* input */) override {
    return nullptr;
  }

 private:
  static config_utilities::Factory::RegistrationRos<FreespaceAllocatorBase,
                                                    NullFreespaceAllocator>
      registration_;
  const Config config_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_SUBMAP_ALLOCATION_NULL_SUBMAP_ALLOCATOR_H_
