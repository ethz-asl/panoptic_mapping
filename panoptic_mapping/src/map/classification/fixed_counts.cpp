#include "panoptic_mapping/map/classification/fixed_counts.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType FixedCountVoxel::getVoxelType() const {
  return ClassVoxelType::kFixedCount;
}

size_t FixedCountVoxel::kNumCounts = 0u;

size_t FixedCountVoxel::numCounts() { return kNumCounts; }
void FixedCountVoxel::setNumCounts(size_t num_counts) {
  if (num_counts == kNumCounts) {
    return;
  }
  if (kNumCounts != 0u) {
    // This means the number of classes has already been initialized and since
    // this is currently a global property raise a warning at least.
    LOG(WARNING) << "The number of classes to be stored in FixedCountVoxels "
                    "was already initialized to "
                 << kNumCounts << ", now set to " << num_counts
                 << ". This can cause undefined behavior.";
  }
  kNumCounts = num_counts;
}

bool FixedCountVoxel::isObserverd() const { return !counts.empty(); }

bool FixedCountVoxel::belongsToSubmap() const {
  // The current index keeps track of the highest count, zero is usually
  // reserved for the belonging submap.
  return current_index == 0;
}

float FixedCountVoxel::getBelongingProbability() const {
  if (counts.empty()) {
    return 0.f;
  }
  return static_cast<float>(counts[0]) / static_cast<float>(total_count);
}

int FixedCountVoxel::getBelongingID() const { return current_index; }

float FixedCountVoxel::getProbability(const int id) const {
  if (id < 0 || id > kNumCounts || counts.empty()) {
    return 0.f;
  }
  return static_cast<float>(counts[id]) / static_cast<float>(total_count);
}

void FixedCountVoxel::incrementCount(const int id, const float weight) {
  if (id < 0 || id > kNumCounts) {
    return;
  }
  // Check initialization lazily allocate the counts but full memory so no
  // re-allocation happens. resize value-initia
  if (counts.empty()) {
    counts.resize(kNumCounts, 0u);
  }
  const ClassificationCount new_count = ++counts[id];
  if (new_count > current_count) {
    current_index = id;
    current_count = new_count;
  }
  ++total_count;
}

std::vector<uint32_t> FixedCountVoxel::serializeVoxelToInt() const {}

void FixedCountVoxel::deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                                              size_t* data_index) {}

config_utilities::Factory::RegistrationRos<ClassLayer, FixedCountLayer, float,
                                           int>
    FixedCountLayer::registration_("fixed_count");

FixedCountLayer::FixedCountLayer(const Config& config, const float voxel_size,
                                 const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType FixedCountLayer::getVoxelType() const {
  return ClassVoxelType::kFixedCount;
}

std::unique_ptr<ClassLayer> FixedCountLayer::clone() const {
  return std::make_unique<FixedCountLayer>(*this);
}

std::unique_ptr<ClassLayer> FixedCountLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<FixedCountLayer>(FixedCountLayer::Config(),
                                           submap_proto.voxel_size(),
                                           submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
