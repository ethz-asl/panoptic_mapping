#include "panoptic_mapping/map/classification/fixed_count.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "panoptic_mapping/tools/serialization.h"

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

bool FixedCountVoxel::mergeVoxel(const ClassVoxel& other) {
  // Check type compatibility.
  auto voxel = dynamic_cast<const FixedCountVoxel*>(&other);
  if (!voxel) {
    LOG(WARNING)
        << "Can not merge voxels that are not of same type (FixedCountVoxel).";
    return false;
  }
  // Since in both cases the belonging submap is at index 0 we just merge the
  // full vector. Also works for semantic segmentation with consistent labels.
  // NOTE(schmluk): For inconsistent labels the belonging counts of other are
  // somewhere in [1, kNumCounts], which is not corrected for here!
  if (voxel->counts.empty()) {
    return true;
  }
  if (counts.empty()) {
    counts = voxel->counts;
    current_count = voxel->current_count;
    current_index = voxel->current_index;
    total_count = voxel->total_count;
    return true;
  }
  current_index = -1;
  current_count = 0;
  total_count += voxel->total_count;
  if (counts.size() != voxel->counts.size()) {
    LOG(WARNING) << "Can not merge FixedCount Voxels of different sizes ("
                 << counts.size() << " vs " << voxel->counts.size() << ").";
    return false;
  }
  for (size_t i = 0; i < counts.size(); ++i) {
    counts[i] += voxel->counts[i];
    if (counts[i] > current_count) {
      current_count = counts[i];
      current_index = i;
    }
  }
  return true;
}

std::vector<uint32_t> FixedCountVoxel::serializeVoxelToInt() const {
  // Store the number of counts first followed by all the values.
  const size_t length = (counts.size() + 3u) / 2u;
  std::vector<uint32_t> result(length);
  result[0] = counts.size();
  for (size_t i = 1; i < counts.size(); i += 2u) {
    result[i / 2 + 1] = int32FromTwoInt16(counts[i - 1], counts[i]);
  }
  if (counts.size() % 2 != 0) {
    result[length - 1] = int32FromTwoInt16(counts.back(), 0u);
  }
  return result;
}

bool FixedCountVoxel::deseriliazeVoxelFromInt(const std::vector<uint32_t>& data,
                                              size_t* data_index) {
  if (*data_index >= data.size()) {
    LOG(WARNING)
        << "Can not deserialize voxel from integer data: Out of range (index: "
        << *data_index << ", data: " << data.size() << ")";
    return false;
  }

  // Check number of counts to load.
  const size_t num_counts = data[*data_index];
  const size_t length = (num_counts + 3u) / 2u;
  if (*data_index + length > data.size()) {
    LOG(WARNING) << "Can not deserialize voxel from integer data: Not enough "
                    "data (index: "
                 << (*data_index + length - 1) << ", data: " << data.size()
                 << ")";
    return false;
  }

  // Load data.
  counts.resize(num_counts);
  current_count = 0;
  current_index = -1;
  total_count = 0;
  std::pair<uint16_t, uint16_t> datum;
  for (size_t i = 0; i < num_counts; ++i) {
    if (i % 2 == 0) {
      datum = twoInt16FromInt32(data[*data_index + 1u + i / 2u]);
      counts[i] = datum.first;
    } else {
      counts[i] = datum.second;
    }

    // Recompute the current count and index from the data.
    if (counts[i] > current_count) {
      current_count = counts[i];
      current_index = i;
    }
    total_count += counts[i];
  }
  *data_index += length;
  return true;
}

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
