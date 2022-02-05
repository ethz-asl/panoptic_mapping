#include "panoptic_mapping/map/classification/panoptic_weight.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "panoptic_mapping/tools/serialization.h"

namespace panoptic_mapping {

ClassVoxelType PanopticWeightVoxel::getVoxelType() const {
  return ClassVoxelType::kPanopticWeight;
}

bool PanopticWeightVoxel::isObserverd() const { return label != 0; }

bool PanopticWeightVoxel::belongsToSubmap() const { return false; }

float PanopticWeightVoxel::getBelongingProbability() const { return 0.f; }

int PanopticWeightVoxel::getBelongingID() const { return label; }

float PanopticWeightVoxel::getProbability(const int id) const { return 0.f; }

void PanopticWeightVoxel::incrementCount(const int id, const float weight) {
  // If the predicted id is the same as the current one increase the weight
  if (id == label) {
    // If label is unknown, no need to reinforce
    if (label == 0) {
      this->weight = 0.f;
    } else {
      this->weight += weight;
    }
  } else {
    // If the predicted id is different subtract the weight
    if (this->weight < weight) {
      // If the weight becomes negative, swap the id to the predicted one
      this->weight = weight - this->weight;
      label = id;
    } else {
      this->weight -= weight;
    }
  }
}

bool PanopticWeightVoxel::mergeVoxel(const ClassVoxel& other) {
  LOG(WARNING) << "Merging not supported for voxel of type PanopticWeightVoxel";
  return false;
}

std::vector<uint32_t> PanopticWeightVoxel::serializeVoxelToInt() const {
  // Store the label followed by its weight
  std::vector<uint32_t> result;
  result.push_back(static_cast<uint32_t>(label));
  result.push_back(int32FromX32<float>(weight));

  return result;
}

bool PanopticWeightVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  if (*data_index >= data.size()) {
    LOG(WARNING)
        << "Can not deserialize voxel from integer data: Out of range (index: "
        << *data_index << ", data: " << data.size() << ")";
    return false;
  }

  // Load data
  label = static_cast<int>(data[*data_index]);
  weight = x32FromInt32<float>(data[*data_index + 1]);

  // Advance the index
  *data_index += 2;

  return true;
}

config_utilities::Factory::RegistrationRos<ClassLayer, PanopticWeightLayer,
                                           float, int>
    PanopticWeightLayer::registration_("panoptic_weight");

PanopticWeightLayer::PanopticWeightLayer(const Config& config,
                                         const float voxel_size,
                                         const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType PanopticWeightLayer::getVoxelType() const {
  return ClassVoxelType::kPanopticWeight;
}

std::unique_ptr<ClassLayer> PanopticWeightLayer::clone() const {
  return std::make_unique<PanopticWeightLayer>(*this);
}

std::unique_ptr<ClassLayer> PanopticWeightLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  return std::make_unique<PanopticWeightLayer>(PanopticWeightLayer::Config(),
                                               submap_proto.voxel_size(),
                                               submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
