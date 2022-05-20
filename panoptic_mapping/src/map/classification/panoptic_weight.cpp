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

bool PanopticWeightVoxel::belongsToSubmap() const { return true; }

float PanopticWeightVoxel::getBelongingProbability() const { return getProbability(label); }

int PanopticWeightVoxel::getBelongingID() const { return label; }

void PanopticWeightVoxel::setTsdfWeight(float tsdf_weight) {
  this->tsdf_weight = tsdf_weight;
}

float PanopticWeightVoxel::getProbability(const int id) const {
  if (id == label) {
    if(label == 0) {
      return 0.f;
    }
    return 0.5 * (1 + label_weight / tsdf_weight);
  } else {
    return 0.5;
  }
}

void PanopticWeightVoxel::incrementCount(const int id, const float weight) {
  // If the predicted id is the same as the current one increase the weight
  if (id == label) {
    // If label is unknown, no need to reinforce
    if (label == 0) {
      label_weight = 0.f;
    } else {
      label_weight += weight;
    }
  } else {
    // If the predicted id is different subtract the weight
    if (label_weight < weight) {
      // If the weight becomes negative, swap the id to the predicted one
      label = id;
      label_weight = weight - label_weight;
    } else {
      label_weight -= weight;
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
  result.push_back(int32FromX32<float>(label_weight));
  result.push_back(int32FromX32<float>(tsdf_weight));

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
  label_weight = x32FromInt32<float>(data[*data_index] + 1);
  tsdf_weight = x32FromInt32<float>(data[*data_index] + 2);

  // Advance the index
  *data_index += 3;

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
