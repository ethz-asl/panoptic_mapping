#include "panoptic_mapping/map/classification/variable_bayesian.h"

#include <memory>
#include <utility>
#include <vector>

namespace panoptic_mapping {

ClassVoxelType VariableBayesianVoxel::getVoxelType() const {
  return ClassVoxelType::kVariableBayesian;
}

bool VariableBayesianVoxel::isObserverd() const { return !probs.empty(); }

bool VariableBayesianVoxel::belongsToSubmap() const { return true; }

float VariableBayesianVoxel::getBelongingProbability() const {
  if (current_id == 0) {
    return 0.f;
  }
  return probs.at(current_id);
}

int VariableBayesianVoxel::getBelongingID() const { return current_id; }

float VariableBayesianVoxel::getProbability(const int id) const {
  if (probs.empty()) {
    return 0.f;
  }
  auto it = probs.find(id);
  if (it == probs.end()) {
    return 0.f;
  }
  return it->second;
}

void VariableBayesianVoxel::incrementCount(const int id, const float weight) {
  // Sanity checks
  if (id == 0) {
    LOG(WARNING) << "Ignore instance id should not be tracked";
    return;
  }
  if (weight < 0.f || weight > 1.f) {
    LOG(WARNING) << "Observed id weight must be a float between 0 and 1.";
    return;
  }

  if (probs.empty()) {
    probs.insert(std::make_pair(id, weight));
    return;
  }

  // Update the probability distribution
  auto it = probs.find(id);
  if (it != probs.end()) {
    it->second =
        kUncertaintyDecay * it->second + (1 - kUncertaintyDecay) * weight;

    // Probability that the voxel belongs to a previously observed
    // instance different from the one currently predicted
    const float unobserved_prob = (1 - weight) / probs.size();
    for (auto& [i, prob] : probs) {
      if (i == id) {
        continue;
      }
      prob =
          kUncertaintyDecay * prob + (1 - kUncertaintyDecay) * unobserved_prob;
    }
  }
  // Add new instance id to the distribution
  else {
    const float new_dist_len = static_cast<float>(1 + probs.size());
    const float new_id_prob = kUncertaintyDecay * (1 / new_dist_len) +
                              (1 - kUncertaintyDecay) * weight;
    probs.insert(std::make_pair(id, new_id_prob));

    // Probability that the voxel belongs to a previously observed
    // instance different from the one currently predicted
    const float unobserved_prob = (1 - weight) / probs.size();
    for (auto& [i, prob] : probs) {
      if (i == id) {
        continue;
      }
      prob =
          kUncertaintyDecay * prob + (1 - kUncertaintyDecay) * unobserved_prob;
    }
  }

  // Compute normalizer
  float normalizer = std::accumulate(
      std::begin(probs), std::end(probs), 0.f,
      [](const float s, const auto& next) { return s + next.second; });

  // Before normalizing, loop through the ids and check which ids have prob
  // lower than the min allowed probability and subtract their prob from the
  // normalizer and discard
  const float min_prob_x_normalizer = normalizer * kMinProbability;
  for (auto it = probs.begin(); it != probs.end();) {
    if (it->second < min_prob_x_normalizer) {
      normalizer -= it->second;
      it = probs.erase(it);
    } else {
      ++it;
    }
  }

  // Now apply the normalizer and compute the max prob id
  current_id = -1;
  float max_prob = 0.f;
  for (auto& [id, prob] : probs) {
    prob /= normalizer;
    if (prob > max_prob) {
      current_id = id;
      max_prob = prob;
    }
  }

}  // namespace panoptic_mapping

bool VariableBayesianVoxel::mergeVoxel(const ClassVoxel& other) {
  LOG(WARNING) << "Merging not supported for VariableBayesianVoxel";
  return false;
}

std::vector<uint32_t> VariableBayesianVoxel::serializeVoxelToInt() const {
  // count + 2 32-bit ints for each id prob pair
  std::vector<uint32_t> result(1 + 2 * probs.size());
  result[0] = probs.size();

  size_t index = 0;
  for (const auto& id_prob_pair : probs) {
    index++;
    result[index] = static_cast<uint32_t>(id_prob_pair.first);
    index++;
    result[index] = int32FromX32(id_prob_pair.second);
  }

  return result;
}

bool VariableBayesianVoxel::deseriliazeVoxelFromInt(
    const std::vector<uint32_t>& data, size_t* data_index) {
  if (*data_index >= data.size()) {
    LOG(WARNING)
        << "Can not deserialize voxel from integer data: Out of range (index: "
        << *data_index << ", data: " << data.size() << ")";
    return false;
  }

  // Get number of id prob pairs to load
  const size_t num_id_prob_pairs = data[*data_index];
  const size_t length = 1 + 2 * num_id_prob_pairs;
  if (*data_index + length > data.size()) {
    LOG(WARNING) << "Can not deserialize voxel from integer data: Not enough "
                    "data (index: "
                 << *data_index << "-" << (*data_index + length)
                 << ", data: " << data.size() << ")";
    return false;
  }

  // Get the data.
  probs.clear();
  current_id = -1;
  float max_prob = 0.f;
  for (size_t i = 1; i < length; i += 2) {
    int id = static_cast<int>(data[*data_index + i]);
    float prob = x32FromInt32<float>(data[*data_index + i + 1]);
    probs.insert(std::make_pair(id, prob));
    if (prob > max_prob) {
      max_prob = prob;
      current_id = id;
    }
  }
  *data_index += length;
  return true;
}

config_utilities::Factory::RegistrationRos<ClassLayer, VariableBayesianLayer, float,
                                           int>
    VariableBayesianLayer::registration_("variable_bayesian");

VariableBayesianLayer::VariableBayesianLayer(const Config& config,
                                     const float voxel_size,
                                     const int voxels_per_side)
    : config_(config.checkValid()),
      ClassLayerImpl(voxel_size, voxels_per_side) {}

ClassVoxelType VariableBayesianLayer::getVoxelType() const {
  return ClassVoxelType::kVariableBayesian;
}

std::unique_ptr<ClassLayer> VariableBayesianLayer::clone() const {
  return std::make_unique<VariableBayesianLayer>(*this);
}

std::unique_ptr<ClassLayer> VariableBayesianLayer::loadFromStream(
    const SubmapProto& submap_proto, std::istream* /* proto_file_ptr */,
    uint64_t* /* tmp_byte_offset_ptr */) {
  // Nothing special needed to configure for binary counts.
  return std::make_unique<VariableBayesianLayer>(VariableBayesianLayer::Config(),
                                         submap_proto.voxel_size(),
                                         submap_proto.voxels_per_side());
}

}  // namespace panoptic_mapping
