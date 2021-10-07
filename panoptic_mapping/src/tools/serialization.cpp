#include "panoptic_mapping/tools/serialization.h"

#include <cmath>
#include <queue>
#include <vector>

typedef panoptic_mapping::ClassVoxel::Counter Counter;

namespace voxblox {
/**
 * Stores the top 3 numbers and their indices in the indices and counts
 * vectors
 */
inline void getTopNElems(const std::vector<Counter>& all_items,
                            std::vector<uint8_t>& indices,
                            std::vector<Counter>& counts, const int n) {
  std::priority_queue<std::pair<Counter, uint8_t>> count_to_idx;
  for (uint8_t i = 0; i < static_cast<uint8_t>(all_items.size()); ++i) {
    count_to_idx.push(std::pair<Counter, uint8_t>(all_items[i], i));
  }
  for (uint8_t i = 0; i < n; ++i) {
    counts.push_back(count_to_idx.top().first);
    indices.push_back(count_to_idx.top().second);
    count_to_idx.pop();
  }
}

/**
 * Fallback Voxblox function utilizing
 * PANOPTIC_MAPPING_DEFAULT_SERIALIZATION_COUNT to determine how many counts to
 * serialize Should not be used if possible as counts can not be chosen
 */
template <>
void Block<panoptic_mapping::ClassUncertaintyVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const {
  serializeBlockToIntegers<panoptic_mapping::ClassUncertaintyVoxel>(
      this, data, PANOPTIC_MAPPING_DEFAULT_SERIALIZATION_COUNT);
}

/**
 * Fallback Voxblox function utilizing
 * PANOPTIC_MAPPING_DEFAULT_SERIALIZATION_COUNT to determine how many counts to
 * serialize Should not be used if possible as counts can not be chosen
 */
template <>
void Block<panoptic_mapping::ClassVoxel>::serializeToIntegers(
    std::vector<uint32_t>* data) const {
  serializeBlockToIntegers<panoptic_mapping::ClassVoxel>(
      this, data, PANOPTIC_MAPPING_DEFAULT_SERIALIZATION_COUNT);
}

template <>
void Block<panoptic_mapping::ClassVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  deserializeBlockFromIntegers(this, data);
}
template <>
void Block<panoptic_mapping::ClassUncertaintyVoxel>::deserializeFromIntegers(
    const std::vector<uint32_t>& data) {
  deserializeBlockFromIntegers(this, data);
}
/**
 * Converts a vector with numbers that are smaller than 32 bit into multiple
 * 32bit integers.
 * @param data: Vector containing data that should be stored
 * @param serialized_data: Vector where the serialized int should be stored
 * @param bits_per_entry: How many bits are needed to encode the data (8bit /
 * 16 bit)
 */
template <typename T>
inline void convert_vector_to_uint32(std::vector<T> data,
                                     std::vector<uint32_t>* serialized_data,
                                     uint8_t bits_per_entry) {
  // Store how many packages have been saved to the serialized vector
  int saved_count = 0;
  // Data encoded as int32
  uint32_t data_int = 0;
  // How many data packages fit in a 32bit int
  const uint8_t data_packages = 32 / bits_per_entry;
  for (int i = 0; i < data.size(); i++) {
    int byte_idx = i % data_packages;
    // Bitshift data to correct position
    data_int += (data.at(i) << (byte_idx * bits_per_entry));
    if (byte_idx == data_packages - 1) {
      // push saved integer
      serialized_data->push_back(data_int);
      saved_count += data_packages;
      data_int = 0;
    }
  }
  // Means last data had e.g. 3 out of 4 bytes
  // filled and was not pushed.
  if (saved_count < data.size()) {
    serialized_data->push_back(data_int);
  }
}

/**
 * Converts a voxel to [int32] and add it to data.
 * Note only the top N counts (defined in
 * PANOPTIC_MAPPING_SERIALIZE_TOP_N_COUNTS) will be stored in order to save
 * storage/memory
 *
 * [Int32] Structure:
 * [ '#ClassesForThisVoxel',
 *   'foreignCount | belongsCount',
 *   'is_gt   |  current_index',
 *   'MostCountIndex1 | MostCountIndex2 | | MostCountIndex3 | ...',
 *   'MostCountValue1 | MostCountValue2 | | MostCountValue3| ...' ]
 *
 *   @returns true if voxel has been initialized (valid classes) false else
 */
bool convertVoxelToInt32(const panoptic_mapping::ClassVoxel& voxel,
                         std::vector<uint32_t>* data,
                         int serialize_top_n_counts) {
  size_t class_count = voxel.counts.size();
  // NOTE @zrene Count indices will be stored as 8bit int. This makes sure it
  // won't overflow. Kind of ugly but it should be unlikely to have more than
  // 257 classes
  CHECK_LT(class_count, 258);

  // Save how many classes are stored for this voxel.
  data->push_back(static_cast<uint32_t>(class_count));

  // Serialize belongs and foreign count
  data->push_back(static_cast<uint32_t>(voxel.belongs_count) |
                  static_cast<uint32_t>(voxel.foreign_count) << 16);

  data->push_back(static_cast<uint32_t>(voxel.is_gt) |
                  static_cast<uint32_t>(voxel.current_index) << 16);

  if (class_count == 0) {
    // This belongs_count and foreign is zero, voxel is not intialized.
    bool voxel_has_been_initialized =
        voxel.belongs_count != 0 || voxel.foreign_count != 0;
    return voxel_has_been_initialized;
  }

  // Store all_count_indices | all_count_values
  std::vector<uint8_t> indices_list;
  std::vector<Counter> score_counts;
  // Get top N indices + counts
  getTopNElems(voxel.counts, indices_list, score_counts,
                  serialize_top_n_counts);
  // Convert them to int32
  convert_vector_to_uint32<uint8_t>(indices_list, data, 8);
  convert_vector_to_uint32<Counter>(score_counts, data, COUNTER_SIZE_BITS);
  return true;
}

bool convertVoxelToInt32(const panoptic_mapping::ClassUncertaintyVoxel& voxel,
                         std::vector<uint32_t>* data,
                         int serialize_top_n_counts) {
  if (convertVoxelToInt32(static_cast<panoptic_mapping::ClassVoxel>(voxel),
                          data, serialize_top_n_counts)) {
    // Save uncertainty value
    data->push_back(static_cast<uint32_t>(voxel.uncertainty_value));
  }
}

bool readVoxelFromInt32(const std::vector<uint32_t>* data, size_t& data_idx,
                        int serialize_top_n_counts,
                        panoptic_mapping::ClassVoxel& voxel) {
  const uint32_t num_classes = data->at(data_idx++);

  const uint32_t bytes_1 = data->at(data_idx++);
  const uint32_t bytes_2 = data->at(data_idx++);
  voxel.foreign_count = static_cast<uint8_t>((bytes_1 & 0xFFFF0000) >> 16);
  voxel.belongs_count = static_cast<uint8_t>(bytes_1 & 0x0000FFFF);

  voxel.current_index = static_cast<uint8_t>((bytes_2 & 0xFFFF0000) >> 16);
  voxel.is_gt = static_cast<bool>(bytes_2 & 0x0000FFFF);

  if (num_classes == 0) {
    // This voxel has no information stored
    voxel.current_index = -1;
    // return if voxel was initialized
    return voxel.belongs_count != 0 || voxel.foreign_count != 0;
  }

  // Classes vector. Initialize with zeros
  voxel.counts.reserve(num_classes);
  for (int i = 0; i < num_classes; i++) {
    voxel.counts.push_back(0);
  }
  std::vector<uint8_t> indices;

  // Read indices. These are encoded in 8bit steps inside the int32
  uint32_t mask = ~static_cast<uint8_t>(0);
  for (int i = 0; i < serialize_top_n_counts; i++) {
    int byte_idx = (i % 4);
    int vector_idx = i / 4;
    uint8_t data_content =
        (data->at(data_idx + vector_idx) & (mask << byte_idx * 8)) >>
        byte_idx * 8;
    indices.push_back(data_content);
  }
  // data_idx posts to int32 storing counts now
  data_idx += std::ceil(serialize_top_n_counts / 4.0);

  // Read counts. These are encoded in <16|8> bit steps inside the int32
  std::vector<Counter> counts;
  constexpr uint32_t data_per_int = 32 / COUNTER_SIZE_BITS;
  mask = ~static_cast<Counter>(0);
  for (uint32_t i = 0; i < serialize_top_n_counts; i++) {
    uint32_t byte_idx = (i % data_per_int);
    uint32_t vector_idx = i / data_per_int;
    Counter data_content = (data->at(data_idx + vector_idx) &
                            (mask << byte_idx * COUNTER_SIZE_BITS)) >>
                           byte_idx * COUNTER_SIZE_BITS;
    counts.push_back(data_content);
  }

  // move data_idx to point to uncertainty
  data_idx +=
      std::ceil(serialize_top_n_counts / static_cast<float>(data_per_int));

  for (int i = 0; i < indices.size(); i++) {
    voxel.counts.at(indices.at(i)) = counts.at(i);
  }
  return true;
}

bool readVoxelFromInt32(const std::vector<uint32_t>* data, size_t& data_idx,
                        int serialize_top_n_counts,
                        panoptic_mapping::ClassUncertaintyVoxel& voxel) {
  if (readVoxelFromInt32(data, data_idx, serialize_top_n_counts,
                         static_cast<panoptic_mapping::ClassVoxel&>(voxel))) {
    voxel.uncertainty_value = data->at(data_idx++);
    return true;
  }
}

template <>
void mergeVoxelAIntoVoxelB<panoptic_mapping::ClassVoxel>(
    const panoptic_mapping::ClassVoxel& voxel_A,
    panoptic_mapping::ClassVoxel* voxel_B) {
  /*
   * This function is not implemented as it is currently only needed at compile
   * time.
   */
}

template <>
void mergeVoxelAIntoVoxelB<panoptic_mapping::ClassUncertaintyVoxel>(
    const panoptic_mapping::ClassUncertaintyVoxel& voxel_A,
    panoptic_mapping::ClassUncertaintyVoxel* voxel_B) {
  /*
   * This function is not implemented as it is currently only needed at compile
   * time.
   */
}

template <>
std::string getVoxelType<panoptic_mapping::ClassUncertaintyVoxel>() {
  return voxel_types::kClass;
}

template <>
std::string getVoxelType<panoptic_mapping::ClassVoxel>() {
  return voxel_types::kClass;
}
}  // namespace voxblox