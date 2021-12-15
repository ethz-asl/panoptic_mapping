#ifndef PANOPTIC_MAPPING_TOOLS_SERIALIZATION_H_
#define PANOPTIC_MAPPING_TOOLS_SERIALIZATION_H_

#include <memory>
#include <utility>

#include <voxblox/Block.pb.h>

#include "panoptic_mapping/Submap.pb.h"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/class_layer.h"
#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

/**
 * Various serialization and de-serialization tools.
 */

// To and from uint32_t serialization.
inline uint32_t int32FromTwoInt16(uint16_t first, uint16_t second) {
  return ((static_cast<uint32_t>(first) << 16) + second);
}

inline uint16_t int16FromTwoInt8(uint8_t first, uint8_t second) {
  return ((static_cast<uint16_t>(first) << 8) + second);
}

inline std::pair<uint16_t, uint16_t> twoInt16FromInt32(uint32_t data) {
  return {static_cast<uint16_t>(data >> 16),
          static_cast<uint16_t>(data & 0xFFFF)};
}

inline std::pair<uint8_t, uint8_t> twoInt8FromInt16(uint16_t data) {
  return {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
}

template <typename T>
inline uint32_t int32FromX32(T data) {
  if (sizeof(T) != sizeof(uint32_t)) {
    LOG(WARNING) << "Can not serialize '" << typeid(T).name() << "' of size '"
                 << sizeof(T) << "' to uint32_t of size 32.";
    return 0;
  }
  uint32_t result;
  memcpy(&result, &data, sizeof(uint32_t));
  return result;
}

template <typename T>
inline T x32FromInt32(uint32_t data) {
  T result;
  if (sizeof(T) != sizeof(uint32_t)) {
    LOG(WARNING) << "Can not deserialize '" << typeid(T).name() << "' of size '"
                 << sizeof(T) << "' from uint32_t of size 32.";
    return result;
  }
  memcpy(&result, &data, sizeof(uint32_t));
  return result;
}

std::unique_ptr<ClassLayer> loadClassLayerFromStream(
    const SubmapProto& submap_proto, std::istream* proto_file_ptr,
    uint64_t* tmp_byte_offset_ptr);

bool saveClassLayerToStream(const ClassLayer& layer);

bool loadClassBlocksFromStream(const SubmapProto& submap_proto,
                               std::istream* proto_file_ptr,
                               uint64_t* tmp_byte_offset_ptr,
                               ClassLayer* layer);

bool isCompatible(const voxblox::BlockProto& block_proto,
                  const ClassLayer& layer);

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_SERIALIZATION_H_
