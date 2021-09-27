#ifndef PANOPTIC_MAPPING_TOOLS_DATA_WRITER_BASE_H_
#define PANOPTIC_MAPPING_TOOLS_DATA_WRITER_BASE_H_

#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * @brief Interface for a utility class that evaluates certain metrics during
 * experiments and writes them to a log file.
 */
class DataWriterBase {
 public:
  DataWriterBase() = default;
  virtual ~DataWriterBase() = default;

  virtual void writeData(double time_stamp,
                         const SubmapCollection& submaps) = 0;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_DATA_WRITER_BASE_H_
