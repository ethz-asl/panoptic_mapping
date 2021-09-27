#ifndef PANOPTIC_MAPPING_TOOLS_NULL_DATA_WRITER_H_
#define PANOPTIC_MAPPING_TOOLS_NULL_DATA_WRITER_H_

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/tools/data_writer_base.h"

namespace panoptic_mapping {

/**
 * @brief The null data writer is a default modulethat does not write any data.
 */
class NullDataWriter : public DataWriterBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    Config() { setConfigName("NullDataWriter"); }

   protected:
    void setupParamsAndPrinting() override;
  };

  explicit NullDataWriter(const Config& config) : config_(config) {}
  virtual ~NullDataWriter() = default;

  // The null data writer does not write any data.
  void writeData(double time_stamp, const SubmapCollection& submaps) override{};

 private:
  static config_utilities::Factory::RegistrationRos<DataWriterBase,
                                                    NullDataWriter>
      registration_;
  const Config config_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_NULL_DATA_WRITER_H_
