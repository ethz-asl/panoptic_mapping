#ifndef PANOPTIC_MAPPING_TOOLS_EVALUATION_DATA_WRITER_H_
#define PANOPTIC_MAPPING_TOOLS_EVALUATION_DATA_WRITER_H_

#include <fstream>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/tools/log_data_writer.h"

namespace panoptic_mapping {

/**
 * @brief Utility class that evaluates allocates a full evaluation folder to
 * this run. In that folder, certain evaluated metrics will be logged and the
 * map stored periodically for further evaluations.
 */
class EvaluationDataWriter : public LogDataWriter {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Log data writer config.
    LogDataWriter::Config log_data_writer_config;

    // Evaluation Fields.
    int store_map_every_n_frames = 0;

    Config() { setConfigName("EvaluationDataWriter"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit EvaluationDataWriter(const Config& config);
  ~EvaluationDataWriter() override = default;

 private:
  static config_utilities::Factory::RegistrationRos<DataWriterBase,
                                                    EvaluationDataWriter>
      registration_;
  const Config config_;

  // Tracking variables.
  int store_submap_frame_ = 0;
  int store_submap_counter_ = 0;

  // Methods.
  void setupLogFile() override;
  void setupEvaluations() override;

  // Evaluations. The data are always preceded by a separating comma.
  void storeSubmaps(const SubmapCollection& submaps);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_EVALUATION_DATA_WRITER_H_
