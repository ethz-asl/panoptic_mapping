#ifndef PANOPTIC_MAPPING_TOOLS_LOG_DATA_WRITER_H_
#define PANOPTIC_MAPPING_TOOLS_LOG_DATA_WRITER_H_

#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/tools/data_writer_base.h"

namespace panoptic_mapping {

/**
 * @brief Utility class that evaluates certain metrics during experiments and
 * writes them to a log file.
 */
class LogDataWriter : public DataWriterBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Target directory to write the output to. This needs to exist already.
    std::string output_directory = "";

    // File name of the log file without suffix.
    std::string file_name = "panoptic_mapping_data";

    // Which fields to evaluate Fields.
    bool evaluate_number_of_submaps = true;
    bool evaluate_number_of_active_submaps = true;
    bool evaluate_number_of_objects = true;

    Config() { setConfigName("LogDataWriter"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit LogDataWriter(const Config& config, bool print_config = true);
  ~LogDataWriter() override;

  void writeData(double time_stamp, const SubmapCollection& submaps) override;

 private:
  static config_utilities::Factory::RegistrationRos<DataWriterBase,
                                                    LogDataWriter>
      registration_;
  const Config config_;

 protected:
  // Data.
  bool is_setup_ = false;
  std::string output_path_;
  std::string outfile_name_;
  std::ofstream outfile_;
  std::vector<std::function<void(const SubmapCollection&)>> evaluations_;

  // Methods.
  virtual void setup();
  virtual void setupLogFile();
  virtual void setupEvaluations();
  void writeEntry(const std::string& value);

  // Evaluations. The data are always preceded by a separating comma.
  void evaluateNumberOfSubmaps(const SubmapCollection& submaps);
  void evaluateNumberOfActiveSubmaps(const SubmapCollection& submaps);
  void evaluateNumberOfObjects(const SubmapCollection& submaps);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_LOG_DATA_WRITER_H_
