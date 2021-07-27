#ifndef PANOPTIC_MAPPING_TOOLS_DATA_WRITER_H_
#define PANOPTIC_MAPPING_TOOLS_DATA_WRITER_H_

#include <fstream>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap_collection.h"

namespace panoptic_mapping {

/**
 * @brief Utility class that evaluates certain metrics during experiments and
 * writes them to a log file.
 */

class DataWriter {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Output.
    std::string output_directory = "";  // Target directory needs to exist.
    std::string file_name = "panoptic_mapping_data";  // Filename without suffix

    // Evaluation Fields.
    bool evaluate_number_of_submaps = true;
    bool evaluate_number_of_active_submaps = true;
    bool evaluate_numer_of_objects = true;
    int store_map_every_n_frames = 0;

    Config() { setConfigName("DataWriter"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit DataWriter(const Config& config);
  virtual ~DataWriter();

  void writeData(double time_stamp, const SubmapCollection& submaps);

 private:
  const Config config_;

  // Data.
  std::string output_path_;
  std::string outfile_name_;
  std::ofstream outfile_;
  std::vector<void (DataWriter::*)(const SubmapCollection&)> evaluations_;

  // Tracking variables.
  int store_submap_frame_ = 0;
  int store_submap_counter_ = 0;

  // Methods.
  void setupEvaluations();
  void writeEntry(const std::string& value);

  // Evaluations. The data are always preceded by a separating comma.
  void evaluateNumberOfSubmaps(const SubmapCollection& submaps);
  void evaluateNumberOfActiveSubmaps(const SubmapCollection& submaps);
  void evaluateNumberOfObjects(const SubmapCollection& submaps);
  void storeSubmaps(const SubmapCollection& submaps);
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_DATA_WRITER_H_
