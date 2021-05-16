#include "panoptic_mapping/tools/data_writer.h"

#include <sys/stat.h>

#include <fstream>
#include <iomanip>
#include <string>
#include <unordered_set>

namespace panoptic_mapping {

void DataWriter::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("output_directory", &output_directory);
  setupParam("file_name", &file_name);
  setupParam("evaluate_number_of_submaps", &evaluate_number_of_submaps);
  setupParam("evaluate_numer_of_objects", &evaluate_numer_of_objects);
}

void DataWriter::Config::checkParams() const {
  // Check the specified path exists.
  struct stat buffer;
  checkParamCond(stat(output_directory.c_str(), &buffer) == 0,
                 "Output directory '" + output_directory + "' does not exist.");
}

DataWriter::DataWriter(const Config& config) : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Setup output file.
  outfile_name_ = config_.file_name;
  if (outfile_name_.back() != '_' && !outfile_name_.empty()) {
    outfile_name_.append("_");
  }
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream timestamp;
  timestamp << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
  outfile_name_.append(timestamp.str());
  outfile_name_ = config_.output_directory + "/" + outfile_name_ + ".csv";
  outfile_.open(outfile_name_);
  if (!outfile_.is_open()) {
    LOG(ERROR) << "Could not open data output file '" << outfile_name_ << "'.";
  }

  // Setup evaluations.
  setupEvaluations();
  LOG_IF(INFO, config_.verbosity >= 1)
      << "Started writing to data file '" << outfile_name_ << "'.";
}

DataWriter::~DataWriter() {
  // NOTE(schmluk): Apparently the destructor doesn't get called from ROS usage.
  if (outfile_.is_open()) {
    outfile_.close();
    LOG_IF(INFO, config_.verbosity >= 1)
        << "Finished writing data to file '" << outfile_name_ << "'.";
  }
}

void DataWriter::setupEvaluations() {
  outfile_ << "Timestamp [s]";

  // Setup all data headers [with units] and evaluation functions to be used.
  if (config_.evaluate_number_of_submaps) {
    writeEntry("NoSubmaps [1]");
    evaluations_.emplace_back(&DataWriter::evaluateNumberOfSubmaps);
  }
  if (config_.evaluate_number_of_active_submaps) {
    writeEntry("NoActiveSubmaps [1]");
    evaluations_.emplace_back(&DataWriter::evaluateNumberOfActiveSubmaps);
  }
  if (config_.evaluate_numer_of_objects) {
    writeEntry("NoObjects [1]");
    evaluations_.emplace_back(&DataWriter::evaluateNumberOfObjects);
  }

  outfile_ << std::endl;
}

void DataWriter::writeEntry(const std::string& value) {
  // Include leading separators (commas) here after timestamp.
  outfile_ << "," << value;
}

void DataWriter::writeData(double time_stamp, const SubmapCollection& submaps) {
  // Log the timestamp.
  outfile_ << std::to_string(time_stamp);

  // Perform all evaluations.
  for (auto& evaluation : evaluations_) {
    (this->*evaluation)(submaps);
  }

  // Finish.
  outfile_ << std::endl;
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Wrote data entry for time '" << time_stamp << "'.";
}

void DataWriter::evaluateNumberOfSubmaps(const SubmapCollection& submaps) {
  writeEntry(std::to_string(submaps.size()));
}
void DataWriter::evaluateNumberOfActiveSubmaps(
    const SubmapCollection& submaps) {
  int active_submaps = 0;
  for (const auto& submap : submaps) {
    if (submap->isActive()) {
      active_submaps++;
    }
  }
  writeEntry(std::to_string(active_submaps));
}
void DataWriter::evaluateNumberOfObjects(const SubmapCollection& submaps) {
  std::unordered_set<int> instance_ids;
  for (const auto& submap_ptr : submaps) {
    // Only count ids > 0 since -1 and similar is used for invalid or other
    // instances. Only count submaps that are observed present.
    if (submap_ptr->getInstanceID() < 0) {
      continue;
    }
    if (submap_ptr->getChangeState() == ChangeState::kAbsent ||
        submap_ptr->getChangeState() == ChangeState::kUnobserved) {
      continue;
    }
    instance_ids.insert(submap_ptr->getInstanceID());
  }
  writeEntry(std::to_string(instance_ids.size()));
}

}  // namespace panoptic_mapping
