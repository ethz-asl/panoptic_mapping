#include "panoptic_mapping/tools/log_data_writer.h"

#include <sys/stat.h>

#include <fstream>
#include <iomanip>
#include <string>
#include <unordered_set>

#include <experimental/filesystem>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<DataWriterBase, LogDataWriter>
    LogDataWriter::registration_("log");

void LogDataWriter::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("output_directory", &output_directory);
  setupParam("file_name", &file_name);
  setupParam("evaluate_number_of_submaps", &evaluate_number_of_submaps);
  setupParam("evaluate_number_of_objects", &evaluate_number_of_objects);
}

void LogDataWriter::Config::checkParams() const {
  // Check the specified path exists.
  struct stat buffer;
  checkParamCond(
      stat(output_directory.c_str(), &buffer) == 0,
      "'output_directory' '" + output_directory + "' does not exist.");
}

LogDataWriter::LogDataWriter(const Config& config, bool print_config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
}

LogDataWriter::~LogDataWriter() {
  // NOTE(schmluk): Apparently the destructor doesn't get called from ROS usage.
  if (outfile_.is_open()) {
    outfile_.close();
    LOG_IF(INFO, config_.verbosity >= 1)
        << "Finished writing data to file '" << outfile_name_ << "'.";
  }
}

void LogDataWriter::setup() {
  if (is_setup_) {
    return;
  }
  // Setup the output file.
  setupLogFile();

  // Setup what to evaluate and log headers.
  outfile_ << "Timestamp [s]";
  setupEvaluations();
  outfile_ << std::endl;

  // Finish.
  LOG_IF(INFO, config_.verbosity >= 1)
      << "Started writing to data file '" << outfile_name_ << "'.";
  is_setup_ = true;
}

void LogDataWriter::setupLogFile() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream timestamp;
  timestamp << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");

  output_path_ = config_.output_directory;

  // Setup the logfile with the timestamp.
  outfile_name_ = config_.file_name;
  if (outfile_name_.back() != '_' && !outfile_name_.empty()) {
    outfile_name_.append("_");
  }
  outfile_name_.append(timestamp.str());
  outfile_name_ = config_.output_directory + "/" + outfile_name_ + ".csv";

  // Setup output file.
  outfile_.open(outfile_name_);
  if (!outfile_.is_open()) {
    LOG(ERROR) << "Could not open data output file '" << outfile_name_ << "'.";
  }
}

void LogDataWriter::setupEvaluations() {
  // Setup all data headers [with units] and evaluation functions to be used.
  if (config_.evaluate_number_of_submaps) {
    writeEntry("NoSubmaps [1]");
    evaluations_.emplace_back([this](const SubmapCollection& submaps) {
      this->evaluateNumberOfSubmaps(submaps);
    });
  }
  if (config_.evaluate_number_of_active_submaps) {
    writeEntry("NoActiveSubmaps [1]");
    evaluations_.emplace_back([this](const SubmapCollection& submaps) {
      this->evaluateNumberOfActiveSubmaps(submaps);
    });
  }
  if (config_.evaluate_number_of_objects) {
    writeEntry("NoObjects [1]");
    evaluations_.emplace_back([this](const SubmapCollection& submaps) {
      this->evaluateNumberOfObjects(submaps);
    });
  }
}

void LogDataWriter::writeEntry(const std::string& value) {
  // Include leading separators (commas) here after timestamp.
  outfile_ << "," << value;
}

void LogDataWriter::writeData(double time_stamp,
                              const SubmapCollection& submaps) {
  // Lazy initialization of the log files.
  if (!is_setup_) {
    setup();
  }

  // Log the timestamp.
  outfile_ << std::to_string(time_stamp);

  // Perform all evaluations.
  for (const auto& evaluation : evaluations_) {
    evaluation(submaps);
  }

  // Finish. This flushes the file buffer.
  outfile_ << std::endl;
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Wrote data entry for time '" << time_stamp << "'.";
}

void LogDataWriter::evaluateNumberOfSubmaps(const SubmapCollection& submaps) {
  writeEntry(std::to_string(submaps.size()));
}

void LogDataWriter::evaluateNumberOfActiveSubmaps(
    const SubmapCollection& submaps) {
  int active_submaps = 0;
  for (const Submap& submap : submaps) {
    if (submap.isActive()) {
      active_submaps++;
    }
  }
  writeEntry(std::to_string(active_submaps));
}

void LogDataWriter::evaluateNumberOfObjects(const SubmapCollection& submaps) {
  std::unordered_set<int> instance_ids;
  for (const Submap& submap : submaps) {
    // Only count ids > 0 since -1 and similar is used for invalid or other
    // instances. Only count submaps that are observed present.
    if (submap.getInstanceID() < 0) {
      continue;
    }
    if (submap.getChangeState() == ChangeState::kAbsent ||
        submap.getChangeState() == ChangeState::kUnobserved) {
      continue;
    }
    instance_ids.insert(submap.getInstanceID());
  }
  writeEntry(std::to_string(instance_ids.size()));
}

}  // namespace panoptic_mapping
