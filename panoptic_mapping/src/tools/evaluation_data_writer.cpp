#include "panoptic_mapping/tools/evaluation_data_writer.h"

#include <sys/stat.h>

#include <fstream>
#include <iomanip>
#include <string>
#include <unordered_set>

#include <experimental/filesystem>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<DataWriterBase, EvaluationDataWriter>
    EvaluationDataWriter::registration_("evaluation");

void EvaluationDataWriter::Config::checkParams() const {}

void EvaluationDataWriter::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("log_data_writer_config", &log_data_writer_config);
  setupParam("store_map_every_n_frames", &store_map_every_n_frames);
}

EvaluationDataWriter::EvaluationDataWriter(const Config& config)
    : config_(config.checkValid()),
      LogDataWriter(config.log_data_writer_config, false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void EvaluationDataWriter::setupLogFile() {
  const bool create_directory = config_.store_map_every_n_frames > 0;
  if (create_directory) {
    // Setup a single directory named with the timestamp.
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream timestamp;
    timestamp << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
    output_path_ =
        config_.log_data_writer_config.output_directory + "/" + timestamp.str();
    if (!std::experimental::filesystem::create_directory(output_path_)) {
      LOG(ERROR) << "Could not create output directory '" << output_path_
                 << "'.";
    }
    outfile_name_ = output_path_ + "/" +
                    (config_.log_data_writer_config.file_name.empty()
                         ? "log"
                         : config_.log_data_writer_config.file_name) +
                    ".csv";
    // Setup output file.
    outfile_.open(outfile_name_);
    if (!outfile_.is_open()) {
      LOG(ERROR) << "Could not open data output file '" << outfile_name_
                 << "'.";
    }
  } else {
    // Setup the standard log file.
    LogDataWriter::setupLogFile();
  }
}

void EvaluationDataWriter::setupEvaluations() {
  LogDataWriter::setupEvaluations();
  // Additional evaluations of the evaluation writer.
  if (config_.store_map_every_n_frames > 0) {
    writeEntry("SavedMapName [-]");
    evaluations_.emplace_back([this](const SubmapCollection& submaps) {
      this->storeSubmaps(submaps);
    });
  }
}

void EvaluationDataWriter::storeSubmaps(const SubmapCollection& submaps) {
  store_submap_frame_++;
  if (store_submap_frame_ < config_.store_map_every_n_frames) {
    writeEntry("");
    return;
  }
  store_submap_frame_ = 0;
  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0') << store_submap_counter_;
  store_submap_counter_++;
  submaps.saveToFile(output_path_ + "/" + ss.str());
  writeEntry(ss.str());
}

}  // namespace panoptic_mapping
