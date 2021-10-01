#include "panoptic_mapping/labels/csv_label_handler.h"

#include <sys/stat.h>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/csv.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<LabelHandlerBase, CsvLabelHandler>
    CsvLabelHandler::registration_("csv");

void CsvLabelHandler::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("file_name", &file_name);
}

void CsvLabelHandler::Config::checkParams() const {
  // Check the specified file exists.
  struct stat buffer;
  checkParamCond(stat(file_name.c_str(), &buffer) == 0,
                 "Target file '" + file_name + "' does not exist.");
}

CsvLabelHandler::CsvLabelHandler(const Config& config, bool print_config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  // Setup the labels from csv file.
  readLabelsFromFile();
}

void CsvLabelHandler::readLabelsFromFile() {
  // NOTE(schmluk): Assumes fixed header names in the target file. Reading
  // exceptions should be handled by the CSVReader. Read all optional columns
  // and write the present ones. All header columns need to be present at the
  // moment.
  io::CSVReader<8> in(config_.file_name);
  in.read_header(io::ignore_extra_column, "InstanceID", "ClassID", "PanopticID",
                 "R", "G", "B", "Name", "Size");

  bool read_row = true;
  std::vector<float> field_count(6, 0.f);
  int missed_count = -1;  // The header is also counter.
  while (read_row) {
    std::string name, size;
    int inst = -1, cls = -1, pan = -1, r = -1, g = -1, b = -1;
    read_row = in.read_row(inst, cls, pan, r, g, b, name, size);

    // Write all found values to the label.
    LabelEntry label;
    if (inst != -1) {
      label.segmentation_id = inst;
      field_count[0] += 1.f;
    } else {
      missed_count++;
      continue;
    }
    if (cls != -1) {
      label.class_id = cls;
      field_count[1] += 1.f;
    }
    if (pan != -1) {
      label.label = pan ? PanopticLabel::kInstance : PanopticLabel::kBackground;
      field_count[2] += 1.f;
    }
    if (!name.empty()) {
      label.name = name;
      field_count[3] += 1.f;
    }
    if (!size.empty()) {
      label.size = size;
      field_count[4] += 1.f;
    }
    if (r != -1 && g != -1 && b != -1) {
      label.color = voxblox::Color(r, g, b);
      field_count[5] += 1.f;
    }
    labels_[inst] = std::make_unique<LabelEntry>(label);
  }

  // Cehck all labels valid.
  if (missed_count) {
    LOG(ERROR)
        << missed_count
        << " labels could not be read, the 'InstanceID' field needs to be set!";
  }

  // Required fields.
  const size_t num_labels = labels_.size();
  const std::vector<std::string> field_names = {
      "InstanceID", "ClassID", "PanopticID", "RGB", "Name", "Size"};
  for (size_t i = 1; i < 3; ++i) {
    if (field_count[i] < num_labels) {
      LOG(WARNING) << "Required field '" << field_names[i]
                   << "' is not set for " << num_labels - field_count[i]
                   << " entries.";
    }
  }

  // Logging.
  std::stringstream info;
  info << "Read " << num_labels << "/" << num_labels + missed_count
       << " labels from '" << config_.file_name << ".";
  if (config_.verbosity >= 3) {
    info << " Read fields: ";
    for (size_t i = 0; i < 6; ++i) {
      info << "\n  -" << field_names[i]
           << std::string(12 - field_names[i].length(), ' ') << "("
           << std::fixed << std::setprecision(1)
           << field_count[i] / static_cast<float>(num_labels) * 100.f << "%)";
    }
  }
  LOG_IF(INFO, config_.verbosity >= 1) << info.str();
}

}  // namespace panoptic_mapping
