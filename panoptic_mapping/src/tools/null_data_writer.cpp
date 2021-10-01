#include "panoptic_mapping/tools/null_data_writer.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<DataWriterBase, NullDataWriter>
    NullDataWriter::registration_("null");

void NullDataWriter::Config::setupParamsAndPrinting() {
  printText("The null data writer does not write any data.");
}

}  // namespace panoptic_mapping
