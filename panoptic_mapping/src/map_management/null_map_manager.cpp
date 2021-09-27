#include "panoptic_mapping/map_management/null_map_manager.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<MapManagerBase, NullMapManager>
    NullMapManager::registration_("null");

void NullMapManager::Config::setupParamsAndPrinting() {
  printText("The null map manager does not execute any actions.");
}

}  // namespace panoptic_mapping
