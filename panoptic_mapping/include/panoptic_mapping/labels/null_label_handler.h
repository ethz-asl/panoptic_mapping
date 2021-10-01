#ifndef PANOPTIC_MAPPING_LABELS_NULL_LABEL_HANDLER_H_
#define PANOPTIC_MAPPING_LABELS_NULL_LABEL_HANDLER_H_

#include <string>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/labels/label_handler_base.h"

namespace panoptic_mapping {

/**
 * @brief This label handler does not read any labels.
 */
class NullLabelHandler : public LabelHandlerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    Config() { setConfigName("NullLabelHandler"); }

   protected:
    void setupParamsAndPrinting() override;
  };

  explicit NullLabelHandler(const Config& config);
  ~NullLabelHandler() override = default;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<LabelHandlerBase,
                                                    NullLabelHandler>
      registration_[[gnu::used]];
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_LABELS_NULL_LABEL_HANDLER_H_
