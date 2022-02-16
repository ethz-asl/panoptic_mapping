#ifndef PANOPTIC_MAPPING_LABELS_RANGE_LABEL_HANDLER_H_
#define PANOPTIC_MAPPING_LABELS_RANGE_LABEL_HANDLER_H_

#include <string>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/labels/label_entry.h"
#include "panoptic_mapping/labels/label_handler_base.h"

namespace panoptic_mapping {

/**
 * @brief This basic label handler allocates labels in an integer range.
 */
class RangeLabelHandler : public LabelHandlerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Number of labels, initialises labels in range [0, num_labels - 1]
    int num_labels;
    Config() { setConfigName("RangeLabelHandler"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit RangeLabelHandler(const Config& config, bool print_config = true);
  ~RangeLabelHandler() override = default;

 private:
  const Config config_;
  static config_utilities::Factory::RegistrationRos<LabelHandlerBase,
                                                    RangeLabelHandler>
      registration_;
  void initialiseLabels();
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_LABELS_RANGE_LABEL_HANDLER_H_
