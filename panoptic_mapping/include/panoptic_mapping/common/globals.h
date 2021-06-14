#ifndef PANOPTIC_MAPPING_COMMON_GLOBALS_H_
#define PANOPTIC_MAPPING_COMMON_GLOBALS_H_

#include <memory>
#include <utility>

#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/common/label_handler.h"

namespace panoptic_mapping {

/**
 * Utility class that provides an interface to globally used components of the
 * system.
 */
class Globals {
 public:
  Globals(std::shared_ptr<Camera> camera,
          std::shared_ptr<LabelHandler> label_handler)
      : camera_(std::move(camera)), label_handler_(std::move(label_handler)) {}
  virtual ~Globals() = default;

  // Access.
  const std::shared_ptr<Camera>& camera() const { return camera_; }
  const std::shared_ptr<LabelHandler>& labelHandler() const {
    return label_handler_;
  }

 private:
  // Components.
  std::shared_ptr<Camera> camera_;
  std::shared_ptr<LabelHandler> label_handler_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_GLOBALS_H_
