#ifndef PANOPTIC_MAPPING_COMMON_GLOBALS_H_
#define PANOPTIC_MAPPING_COMMON_GLOBALS_H_

#include <memory>
#include <utility>

#include "panoptic_mapping/common/camera.h"
#include "panoptic_mapping/labels/label_handler_base.h"

namespace panoptic_mapping {

/**
 * @brief Utility class that provides an interface to globally used components
 * of the system.
 */
class Globals {
 public:
  Globals(std::shared_ptr<Camera> camera,
          std::shared_ptr<LabelHandlerBase> label_handler)
      : camera_(std::move(camera)), label_handler_(std::move(label_handler)) {}
  virtual ~Globals() = default;

  // Access.
  const std::shared_ptr<Camera>& camera() const { return camera_; }
  const std::shared_ptr<LabelHandlerBase>& labelHandler() const {
    return label_handler_;
  }
  const Transformation& getT_W_C() const { return T_W_C_; }
  void setT_W_C(const Transformation& T_W_C) { T_W_C_ = T_W_C; }

 private:
  // Components.
  std::shared_ptr<Camera> camera_;
  std::shared_ptr<LabelHandlerBase> label_handler_;
  Transformation T_W_C_;  // Pose of the camera in the global frame.
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_GLOBALS_H_
