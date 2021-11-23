#ifndef PANOPTIC_MAPPING_TOOLS_COLORING_H_
#define PANOPTIC_MAPPING_TOOLS_COLORING_H_

#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {
/**
 * @brief Maps a value in [0, 1] to a color from red (1) via yellow (0.5) to
 * green (1).
 *
 * @param value Input value in [0, 1]
 * @return Color
 */
inline Color redToGreenGradient(const float value) {
  Color color;
  color.b = 0u;
  if (value > 0.5f) {
    color.r = ((1.f - value) * 2.f * 255.f);
    color.g = 255u;
  } else {
    color.r = 255u;
    color.g = (value * 2.f * 255.f);
  }
  return color;
}

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TOOLS_COLORING_H_
