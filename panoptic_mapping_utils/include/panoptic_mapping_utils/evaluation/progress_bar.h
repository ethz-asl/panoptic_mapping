#ifndef PANOPTIC_MAPPING_UTILS_EVALUATION_PROGRESS_BAR_H_
#define PANOPTIC_MAPPING_UTILS_EVALUATION_PROGRESS_BAR_H_

#include <cstdio>
#include <string>
#include <algorithm>

namespace panoptic_mapping {

// Print a progress bar to console for slow tasks.

class ProgressBar {
 public:
  explicit ProgressBar(int width = 80, char symbol = '=')
      : width_(width < 8 ? 8 : width), symbol_(symbol) {}
  virtual ~ProgressBar() = default;

  bool display(float percentage) const {
    percentage = std::min(std::max(percentage, 0.f), 1.f);
    const int value = static_cast<int>(percentage * 100.f);
    const int length = static_cast<int>(percentage * (width_ - 7));
    std::printf("\r%3d%% [%s%s]", value, std::string(length, symbol_).c_str(),
                std::string(width_ - 7 - length, ' ').c_str());
    if (value >= 100) {
      std::cout << std::endl;
      return true;  // returns whether progress finished.
    } else {
      std::fflush(stdout);
      return false;
    }
  }

 private:
  const char symbol_;
  const int width_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_UTILS_EVALUATION_PROGRESS_BAR_H_