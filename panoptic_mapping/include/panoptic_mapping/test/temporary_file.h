#ifndef PANOPTIC_MAPPING_TEST_TEMPORARY_FILE_H_
#define PANOPTIC_MAPPING_TEST_TEMPORARY_FILE_H_

#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include <experimental/filesystem>

namespace panoptic_mapping {
namespace test {

/**
 * @brief Opens and manages the lifetime of a temporary file for testing.
 *
 */
class TempFile {
 public:
  explicit TempFile(const std::string& name = "") {
    // For Ubuntu /tmp/ should always exist and no subdirectories are used.
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream ss;
    ss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    file_name_ = "/tmp/" + name + ss.str() + ".tmp";
    stream_.open(file_name_, std::fstream::in | std::fstream::out |
                                 std::fstream::trunc | std::fstream::binary);
  }
  ~TempFile() {
    // Close and remove the temporary file.
    if (stream_.is_open()) {
      stream_.close();
    }
    if (std::experimental::filesystem::exists(file_name_)) {
      std::experimental::filesystem::remove(file_name_);
    }
  }
  operator bool() const { return stream_.is_open(); }
  std::fstream& stream() { return stream_; }
  const std::fstream& stream() const { return stream_; }

 private:
  std::fstream stream_;
  std::string file_name_;
};

}  // namespace test
}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_TEST_TEMPORARY_FILE_H_
