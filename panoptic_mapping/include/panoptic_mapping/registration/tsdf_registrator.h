#ifndef PANOPTIC_MAPPING_REGISTRATION_TSDF_REGISTRATOR_H_
#define PANOPTIC_MAPPING_REGISTRATION_TSDF_REGISTRATOR_H_

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/core/common.h"

namespace panoptic_mapping {

class Submap;
class SubmapCollection;

/**
 * This class directly compares TSDF volumes of two submaps with each other,
 * using the registration constraints adapted from voxgraph to detect matches
 * and solve for transformations using ceres.
 */

class TsdfRegistrator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    double min_voxel_weight = 1e-6;
    double error_threshold = -1;  // m, negative values are multiples of
    // the average voxel_size.
    bool weight_error_by_tsdf_weight = true;
    int min_number_of_matching_points = 50;
    double match_rejection_percentage = 0.9;  // Percentage of points required
    // within the error threshold.

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Change detection data is stored with each submap.
  struct ChangeDetectionData {
    bool is_matched = false;
    int matching_submap_id = 0;
  };

  struct ChangeDetectionResult {
    unsigned int number_of_checked_points = 0;   // Checked points of reference.
    unsigned int number_of_observed_points = 0;  // Points observed in other.
    unsigned int points_within_threshold = 0;
    double distance_rmse = 0.0;
  };

  explicit TsdfRegistrator(const Config& config);
  virtual ~TsdfRegistrator() = default;

  // Check whether there is significant difference between the two submaps.
  void computeIsoSurfacePoints(Submap* submap) const;
  ChangeDetectionResult computeSurfaceDifference(Submap* reference,
                                                 Submap* other) const;
  void checkSubmapCollectionForChange(const SubmapCollection& submaps);
  void resetChangeTracking();

 private:
  bool isMatch(const ChangeDetectionResult& change_data) const;

 private:
  const Config config_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_REGISTRATION_TSDF_REGISTRATOR_H_
