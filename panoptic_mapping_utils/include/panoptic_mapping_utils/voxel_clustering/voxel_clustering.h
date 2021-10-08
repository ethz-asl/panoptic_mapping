#include <queue>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"

using Instance = std::vector<voxblox::GlobalIndex>;

namespace panoptic_mapping {
namespace voxel_clustering {
enum ScoringMethod { ENTROPY, UNCERTAINTY, BELONGS_PROBABILITY, SIZE };

/**
 * Contains information for combined voxels.
 */
struct InstanceInfo {
  int size;
  float mean_score;
  int assigned_class;
  Instance instance;

  InstanceInfo(int size, float mean_score, int assigned_class)
      : size(size), mean_score(mean_score), assigned_class(assigned_class) {}

  bool operator<(const InstanceInfo& info) const {
    return mean_score < info.mean_score;
  }
};

/**
 *  Performs wavefront exploration to find connected regions (by semantic class)
 * in the given submap.
 *
 * @param seed Seed Voxel Index to start wavefront exploration
 * @param closed_list Voxels that have been checked already. Will be filled up
 * by algorithm.
 * @param map The submap to use
 * @param result After function ended, this contains all voxels that are
 * connected to the seed and have the same semantic class
 * @return false if operation failed
 */
bool wavefrontExploration(const voxblox::GlobalIndex& seed,
                          voxblox::LongIndexSet& closed_list,
                          const panoptic_mapping::Submap* map,
                          Instance* result);

/**
 * Performs wavefront exploration for each seed in the seeds vector in order to find all instances that belong
 * to the given seeds
 * @param seeds List of voxel indices
 * @param map Submap which should be used
 * @param instances Vector where all found instances should be stored
 */
bool getConnectedInstancesForSeeds(std::vector<voxblox::GlobalIndex>& seeds,
                                   const panoptic_mapping::Submap* map,
                                   std::vector<Instance>* instances);

/**
 * Calculates all connected instances in a given submap.
 * @param map  Submap to use
 * @param instances Contains all connected instances that have been found
 */
bool getAllConnectedInstances(const panoptic_mapping::Submap* map,
                              std::vector<Instance>* instances);

/**
 * Scores a given list of instances using a scoring function definied by the scoring method
 * @param map Submap to use
 * @param instances Instances to score
 * @param include_groundtruth If false, ignore voxels that are labeled as groundtruth for scoring
 * @param scoring_method What scoring functions to use
 * @param scored_instances PriorityQueue containing scored instances. Use .top() to get the instance with highest score
 */
void scoreConnectedInstances(
    const panoptic_mapping::Submap* map, const std::vector<Instance>* instances,
    bool include_groundtruth, ScoringMethod scoring_method,
    std::priority_queue<InstanceInfo>* scored_instances);
}  // namespace voxel_clustering
}  // namespace panoptic_mapping