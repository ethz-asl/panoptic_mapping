#ifndef PANOPTIC_MAPPING_TEST_UTILS_H
#define PANOPTIC_MAPPING_TEST_UTILS_H
#include <random>
#include "panoptic_mapping/common/common.h"

const int kVoxelsPerSide = 16;
const int kNumClasses = 40;
const float kVoxelSize = 0.1;
const voxblox::Point origin(0, 0, 0);
const int k_num_classes = 40;
int top_n_to_serialize = 12;
const float kTol = 0.00001;

/** HELPER FUNCTIONS **/
inline void initializeVoxel(panoptic_mapping::ClassVoxel* voxel,
                            int num_classes) {
    for (int i = 0; i < num_classes; i++) {
        voxel->counts.push_back(0);
    }
}

inline void checkVoxelEqual(const panoptic_mapping::ClassVoxel& v1,
                            const panoptic_mapping::ClassVoxel& v2) {
    EXPECT_EQ(v1.is_groundtruth, v2.is_groundtruth);
    EXPECT_EQ(v1.current_index, v2.current_index);
    EXPECT_EQ(v1.belongs_count, v2.belongs_count);
    EXPECT_EQ(v1.foreign_count, v2.foreign_count);
    EXPECT_EQ(v1.counts.size(), v2.counts.size());
    if (v1.counts.empty()) {
        return;
    }
    // Only check if top_n counts match:
    std::priority_queue<std::pair<int, int>> class_idx_to_count_1;
    for (int i = 0; i < v1.counts.size(); ++i) {
        class_idx_to_count_1.push(std::pair<int, int>(v1.counts.at(i), i));
    }
    std::priority_queue<std::pair<int, int>> class_idx_to_count_2;
    for (int i = 0; i < v2.counts.size(); ++i) {
        class_idx_to_count_2.push(std::pair<int, int>(v2.counts.at(i), i));
    }
    for (int i = 0; i < top_n_to_serialize; ++i) {
        EXPECT_EQ(class_idx_to_count_1.top().first,
                  class_idx_to_count_2.top().first);
        EXPECT_EQ(class_idx_to_count_1.top().second,
                  class_idx_to_count_2.top().second);
        class_idx_to_count_1.pop();
        class_idx_to_count_2.pop();
    }
}
inline void checkVoxelEqual(
        const panoptic_mapping::TsdfVoxel& v1,
        const panoptic_mapping::TsdfVoxel& v2) {
    EXPECT_NEAR(v1.distance, v2.distance, kTol);
    EXPECT_NEAR(v1.weight, v2.weight, kTol);
    EXPECT_EQ(v1.color.r, v2.color.r);
    EXPECT_EQ(v1.color.g, v2.color.g);
    EXPECT_EQ(v1.color.b, v2.color.b);
}

inline void checkVoxelEqual(
        const panoptic_mapping::ClassUncertaintyVoxel& v1,
        const panoptic_mapping::ClassUncertaintyVoxel& v2) {
    checkVoxelEqual(static_cast<panoptic_mapping::ClassVoxel>(v1),
                    static_cast<panoptic_mapping::ClassVoxel>(v2));
    EXPECT_EQ(v1.uncertainty_value, v2.uncertainty_value);
}

template <typename T>
inline void checkBlockEqual(const voxblox::Block<T>* blk1,
                            const voxblox::Block<T>* blk2) {
    for (int i = 0; i < blk1->num_voxels(); i++) {
        checkVoxelEqual(blk1->getVoxelByLinearIndex(i),
                        blk2->getVoxelByLinearIndex(i));
    }
}

/**
 * Randomly assigns values to the voxels.
 */
void loadRandomVoxels(panoptic_mapping::TsdfVoxel* tsdf_voxel, panoptic_mapping::ClassUncertaintyVoxel* class_voxel) {
    std::random_device
            rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 1.0);
    bool skip_voxel = dis(gen) > 0.8;  // Leave 20% uninitialized
    if (skip_voxel) {
        return;
    }

    // TSDF Voxel
    tsdf_voxel->distance = static_cast<float>(dis(gen));
    tsdf_voxel->weight = static_cast<float>(dis(gen));
    tsdf_voxel->color = panoptic_mapping::Color(static_cast<uint8_t>(dis(gen) * 255),
                              static_cast<uint8_t>(dis(gen) * 255),
                              static_cast<uint8_t>(dis(gen) * 255));

    if (!class_voxel) {
        return;
    }
    // Class Voxel
    bool use_binary = dis(gen) > 0.8;  // Use binary 20% of the time

    if (use_binary) {
        int num_votes = static_cast<int>(dis(gen) * 100) + 1;
        for (int vote = 0; vote < num_votes; vote++) {
            classVoxelIncrementBinary(class_voxel, dis(gen) > 0.5);
        }
    } else {
        // Initialize counts
        for (int i = 0; i < k_num_classes; ++i) class_voxel->counts.push_back(0);
        classVoxelIncrementClass(class_voxel, 0);

        // Make sure we have unique top 3 counts
        std::vector<int> votes;
        for (int i = 1; i <= k_num_classes; i++) {
            votes.push_back(i);
        }
        std::shuffle(votes.begin(), votes.end(), gen);

        for (int j = 0; j < votes.size(); j++) {
            // Leave out some votes randomly
            if (dis(gen) > 0.1) {
                for (int k = 0; k <= votes.at(j); k++) {
                    classVoxelIncrementClass(class_voxel, j);
                }
            }
        }
    }
    // Randomly assign GT
    class_voxel->is_groundtruth = dis(gen) > 0.9;
    // Randomly assign uncertainty
    classVoxelUpdateUncertainty(class_voxel, dis(gen));
}

#endif //PANOPTIC_MAPPING_TEST_UTILS_H
