#include "panoptic_mapping/preprocessing/detectron_id_tracker.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, DetectronIDTracker,
                                           std::shared_ptr<LabelHandler>>
    DetectronIDTracker::registration_("detectron");

void DetectronIDTracker::Config::checkParams() const {
  checkParamGT(voxels_per_side, 0, "voxels_per_side");
  checkParamGT(instance_voxel_size, 0.f, "instance_voxel_size");
  checkParamGT(background_voxel_size, 0.f, "background_voxel_size");
  checkParamGT(unknown_voxel_size, 0.f, "unknown_voxel_size");
  checkParamGT(freespace_voxel_size, 0.f, "freespace_voxel_size");
  checkParamGT(width, 0, "width");
  checkParamGT(heihgt, 0, "heihgt");
  checkParamGT(vx, 0.f, "vx");
  checkParamGT(vy, 0.f, "vy");
  checkParamGT(fx, 0.f, "fx");
  checkParamGT(fy, 0.f, "fy");
}

void DetectronIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("instance_voxel_size", &instance_voxel_size);
  setupParam("background_voxel_size", &background_voxel_size);
  setupParam("unknown_voxel_size", &unknown_voxel_size);
  setupParam("freespace_voxel_size", &freespace_voxel_size);
  setupParam("voxels_per_side", &voxels_per_side);
  setupParam("width", &width);
  setupParam("heihgt", &heihgt);
  setupParam("vx", &vx);
  setupParam("vy", &vy);
  setupParam("fx", &fx);
  setupParam("fy", &fy);
}

DetectronIDTracker::DetectronIDTracker(
    const Config& config, std::shared_ptr<LabelHandler> label_handler)
    : config_(config.checkValid()), IDTrackerBase(std::move(label_handler)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void DetectronIDTracker::processImages(SubmapCollection* submaps,
                                       const Transformation& T_M_C,
                                       const cv::Mat& depth_image,
                                       const cv::Mat& color_image,
                                       cv::Mat* id_image,
                                       const DetectronLabels& labels) {
  CHECK_NOTNULL(id_image);
  CV_Assert(id_image->depth() == CV_8U);
  std::cout << "Processing Detectron IDs." << std::endl;
  const cv::MatIterator_<uchar> begin = id_image->begin<uchar>();
  const cv::MatIterator_<uchar> end = id_image->end<uchar>();
  std::unordered_map<int, int> detectron_to_submap_id;

  if (!is_initialized_) {
    // If this is the first image don't track, all ids will be newly allocated.
    is_initialized_ = true;
  } else {
    // Track the image against the previous one.
    Transformation T_prev_is = T_M_C_prev_.inverse() * T_M_C;

    // Store the occurences of previous submap ids vs current detectron ids.
    // Note(schmluk): assumes depth and id images are from identical camera!
    std::unordered_map<int, std::unordered_map<int, int>> counts;
    std::unordered_map<int, int> detectron_id_occurences;
    for (int v = 0; v < depth_image.rows; v++) {
      for (int u = 0; u < depth_image.cols; u++) {
        Point p_is;
        p_is.z() = depth_image.at<float>(v, u);
        p_is.x() = (static_cast<float>(u) - config_.vx) * p_is.z() / config_.fx;
        p_is.y() = (static_cast<float>(v) - config_.vy) * p_is.z() / config_.fy;
        const Point p_prev = T_prev_is * p_is;
        int u_prev =
            std::round(config_.fx * p_prev.x() / p_prev.z() + config_.vx);
        if (u_prev < 0 || u_prev >= id_image_prev_.cols) {
          continue;
        }
        int v_prev =
            std::round(config_.fy * p_prev.y() / p_prev.z() + config_.vy);
        if (v_prev < 0 || v_prev >= id_image_prev_.rows) {
          continue;
        }

        // Store the found id.
        int detectron_id = static_cast<int>(id_image->at<uchar>(v, u));
        int prev_id =
            static_cast<int>(id_image_prev_.at<uchar>(v_prev, u_prev));
        auto it = detectron_id_occurences.find(detectron_id);
        if (it == detectron_id_occurences.end()) {
          detectron_id_occurences[detectron_id] = 1;
        } else {
          (it->second)++;
        }

        auto it2 = counts.find(detectron_id);
        if (it2 == counts.end()) {
          it2 = counts.emplace(detectron_id, std::unordered_map<int, int>())
                    .first;
        }

        auto it3 = it2->second.find(prev_id);
        if (it3 == it2->second.end()) {
          it2->second[prev_id] = 1;
        } else {
          (it3->second)++;
        }
      }
    }

    // Lookup the submap with highest overlap and matching panoptic and class
    // id.
    int number_of_matched_ids = 0;
    for (const auto& id_occurences : counts) {
      std::vector<std::pair<int, int>> occurences;
      occurences.reserve(id_occurences.second.size());
      for (const auto& occ : id_occurences.second) {
        occurences.emplace_back(occ);
      }
      std::sort(
          occurences.begin(), occurences.end(),
          [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
            return lhs.second > rhs.second;
          });

      // Find the best match.
      for (const auto& occ : occurences) {
        const Submap& submap = submaps->getSubmap(occ.first);
        auto it = labels.find(id_occurences.first);
        if (it == labels.end()) {
          LOG(WARNING) << "Could not find a detectron label for id "
                       << id_occurences.first << ".";
          break;
        }

        // Check class and panoptic label.
        if (it->second.category_id != submap.getClassID()) {
          continue;
        }
        if ((!it->second.is_thing &&
             submap.getLabel() != PanopticLabel::kBackground) ||
            (it->second.is_thing &&
             submap.getLabel() != PanopticLabel::kInstance)) {
          continue;
        }

        // Store match.
        detectron_to_submap_id[id_occurences.first] = submap.getID();
        number_of_matched_ids++;
        if (config_.verbosity >= 4) {
          // NOTE(schmluk): better use IoU here instead of highest overlap?
          float percentage =
              100.f * static_cast<float>(occ.second) /
              static_cast<float>(detectron_id_occurences[id_occurences.first]);
          LOG(INFO) << "Matched detectron id " << id_occurences.first << " ("
                    << std::fixed << std::setprecision(2) << percentage
                    << "%).";
        }
        break;
      }
    }
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Matched " << number_of_matched_ids << "/"
        << detectron_id_occurences.size()
        << " detectron ids with the previous frame.";
  }

  // Write the new ids to the id image and allocate submaps.
  for (auto it = begin; it != end; ++it) {
    int detectron_id = static_cast<int>(*it);
    auto id_it = detectron_to_submap_id.find(detectron_id);
    if (id_it == detectron_to_submap_id.end()) {
      int submap_id = allocateSubmap(detectron_id, submaps, labels);
      detectron_to_submap_id[detectron_id] = submap_id;
      *it = static_cast<uchar>(submap_id);
    } else {
      *it = static_cast<uchar>(id_it->second);
    }
  }
  printAndResetWarnings();


  // Allocate free space map if required.
  allocateFreeSpaceSubmap(submaps);

  // Store data.
  T_M_C_prev_ = T_M_C;
  id_image_prev_ = *id_image;
}

void DetectronIDTracker::processImages(SubmapCollection* submaps,
                                       const Transformation& T_M_C,
                                       const cv::Mat& depth_image,
                                       const cv::Mat& color_image,
                                       cv::Mat* id_image) {
  LOG(WARNING) << "The detectron id tracker can only be used with labels.";
}

void DetectronIDTracker::processPointcloud(SubmapCollection* submaps,
                                           const Transformation& T_M_C,
                                           const Pointcloud& pointcloud,
                                           const Colors& colors,
                                           std::vector<int>* ids) {
  LOG(WARNING) << "The detectron id tracker can only be used with labels.";
}

int DetectronIDTracker::allocateSubmap(int detectron_id,
                                       SubmapCollection* submaps,
                                       const DetectronLabels& labels) {
  // Check whether the instance code is known.
  auto it = labels.find(detectron_id);
  PanopticLabel pan_label = PanopticLabel::kUnknown;
  if (it == labels.end()) {
    auto error_it = unknown_ids.find(detectron_id);
    if (error_it == unknown_ids.end()) {
      unknown_ids[detectron_id] = 1;
    } else {
      unknown_ids[detectron_id] += 1;
    }
  } else {
    if (it->second.is_thing) {
      pan_label = PanopticLabel::kInstance;
    } else {
      pan_label = PanopticLabel::kBackground;
    }
  }

  // Allocate new submap.
  Submap::Config cfg;
  cfg.voxels_per_side = config_.voxels_per_side;
  switch (pan_label) {
    case PanopticLabel::kInstance: {
      cfg.voxel_size = config_.instance_voxel_size;
      break;
    }
    case PanopticLabel::kBackground: {
      cfg.voxel_size = config_.background_voxel_size;
      break;
    }
    case PanopticLabel::kFreeSpace: {
      cfg.voxel_size = config_.freespace_voxel_size;
      break;
    }
    case PanopticLabel::kUnknown: {
      cfg.voxel_size = config_.unknown_voxel_size;
      break;
    }
  }
  Submap* new_submap = submaps->createSubmap(cfg);
  new_submap->setLabel(pan_label);
  // TODO(schmluk): add proper data.
  if (pan_label != PanopticLabel::kUnknown) {
    new_submap->setClassID(it->second.category_id);
  } else {
    // new_submap->setClassID(-1);
  }
  // new_submap->setInstanceID(new_instance);
  // new_submap->setName(label_handler_->getName(new_instance));
  return new_submap->getID();
}

void DetectronIDTracker::allocateFreeSpaceSubmap(SubmapCollection* submaps) {
  if (submaps->getActiveFreeSpaceSubmapID() >= 0) {
    // Currently only allocate one free space submap in the beginning.
    return;
  }

  // Create a new freespace submap.
  Submap::Config config;
  config.voxels_per_side = config_.voxels_per_side;
  config.voxel_size = config_.freespace_voxel_size;
  Submap* space_submap = submaps->createSubmap(config);
  space_submap->setLabel(PanopticLabel::kFreeSpace);
  space_submap->setInstanceID(-1);  // Will never appear in a seg image.
  space_submap->setName("FreeSpace");
  submaps->setActiveFreeSpaceSubmapID(space_submap->getID());
}

void DetectronIDTracker::printAndResetWarnings() {
  for (auto it : unknown_ids) {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Encountered " << it.second
        << " occurences of unknown segmentation ID '" << it.first << "'.";
  }
  unknown_ids.clear();
}

}  // namespace panoptic_mapping
