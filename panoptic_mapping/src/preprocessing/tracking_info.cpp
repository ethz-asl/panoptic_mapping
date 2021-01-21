#include "panoptic_mapping/preprocessing/tracking_info.h"

namespace panoptic_mapping {

TrackingInfo::TrackingInfo(int input_id) : input_id_(input_id) {}

void TrackingInfo::insertRenderedID(int rendered_id) {
  auto it = rendered_count.find(rendered_id);
  if (it == rendered_count.end()) {
    rendered_count[rendered_id] = 1;
  } else {
    it->second++;
  }
}

void TrackingInfos::insertIdPair(int input_id, int rendered_id) {
  // Let the infos track the input ids.
  auto it = infos_.find(input_id);
  if (it == infos_.end()) {
    it = infos_.emplace(input_id, TrackingInfo(input_id)).first;
  }
  it->second.insertRenderedID(rendered_id);
}

}  // namespace panoptic_mapping
