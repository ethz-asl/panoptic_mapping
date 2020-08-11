#include "panoptic_mapping/integrator/integrator_base.h"

namespace panoptic_mapping {

void IntegratorBase::processPointcloud(SubmapCollection *submaps,
                                       const Transformation &T_M_C,
                                       const Pointcloud &pointcloud,
                                       const Colors &colors,
                                       const std::vector<int> &ids) {
  LOG(WARNING) << "This TSDF integrator does not support integrating point clouds.";
}

void IntegratorBase::processImages(SubmapCollection *submaps,
                                   const Transformation &T_M_C,
                                   const cv::Mat &depth_image,
                                   const cv::Mat &color_image,
                                   const cv::Mat &id_image) {
  LOG(WARNING) << "This TSDF integrator does not support integrating images.";
}

}  // namespace panoptic_mapping