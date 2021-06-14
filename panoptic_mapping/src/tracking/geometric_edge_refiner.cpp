#include "panoptic_mapping/tracking/geometric_edge_refiner.h"

namespace panoptic_mapping {

void GeomtricEdgeRefiner::Config::checkParams() const {
  //  checkParamGT(voxels_per_side, 0, "voxels_per_side");
}

void GeomtricEdgeRefiner::Config::setupParamsAndPrinting() {
  //  setupParam("verbosity", &verbosity);
}

GeomtricEdgeRefiner::GeomtricEdgeRefiner(const Config& config)
    : config_(config.checkValid()) {}

void GeomtricEdgeRefiner::setup(const Camera::Config& camera) {
  normal_computer_.setCols(camera.width);
  normal_computer_.setRows(camera.height);
  normal_computer_.setDepth(CV_32F);
  // normal_computer_.setMethod(0);
  normal_computer_.setWindowSize(3);  // [1, 3, 5, 7]
  float data[9] = {camera.fx, 0.f, camera.vx, 0.f, camera.fy,
                   camera.vy, 0.f, 0.f,       1.f};
  cv::Mat k(3, 3, CV_32F, data);
  normal_computer_.setK(k);
  normals_ = cv::Mat(camera.height, camera.width, CV_32FC3);
}

void GeomtricEdgeRefiner::refinePrediction(const cv::Mat& depth_image,
                                           const cv::Mat& vertex_map,
                                           cv::Mat* id_image) {
  // Compute the normal map.
  //  cv::Mat depth_uint;
  //  depth_image.convertTo(depth_uint, CV_32FC3);
  //  normal_computer_(depth_image, normals_);
  normals_ = cv::Mat(depth_image.size(), CV_32FC3);
  //  for(int u= 1; u < vertex_map.cols-1; ++u)  {
  //    for(int v = 1; v < vertex_map.rows-1; ++v)    {
  ////      auto d = normals_.at<cv::Vec3f>(v, u);
  //
  //      const float dzdy = (depth_image.at<float>(u+1, v) -
  //      depth_image.at<float>(u-1, v)) / 2.f; const float dzdx =
  //      (depth_image.at<float>(u, v+1) - depth_image.at<float>(u, v-1)) / 2.f;
  //      const cv::Vec3f d(-dzdx, -dzdy, 1.0f);
  //      normals_.at<cv::Vec3f>(v, u) = cv::normalize(d);
  //      std::cout << u << ", " << v <<": " << d[0] << ", " << d[1] << ", " <<
  //      d[2] << std::endl;
  //    }
  //  }
}

}  // namespace panoptic_mapping
