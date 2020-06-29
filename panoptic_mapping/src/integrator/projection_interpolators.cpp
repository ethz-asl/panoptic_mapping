#include "panoptic_mapping/integrator/projection_interpolators.h"

#include <cmath>

namespace panoptic_mapping {

std::unique_ptr<InterpolatorBase> InterpolatorFactory::create(const std::string &type) {
  if (type == "nearest") {
    return std::make_unique<InterpolatorNearest>();
  } else if (type == "bilinear") {
    return std::make_unique<InterpolatorBilinear>();
  } else if (type == "adaptive") {
    return std::make_unique<InterpolatorAdaptive>();
  } else if (type == "semantic") {
    return std::make_unique<InterpolatorSemantic>();
  } else {
    LOG(ERROR) << "Unknown interpolator type '" << type << "'.";
    return nullptr;
  }
}

void InterpolatorNearest::computeWeights(float u,
                                         float v,
                                         int id,
                                         bool *belongs_to_this_submap,
                                         const Eigen::MatrixXf &range_image,
                                         const cv::Mat &id_image) {
  u_ = std::round(u);
  v_ = std::round(v);
  *belongs_to_this_submap = id_image.at<cv::Vec3b>(v_, u_)[0] == id;
}

float InterpolatorNearest::interpolateDepth(const Eigen::MatrixXf &range_image) {
  return range_image(v_, u_);
}

Color InterpolatorNearest::interpolateColor(const cv::Mat &color_image) {
  auto color_bgr = color_image.at<cv::Vec3b>(v_, u_);
  return Color(color_bgr[2], color_bgr[1], color_bgr[0]);
}

void InterpolatorBilinear::computeWeights(float u,
                                          float v,
                                          int id,
                                          bool *belongs_to_this_submap,
                                          const Eigen::MatrixXf &range_image,
                                          const cv::Mat &id_image) {
  u_ = std::floor(u);
  v_ = std::floor(v);
  float du = u - (float)u_;
  float dv = v - (float)v_;
  weight_[0] = (1.f -du) * (1.f - dv);
  weight_[1] = (1.f -du) * dv;
  weight_[2] = du * (1.f -dv);
  weight_[3] = du * dv;
  float c1 = id_image.at<cv::Vec3b>(v_, u_)[0] == id ? 1.f : 0.f;
  float c2 = id_image.at<cv::Vec3b>(v_ + 1, u_)[0] == id ? 1.f : 0.f;
  float c3 = id_image.at<cv::Vec3b>(v_, u_ + 1)[0] == id ? 1.f : 0.f;
  float c4 = id_image.at<cv::Vec3b>(v_ + 1, u_ + 1)[0] == id ? 1.f : 0.f;
  *belongs_to_this_submap = c1 * weight_[0] + c2 * weight_[1] + c3 * weight_[2] + c4 * weight_[3] >= 0.5f;
}

float InterpolatorBilinear::interpolateDepth(const Eigen::MatrixXf &range_image) {
  return range_image(v_, u_) * weight_[0] + range_image(v_ + 1, u_) * weight_[1] +
      range_image(v_, u_ + 1) * weight_[2] + range_image(v_ + 1, u_ + 1) * weight_[3];
}

Color InterpolatorBilinear::interpolateColor(const cv::Mat &color_image) {
  Eigen::Vector3f color(0, 0, 0);
  auto c1 = color_image.at<cv::Vec3b>(v_, u_);
  auto c2 = color_image.at<cv::Vec3b>(v_ + 1, u_);
  auto c3 = color_image.at<cv::Vec3b>(v_, u_ + 1);
  auto c4 = color_image.at<cv::Vec3b>(v_ + 1, u_ + 1);
  for (size_t i = 0; i < 3; ++i) {
    color[i] = c1[i] * weight_[0] + c2[i] * weight_[1] + c3[i] * weight_[2] + c4[i] * weight_[3];
  }
  return Color(color[2], color[1], color[0]);   // bgr image
}

void InterpolatorAdaptive::computeWeights(float u,
                                          float v,
                                          int id,
                                          bool *belongs_to_this_submap,
                                          const Eigen::MatrixXf &range_image,
                                          const cv::Mat &id_image) {
  constexpr float max_depth_difference = 0.2; // m
  u_ = std::floor(u);
  v_ = std::floor(v);
  use_bilinear_ = true;

  // check all labels match
  for (size_t i = 0; i < 4; ++i) {
    if (id_image.at<cv::Vec3b>(v_ + v_offset_[i], u_ + u_offset_[i])[0] != id) {
      use_bilinear_ = false;
      break;
    }
  }

  // check max depth difference
  if (use_bilinear_) {
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
    for (size_t i = 0; i < 4; ++i) {
      float depth = range_image(v_ + v_offset_[i], u_ + u_offset_[i]) > max;
      if (depth > max) {
        max = depth;
      } else if (depth < min) {
        min = depth;
      }
    }
    if (max - min < max_depth_difference) {
      return InterpolatorBilinear::computeWeights(u, v, id, belongs_to_this_submap, range_image, id_image);
    }
    use_bilinear_ = false;
  }
  *belongs_to_this_submap = id_image.at<cv::Vec3b>(std::round(v), std::round(u))[0] == id;
}

float InterpolatorAdaptive::interpolateDepth(const Eigen::MatrixXf &range_image) {
  if (use_bilinear_) {
    return InterpolatorBilinear::interpolateDepth(range_image);
  }
  return range_image(v_, u_);
}

Color InterpolatorAdaptive::interpolateColor(const cv::Mat &color_image) {
  if (use_bilinear_) {
    return InterpolatorBilinear::interpolateColor(color_image);
  }
  auto color_bgr = color_image.at<cv::Vec3b>(v_, u_);
  return Color(color_bgr[2], color_bgr[1], color_bgr[0]);
}

void InterpolatorSemantic::computeWeights(float u,
                                          float v,
                                          int id,
                                          bool *belongs_to_this_submap,
                                          const Eigen::MatrixXf &range_image,
                                          const cv::Mat &id_image) {
  u_ = std::floor(u);
  v_ = std::floor(v);
  total_weight_ = 0.f;

  // check all labels
  for (size_t i = 0; i < 4; ++i) {
    if (id_image.at<cv::Vec3b>(v_ + v_offset_[i], u_ + u_offset_[i])[0] == id) {
      weight_[i] = 2.f - std::pow((float) v_ + (float) v_offset_[i] - v, 2.f)
          + std::pow((float) u_ + (float) u_offset_[i] - u, 2.f);
      total_weight_ += weight_[i];
    } else {
      weight_[i] = 0.f;
    }
  }
  if (total_weight_ == 0.f) {
    *belongs_to_this_submap = false;
    weight_[0] = 1.f;
    weight_[1] = 1.f;
    weight_[2] = 1.f;
    weight_[3] = 1.f;
    total_weight_ = 4.f;
  } else {
    *belongs_to_this_submap = true;
  }
}

float InterpolatorSemantic::interpolateDepth(const Eigen::MatrixXf &range_image) {
  float depth = 0;
  for (size_t i = 0; i < 4; ++i) {
    depth += weight_[i] * range_image(v_ + v_offset_[i], u_ + u_offset_[i]);
  }
  return depth / total_weight_;
}

Color InterpolatorSemantic::interpolateColor(const cv::Mat &color_image) {
  Eigen::Vector3f color(0, 0, 0);
  for (size_t i = 0; i < 4; ++i) {
    auto bgr = color_image.at<cv::Vec3b>(v_ + v_offset_[i], u_ + u_offset_[i]);
    for (size_t j = 0; j < 3; ++j) {
      color[j] += bgr[j] * weight_[i];
    }
  }
  return Color(color[2] / total_weight_, color[1] / total_weight_, color[0] / total_weight_);
}

}  // namespace panoptic_mapping