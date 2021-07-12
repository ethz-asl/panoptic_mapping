#ifndef PANOPTIC_MAPPING_UTILS_EVALUATION_MAP_EVALUATOR_H_
#define PANOPTIC_MAPPING_UTILS_EVALUATION_MAP_EVALUATOR_H_

#include <memory>
#include <string>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/3rd_party/nanoflann.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping/tools/planning_interface.h>
#include <panoptic_mapping_ros/visualization/submap_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <voxblox_ros/tsdf_server.h>

namespace panoptic_mapping {

// Evaluation tools in a ROS node for visualization. Largely based on the
// voxblox_ros/voxblox_eval.cc code.
class MapEvaluator {
 public:
  struct EvaluationRequest
      : public config_utilities::Config<EvaluationRequest> {
    int verbosity = 4;

    // Data handling.
    std::string map_file;
    std::string ground_truth_pointcloud_file;

    // Evaluation
    float maximum_distance = 0.2;  // m
    bool visualize = true;
    bool evaluate = true;
    bool compute_coloring = false;  // Use map_file to load and display.
    bool ignore_truncated_points = false;
    bool color_by_max_error = false;  // false: color by average error

    EvaluationRequest() { setConfigName("MapEvaluator::EvaluationRequest"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // nanoflann pointcloud adapter.
  struct TreeData {
    std::vector<Point> points;

    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
      if (dim == 0)
        return points[idx].x();
      else if (dim == 1)
        return points[idx].y();
      else
        return points[idx].z();
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
      return false;
    }
  };
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, TreeData>, TreeData, 3>
      KDTree;

  // Constructor.
  MapEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~MapEvaluator() = default;

  // Access.
  bool evaluate(const EvaluationRequest& request);
  void publishVisualization();

 private:
  void computeReconstructionError(const EvaluationRequest& request);
  void visualizeReconstructionError(const EvaluationRequest& request);
  void computeErrorHistogram(
      const EvaluationRequest& request);  // Placeholder atm

 private:
  // ROS.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Files.
  std::ofstream output_file_;

  // Stored data.
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> gt_ptcloud_;
  std::shared_ptr<SubmapCollection> submaps_;
  std::shared_ptr<TsdfLayer> voxblox_;
  bool use_voxblox_;
  std::string target_directory_;
  std::string target_map_name_;
  std::unique_ptr<PlanningInterface> planning_;
  std::unique_ptr<SubmapVisualizer> visualizer_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_UTILS_EVALUATION_MAP_EVALUATOR_H_
