#include <panoptic_mapping/3rd_party/config_utilities.hpp>

#include "panoptic_mapping_utils/evaluation/map_evaluator.h"

int main(int argc, char** argv) {
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Run ros.
  ros::init(argc, argv, "evaluation_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Evaluator.
  panoptic_mapping::MapEvaluator evaluator(nh, nh_private);

  // Process the request
  auto request = config_utilities::getConfigFromRos<
      panoptic_mapping::MapEvaluator::EvaluationRequest>(nh_private);
  evaluator.evaluate(request);

  // Visualize.
  if (request.visualize) {
    while (ros::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      evaluator.publishVisualization();
    }
  }
  return 0;
}
