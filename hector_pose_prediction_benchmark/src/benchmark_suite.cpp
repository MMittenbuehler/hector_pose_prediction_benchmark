#include <hector_pose_prediction_benchmark/benchmark_suite.h>

namespace hector_pose_prediction_benchmark {

BenchmarkSuite::BenchmarkSuite(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh)
{
  std::string pose_predictor_name;
  if (!pnh_.getParam("pose_predictor", pose_predictor_name)) {
    ROS_ERROR_STREAM("Mandatory parameter " << pnh_.getNamespace() << "/pose_predictor is missing");
  }
  pose_predictor_ = createPosePredictor(pose_predictor_name, pnh_);
}

void BenchmarkSuite::runBenchmark()
{

}

}
