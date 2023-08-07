#include <hector_pose_prediction_benchmark/benchmark_suite.h>

namespace hector_pose_prediction_benchmark {

BenchmarkSuite::BenchmarkSuite(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh)
{
  loadParameters(pnh_);
  pose_predictor_ = createPosePredictor(pose_predictor_name_, pnh_);
  benchmark_ = std::make_shared<hector_pose_prediction_benchmark_tools::PosePredictionBenchmark>(pose_predictor_);
}

void BenchmarkSuite::runBenchmark()
{
  benchmark_->evaluateFromBag(bag_file_path_, false);

  if (!result_folder_.empty()) {
    std::string time_stamp = getTimestamp();
    std::string save_path = result_folder_ + "/" + pose_predictor_name_ + "-" + time_stamp + ".csv";
    benchmark_->saveToCsv(save_path);
    ROS_INFO_STREAM("Saved benchmark results to " << save_path);
  }
}

bool BenchmarkSuite::loadParameters(const ros::NodeHandle& nh)
{
  result_folder_ = nh.param("result_folder", std::string());
  if (!nh.getParam("bag_file_path", bag_file_path_)) {
    ROS_ERROR_STREAM("Mandatory parameter " << pnh_.getNamespace() << "/bag_file_path is missing");
    return false;
  }
  if (!nh.getParam("pose_predictor", pose_predictor_name_)) {
    ROS_ERROR_STREAM("Mandatory parameter " << pnh_.getNamespace() << "/pose_predictor is missing");
    return false;
  }
  return true;
}

std::string BenchmarkSuite::getTimestamp()
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return oss.str();
}

}
