#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_BENCHMARK_SUITE_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_BENCHMARK_SUITE_H

#include <ros/ros.h>
#include <hector_pose_prediction_benchmark_tools/pose_prediction_benchmark.h>

#include <hector_pose_prediction_benchmark/pose_predictor_factory.h>


namespace hector_pose_prediction_benchmark {

class BenchmarkSuite
{
public:
  BenchmarkSuite(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  void runBenchmark();

private:
  bool loadParameters(const ros::NodeHandle& nh);
  std::string getTimestamp();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  hector_pose_prediction_benchmark_tools::PosePredictionBenchmark::Ptr benchmark_;

  // Parameters
  std::string pose_predictor_name_;
  std::string bag_file_path_;
  std::string result_folder_;
};

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_BENCHMARK_SUITE_H
