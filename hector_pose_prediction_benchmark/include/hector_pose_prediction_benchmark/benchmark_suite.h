#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_BENCHMARK_SUITE_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_BENCHMARK_SUITE_H

#include <ros/ros.h>
#include <hector_pose_prediction_benchmark/pose_predictor_factory.h>

namespace hector_pose_prediction_benchmark {

class BenchmarkSuite
{
public:
  BenchmarkSuite(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  void runBenchmark();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
};

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_BENCHMARK_SUITE_H
