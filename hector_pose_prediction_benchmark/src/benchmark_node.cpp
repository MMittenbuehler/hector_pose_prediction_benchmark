#include <ros/ros.h>

#include <hector_pose_prediction_benchmark/benchmark_suite.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_prediction_benchmark_suite");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  hector_pose_prediction_benchmark::BenchmarkSuite benchmark(nh, pnh);
  benchmark.runBenchmark();

  ros::spin();
  return 0;
}

