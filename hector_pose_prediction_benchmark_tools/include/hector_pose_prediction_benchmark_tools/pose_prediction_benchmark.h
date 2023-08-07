#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_POSEPREDICTIONBENCHMARK_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_POSEPREDICTIONBENCHMARK_H

#include <hector_pose_prediction_interface/pose_predictor.h>
#include <nav_msgs/Path.h>

#include <hector_pose_prediction_benchmark_tools/data_point.h>

namespace hector_pose_prediction_benchmark_tools {

class PosePredictionBenchmark {
public:
  explicit PosePredictionBenchmark(hector_pose_prediction_interface::PosePredictor<double>::Ptr  pose_predictor);
  void evaluate(const nav_msgs::Path& path);
  bool saveToCsv(const std::string& csv_file_path) const;
private:
  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  std::vector<DataPoint> data_;
};

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_POSEPREDICTIONBENCHMARK_H
