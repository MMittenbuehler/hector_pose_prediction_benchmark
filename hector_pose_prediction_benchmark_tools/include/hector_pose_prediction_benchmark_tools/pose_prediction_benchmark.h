#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_POSEPREDICTIONBENCHMARK_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_POSEPREDICTIONBENCHMARK_H

#include <hector_pose_prediction_interface/pose_predictor.h>
#include <nav_msgs/Path.h>

#include <hector_pose_prediction_benchmark_tools/data_point.h>
#include <hector_pose_prediction_benchmark_tools/types.h>
#include <ros/publisher.h>
#include <moveit/robot_model/robot_model.h>
#include <visualization_msgs/MarkerArray.h>

namespace hector_pose_prediction_benchmark_tools {

class PosePredictionBenchmark {
public:
  explicit PosePredictionBenchmark(hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor);
  void evaluate(const nav_msgs::Path& path, const std::vector<JointPositionMap>& joint_positions, bool init_from_previous=false);
  void evaluateFromBag(const std::string& bag_path, double path_resolution=0.05, bool init_from_previous=false);
  bool saveToCsv(const std::string& csv_file_path) const;

  [[nodiscard]] double getWaitTime() const;
  void setWaitTime(double seconds);

  typedef std::shared_ptr<PosePredictionBenchmark> Ptr;
private:
  void addRobotMarkerToArray(const hector_math::Pose<double>& pose,
                             const JointPositionMap& joint_positions,
                             const Eigen::Vector4f& rgba,
                             const std::string& ns,
                             visualization_msgs::MarkerArray& array);

  hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor_;
  std::vector<DataPoint> data_;

  robot_model::RobotModelPtr robot_model_;
  ros::Publisher all_robot_states_pub_;
  ros::Publisher current_robot_state_pub_;

  double wait_time_;
};

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_POSEPREDICTIONBENCHMARK_H
