#include <hector_pose_prediction_benchmark_tools/pose_prediction_benchmark.h>

#include <hector_pose_prediction_benchmark_tools/util.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <ros/console.h>
#include <fstream>

namespace hector_pose_prediction_benchmark_tools {
PosePredictionBenchmark::PosePredictionBenchmark(
    hector_pose_prediction_interface::PosePredictor<double>::Ptr  pose_predictor)
: pose_predictor_(std::move(pose_predictor)) {}

void PosePredictionBenchmark::evaluate(const nav_msgs::Path& path)
{
  data_.clear();

  for (const auto& pose_msg: path.poses) {
    DataPoint data_point;
    data_point.time = pose_msg.header.stamp;
    hector_math::Pose<double> pose = poseMsgToHectorMath(pose_msg.pose);
    // TODO set joint state
    // Estimate stability from input pose
    if (pose_predictor_->estimateContactInformation(pose, data_point.estimated_support_polygon, data_point.estimated_contact_information)
        && !data_point.estimated_support_polygon.contact_hull_points.empty()) {
      Eigen::Vector3d com_base = pose_predictor_->robotModel()->centerOfMass();
      Eigen::Vector3d com_world = pose * com_base;
      Eigen::Vector3d gravity(0, 0, -9.81);
      data_point.estimated_stability = hector_stability_metrics::non_differentiable::computeForceAngleStabilityMeasureValue(data_point.estimated_support_polygon.contact_hull_points, data_point.estimated_support_polygon.edge_stabilities, com_world, gravity);
    } else {
      data_point.estimated_stability = std::nan("");
    }


    // Erase pose information, that we want to estimate
    pose.translation().z() = 0;
    // TODO orientation

    data_point.input_pose = pose;
    data_point.predicted_stability = pose_predictor_->predictPoseAndContactInformation(pose, data_point.predicted_support_polygon, data_point.predicted_contact_information);
    data_point.predicted_pose = pose;
    data_.push_back(std::move(data_point));
  }
}

bool PosePredictionBenchmark::saveToCsv(const std::string& csv_file_path) const
{
  if (!createParentDirectory(csv_file_path)) {
    return false;
  }

  std::ofstream file;
  file.open(csv_file_path);

  file << "time,"
       << "input_position_x,"
       << "input_position_y,"
       << "input_position_z,"
       << "input_orientation_roll,"
       << "input_orientation_pitch,"
       << "input_orientation_yaw,"
       << "estimated_stability"
       << "predicted_position_x,"
       << "predicted_position_y,"
       << "predicted_position_z,"
       << "predicted_orientation_roll,"
       << "predicted_orientation_pitch,"
       << "predicted_orientation_yaw,"
       << "predicted_stability"
       << std::endl;
  for (const auto& data_point: data_) {
    file << data_point.time.toSec() << ", ";
    file << data_point.input_pose.translation().x() << ", ";
    file << data_point.input_pose.translation().y() << ", ";
    file << data_point.input_pose.translation().z() << ", ";
    file << data_point.estimated_stability << ", ";

    file << data_point.predicted_pose.translation().x() << ", ";
    file << data_point.predicted_pose.translation().y() << ", ";
    file << data_point.predicted_pose.translation().z() << ", ";
    file << data_point.predicted_stability << std::endl;
  }

  file.close();
  return false;
}

}
