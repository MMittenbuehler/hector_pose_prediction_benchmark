#include <hector_pose_prediction_benchmark_tools/pose_prediction_benchmark.h>

#include <hector_pose_prediction_benchmark_tools/util.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>
#include <fstream>

namespace hector_pose_prediction_benchmark_tools {
PosePredictionBenchmark::PosePredictionBenchmark(
    hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor)
: pose_predictor_(std::move(pose_predictor)) {}

void PosePredictionBenchmark::evaluate(const nav_msgs::Path& path, const std::vector<JointPositionMap>& joint_positions, bool init_from_previous)
{
  data_.clear();
  if (path.poses.size() != joint_positions.size()) {
    ROS_ERROR_STREAM("Path size (" << path.poses.size() << ") does not match joint position vector size (" << joint_positions.size() << ").");
    return;
  }

  bool first_iteration = true;
  hector_math::Pose<double> previous_predicted_pose;

  for (unsigned int i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::PoseStamped& pose_msg = path.poses[i];
    const JointPositionMap& joint_position = joint_positions[i];
    pose_predictor_->robotModel()->updateJointPositions(joint_position);
    DataPoint data_point;
    data_point.time = pose_msg.header.stamp;
    hector_math::Pose<double> pose = poseMsgToHectorMath(pose_msg.pose);

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
    Eigen::Vector3d rpy = rotationToEulerAngles(pose.asTransform().linear());
    if (!first_iteration && init_from_previous) {
      Eigen::Vector3d previous_rpy = rotationToEulerAngles(previous_predicted_pose.asTransform().linear());
      pose =  hector_math::Pose<double>(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ())
                                       * Eigen::AngleAxisd(previous_rpy(1), Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(previous_rpy(0), Eigen::Vector3d::UnitX()));
      pose.translation().z() = previous_predicted_pose.translation().z();
    } else {
      pose = hector_math::Pose<double>(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
      pose.translation().z() = 0; // arbitrary initialization
    }

    // Pose prediction
    data_point.input_pose = pose;
    data_point.predicted_stability = pose_predictor_->predictPoseAndContactInformation(pose, data_point.predicted_support_polygon, data_point.predicted_contact_information);
    data_point.predicted_pose = pose;
    previous_predicted_pose = pose;
    first_iteration = false;
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
    Eigen::Vector3d input_rpy = rotationToEulerAngles(data_point.input_pose.asTransform().linear());
    file << input_rpy(0) << ", ";
    file << input_rpy(1) << ", ";
    file << input_rpy(2) << ", ";
    file << data_point.estimated_stability << ", ";


    file << data_point.predicted_pose.translation().x() << ", ";
    file << data_point.predicted_pose.translation().y() << ", ";
    file << data_point.predicted_pose.translation().z() << ", ";
    Eigen::Vector3d prediction_rpy = rotationToEulerAngles(data_point.predicted_pose.asTransform().linear());
    file << prediction_rpy(0) << ", ";
    file << prediction_rpy(1) << ", ";
    file << prediction_rpy(2) << ", ";
    file << data_point.predicted_stability << std::endl;
  }

  file.close();
  return true;
}
void PosePredictionBenchmark::evaluateFromBag(const std::string& bag_path, bool init_from_previous)
{
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    ROS_ERROR_STREAM(e.what());
    return;
  }

  std::vector<std::string> topics{"/tf", "/joint_states"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::unordered_map<std::string, double> joint_position_map;
  std::set<std::string> missing_joint_states(pose_predictor_->robotModel()->jointNames().begin(),
                                             pose_predictor_->robotModel()->jointNames().end());
  nav_msgs::Path path;
  path.header.frame_id = "world";
  std::vector<JointPositionMap> joint_positions;
  for(const rosbag::MessageInstance& m: view) {
    sensor_msgs::JointState::ConstPtr joint_state_msg = m.instantiate<sensor_msgs::JointState>();
    updateJointPositionMap(joint_state_msg, joint_position_map, missing_joint_states);

    tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
    if (missing_joint_states.empty() && tf_msg) {
      for (const auto& transform_msg: tf_msg->transforms) {
        if (transform_msg.child_frame_id != "base_link" || transform_msg.header.frame_id != path.header.frame_id) {
          continue;
        }
        if (addPoseToPath(transform_msg, path, 0.05)) {
          joint_positions.push_back(joint_position_map);
        }
      }
    }
  }
  bag.close();

  evaluate(path, joint_positions, init_from_previous);
}

}
